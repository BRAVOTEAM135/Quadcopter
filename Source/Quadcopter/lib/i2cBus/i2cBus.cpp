#include "i2cBus.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/*
 * READING GUIDE
 *
 * This driver gives the TWI hardware one instruction at a time. Hardware clocks
 * the bits on SDA/SCL; software handles the next step when an interrupt arrives.
 * The program can run other tasks between those interrupts.
 *
 * There are three parts to a transaction:
 *   start...() / readRegisters() / writeRegister(): save the request, send START.
 *   ISR(TWI_vect): react to each hardware event and arrange the next byte.
 *   service() + take...(): finish STOP/timeout handling, then collect the result.
 *
 * The bus sequences are:
 *   Probe: START -> address+WRITE -> STOP
 *   Write: START -> address+WRITE -> register -> value -> STOP
 *   Read:  START -> address+WRITE -> register -> repeated START
 *               -> address+READ -> received bytes -> STOP
 *
 * A register read first WRITES the register number to tell the sensor which
 * bytes we want. The repeated START switches direction without releasing the
 * bus. During reception, we ACK each byte except the last; the final NACK tells
 * the sensor "that was the last byte I wanted" and is normal, not an error.
 *
 * Hardware register names used below:
 *   TWCR: control - request START/STOP, continue a transfer, enable interrupts.
 *   TWSR: status  - tells the interrupt handler what just happened.
 *   TWDR: data    - the byte to transmit, or the byte just received.
 *   TWBR: bitrate - helps set the SCL clock frequency.
 *
 * (1 << TWEN) makes a mask selecting the TWEN bit. The | operators combine
 * masks so a single assignment to TWCR sets the required control bits.
 *
 * Hardware reference: ATmega328P datasheet, "2-wire Serial Interface":
 * https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 */

// This unnamed namespace keeps implementation details private to this file.
// The state is shared by the single physical TWI peripheral, not per i2cBus object.
namespace {
    constexpr uint32_t I2C_FREQUENCY = 100000UL;
    constexpr uint32_t I2C_TIMEOUT_US = 5000UL;
    // Three different questions, rather than one huge list of combined states:
    // State:     is the overall transaction free, active, stopping, or finished?
    // Operation: what did the caller ask for?
    // Phase:     which part of its byte sequence are we currently handling?
    //
    // Normal lifecycle:
    // Idle -> Busy -> Stopping -> Complete -> Idle
    //                 service() checks STOP   take...() collects the result
    // Timeouts and lost arbitration can go directly from Busy to Complete.
    enum class State : uint8_t { Closed, Idle, Busy, Stopping, Cancelling, Complete };
    enum class Operation : uint8_t { Probe, Read, Write };
    // Register includes the initial write-address/register-number portion.
    // Value means the write value is in flight. Reading includes the repeated
    // START, read address, and incoming data bytes.
    enum class Phase : uint8_t { Register, Value, Reading };

    // These variables are accessed by both normal code and the interrupt.
    // volatile makes the compiler perform the memory accesses rather than
    // assuming the value cannot change unexpectedly. It does not by itself
    // make an update safe against interruption; ATOMIC_BLOCK handles that below.
    volatile State state = State::Closed;
    volatile Operation operation = Operation::Probe;
    volatile Phase phase = Phase::Register;
    volatile i2cBus::Result transferResult = i2cBus::Result::BusError;
    volatile uint8_t address = 0;
    volatile uint8_t registerAddress = 0;
    volatile uint8_t writeValue = 0;
    // A volatile POINTER to the caller's array. The interrupt writes incoming
    // bytes directly into that array; there is no extra internal receive copy.
    uint8_t* volatile readBuffer = nullptr;
    volatile uint8_t readLength = 0;
    volatile uint8_t readIndex = 0;
    // These two are used only by normal code, so they do not need volatile.
    const void* transactionOwner = nullptr;
    uint32_t transactionStartUs = 0;

    // Let hardware perform its next step. For reception, acknowledge selects
    // the response to the NEXT received byte: true = ACK, false = NACK.
    // TWEN keeps TWI enabled; TWIE requests an interrupt at the next event.
    // TWINT is unusual: writing 1 CLEARS its flag and releases the hardware
    // to continue. All TWDR updates must therefore happen before this call.
    /// Release TWI to perform its next byte step; acknowledge selects ACK or NACK for
    /// reception.
    void resume(bool acknowledge = false) {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) |
               (acknowledge ? (1 << TWEA) : 0);
    }

    // Remember the outcome and request STOP. This does not wait for STOP to
    // finish. Leave the transaction Stopping for service() to check later.
    // Omitting TWIE disables TWI interrupts during this final step.
    /// Record the transfer result and request STOP without waiting for STOP completion.
    void stop(i2cBus::Result result) {
        transferResult = result;
        state = State::Stopping;
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    }

    // Shared setup for all three public request functions. Different operations
    // use different parameters: a probe needs only an address, a read needs a
    // receive buffer, and a write needs one value.
    /// Validate and publish a transfer request atomically, then send START; return false if
    /// busy, closed or invalid.
    bool start(Operation requestedOperation, uint8_t deviceAddress,
               uint8_t reg, uint8_t value, uint8_t* data, uint8_t length,
               uint32_t nowUs, const void* owner) {
        // Reject the I2C address ranges reserved for special protocol purposes.
        if (deviceAddress < 0x08 || deviceAddress > 0x77) {
            return false;
        }
        bool accepted = false;
        // Briefly disable interrupts while publishing the request. Otherwise
        // the ISR could see a mixture of old and new transaction settings.
        // ATOMIC_RESTORESTATE restores the previous interrupt enable state.
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (state == State::Idle) {
                address = deviceAddress;
                registerAddress = reg;
                writeValue = value;
                readBuffer = data;
                readLength = length;
                readIndex = 0;
                operation = requestedOperation;
                phase = Phase::Register;
                transactionOwner = owner;
                transactionStartUs = nowUs;
                state = State::Busy;
                // TWSTA asks hardware to send START. Return immediately:
                // the 0x08 interrupt below continues the transaction later.
                TWCR = (1 << TWINT) | (1 << TWSTA) |
                       (1 << TWEN) | (1 << TWIE);
                accepted = true;
            }
        }
        return accepted;
    }
}

/// Configure and enable TWI at 100 kHz; call only when no transfer or unread result is
/// outstanding.
void i2cBus::init() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TWCR = 0; // Disable TWI while configuring it.
        TWSR = 0; // Select bitrate prescaler = 1.
        // Datasheet: SCL = F_CPU / (16 + 2 * TWBR * prescaler).
        // At 16 MHz and prescaler 1, TWBR = 72 gives a 100 kHz SCL clock.
        TWBR = static_cast<uint8_t>(((F_CPU / I2C_FREQUENCY) - 16UL) / 2UL);
        TWCR = (1 << TWEN); // Enable TWI; no transaction/interrupts yet.
        state = State::Idle;
        transactionOwner = nullptr;
        readBuffer = nullptr;
    }
}

/// Request an address-acknowledgement probe; true means accepted, not acknowledged yet.
bool i2cBus::startProbe(uint8_t deviceAddress, uint32_t nowUs) {
    // Unused register/value/buffer arguments are zero/null for a probe.
    return start(Operation::Probe, deviceAddress, 0, 0, nullptr, 0, nowUs, nullptr);
}

/// Disable TWI and its interrupts, abort all transfers and discard results; close sensor
/// drivers first.
void i2cBus::close() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TWCR = 0; // Disable the peripheral and its interrupt before releasing buffers.
        state = State::Closed;
        readBuffer = nullptr;
        transactionOwner = nullptr;
        readLength = 0;
        readIndex = 0;
    }
}

/// Cancel the matching owner's transfer and stop writes to its receive buffer; true means
/// cancelled. Continue service() to finish STOP or its timeout.
bool i2cBus::cancel(const void* owner) {
    bool cancelled = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (state != State::Closed && state != State::Idle &&
            state != State::Cancelling && operation != Operation::Probe &&
            transactionOwner == owner) {
            // Disable byte interrupts immediately. A finished transfer needs
            // no STOP; an active transfer releases the bus asynchronously.
            if (state == State::Complete) {
                state = State::Idle;
            } else {
                state = State::Cancelling;
                TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
            }
            readBuffer = nullptr;
            transactionOwner = nullptr;
            readLength = 0;
            readIndex = 0;
            cancelled = true;
        }
    }
    return cancelled;
}

/// Take an atomic status snapshot without advancing or consuming a transaction.
DriverStatus i2cBus::getStatus() const {
    DriverStatus status = {false, false, DriverError::None};
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        status.ready = state != State::Closed;
        status.busy = state != State::Closed && state != State::Idle;
        if (state == State::Complete) {
            switch (transferResult) {
                case Result::Ack: break;
                case Result::Nack: status.error = DriverError::Nack; break;
                case Result::Timeout: status.error = DriverError::Timeout; break;
                default: status.error = DriverError::BusError; break;
            }
        }
    }
    return status;
}

/// Start a 1..32-byte register read; true means accepted. Keep the receive buffer valid until
/// collection or cancellation.
bool i2cBus::readRegisters(uint8_t deviceAddress, uint8_t reg,
                           uint8_t* data, uint8_t length, uint32_t nowUs,
                           const void* owner) {
    // Reject requests the interrupt handler could not safely fill.
    if (data == nullptr || length == 0 || length > 32) {
        return false;
    }
    return start(Operation::Read, deviceAddress, reg, 0, data, length, nowUs, owner);
}

/// Start a single-byte register write; true means accepted. The value is copied, so no transmit
/// buffer is needed.
bool i2cBus::writeRegister(uint8_t deviceAddress, uint8_t reg,
                           uint8_t value, uint32_t nowUs, const void* owner) {
    return start(Operation::Write, deviceAddress, reg, value, nullptr, 0, nowUs, owner);
}

/// Check STOP completion, cancellation and the 5 ms timeout without waiting for hardware.
void i2cBus::service(uint32_t nowUs) {
    // This runs in loop(), not in the ISR. Keep the block short: it only checks
    // state and registers; it never waits for hardware to finish.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Hardware clears TWSTO after STOP. STOP does not produce the usual
        // completion interrupt, which is why this check lives in service().
        if ((state == State::Stopping || state == State::Cancelling) &&
            (TWCR & (1 << TWSTO)) == 0) {
            state = state == State::Cancelling ? State::Idle : State::Complete;
        }
        // Unsigned subtraction still measures elapsed time across the 32-bit
        // clock's wraparound. Do not time out an already completed result.
        if ((state == State::Busy || state == State::Stopping || state == State::Cancelling) &&
            static_cast<uint32_t>(nowUs - transactionStartUs) >= I2C_TIMEOUT_US) {
            // Abort by resetting the TWI peripheral. This releases its control
            // of the pins; it cannot force a faulty sensor to release SDA/SCL.
            TWCR = 0;
            TWCR = (1 << TWEN);
            transferResult = Result::Timeout;
            state = state == State::Cancelling ? State::Idle : State::Complete;
        }
    }
}

/// Collect a completed probe once and release the bus; false means no probe result is ready.
bool i2cBus::takeProbeResult(ProbeResult& result) {
    bool available = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (state == State::Complete && operation == Operation::Probe) {
            result = transferResult;
            state = State::Idle; // Only now may a new request replace this one.
            available = true;
        }
    }
    return available;
}

/// Collect a finished register transfer for the same owner and release the bus; false means
/// unfinished or wrong owner.
bool i2cBus::takeResult(Result& result, const void* owner) {
    bool available = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Keep another driver's result reserved for that driver. Checking the
        // owner pointer does not read the object it points to; it is just an ID.
        if (state == State::Complete && operation != Operation::Probe &&
            transactionOwner == owner) {
            result = transferResult;
            readBuffer = nullptr; // The driver no longer needs the caller's array.
            transactionOwner = nullptr;
            state = State::Idle;
            available = true;
        }
    }
    return available;
}

// HARDWARE EVENT HANDLER
// Entered automatically when TWI completes a START, an address, or a data byte.
// Each case arranges the next small step and returns; no waiting or printing.
/// Handle one TWI hardware event and arrange the next transfer step without waiting.
ISR(TWI_vect) {
    if (state == State::Closed) {
        TWCR = 0;
        return;
    }
    if (state == State::Cancelling) {
        // Leave STOP in progress; no cancelled byte may touch the old buffer.
        return;
    }
    if (state != State::Busy) {
        // No active transfer should need an interrupt. Clear the flag and
        // disable further TWI interrupts, leaving the peripheral enabled.
        TWCR = (1 << TWINT) | (1 << TWEN);
        return;
    }
    // The upper five bits of TWSR are a hardware-defined status code. 0xF8
    // masks off the lower bits, which include the bitrate prescaler settings.
    // These hex numbers come from the datasheet, not our own state numbering.
    switch (TWSR & 0xF8) {
        case 0x08: // START.
        case 0x10: // Repeated START for the read half of a register read.
            // Pack the 7-bit address into bits 7..1. Bit 0 chooses direction:
            // 0 = write, 1 = read. Example: address 0x53 becomes 0xA6 or 0xA7.
            TWDR = static_cast<uint8_t>((address << 1) |
                                       (phase == Phase::Reading ? 1 : 0));
            resume();
            break;
        case 0x18: // SLA+W acknowledged.
            // SLA+W means "slave address plus write bit". A probe has answered
            // its only question already; register operations must now send reg.
            if (operation == Operation::Probe) {
                stop(i2cBus::Result::Ack);
            } else {
                TWDR = registerAddress;
                resume();
            }
            break;
        case 0x28: // Transmitted register address or value acknowledged.
            // The SAME status is used after either transmitted byte, so we
            // inspect operation + phase to decide which byte comes next.
            if (operation == Operation::Read && phase == Phase::Register) {
                // Register number accepted. Keep the bus and change to reading.
                phase = Phase::Reading;
                TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
            } else if (operation == Operation::Write && phase == Phase::Register) {
                // Register number accepted. Now send its new value.
                phase = Phase::Value;
                TWDR = writeValue;
                resume();
            } else if (operation == Operation::Write && phase == Phase::Value) {
                // The new value was accepted: the write is finished.
                stop(i2cBus::Result::Ack);
            } else {
                stop(i2cBus::Result::BusError);
            }
            break;
        case 0x40: // SLA+R acknowledged.
            // Address accepted for reading. With one byte requested, arrange
            // NACK immediately. With more bytes, arrange ACK for the first one.
            if (operation == Operation::Read && phase == Phase::Reading) {
                resume(readLength > 1);
            } else {
                stop(i2cBus::Result::BusError);
            }
            break;
        case 0x50: // Received a byte, sent ACK; at least one byte must remain.
            // Check the expected operation and remaining capacity before
            // writing through the caller's pointer.
            if (operation != Operation::Read || phase != Phase::Reading ||
                readIndex >= readLength - 1) {
                stop(i2cBus::Result::BusError);
                break;
            }
            readBuffer[readIndex++] = TWDR;
            // After storing this byte, decide the response to the NEXT byte.
            // If only the final byte remains, switch to NACK now.
            resume(readIndex < readLength - 1);
            break;
        case 0x58: // Received final byte, sent NACK.
            // Our own final NACK is intentional. Store the last byte and report
            // success; a sensor refusing an address/data byte is handled below.
            if (operation != Operation::Read || phase != Phase::Reading ||
                readIndex != readLength - 1) {
                stop(i2cBus::Result::BusError);
                break;
            }
            readBuffer[readIndex++] = TWDR;
            stop(i2cBus::Result::Ack);
            break;
        case 0x20: // SLA+W NACK.
        case 0x30: // Register/value NACK.
        case 0x48: // SLA+R NACK.
            // Here the SENSOR sent NACK. End the transaction and report it.
            stop(i2cBus::Result::Nack);
            break;
        case 0x38: // Arbitration lost: another master owns the bus; no STOP.
            // Release the peripheral's participation without disrupting the
            // winning master. This driver reports the error rather than retrying.
            transferResult = i2cBus::Result::BusError;
            TWCR = (1 << TWINT) | (1 << TWEN);
            state = State::Complete;
            break;
        default:
            // Covers bus error 0x00 and any state we cannot handle. Request
            // STOP/recovery and let service() observe TWSTO clearing. For 0x00,
            // hardware releases the bus and clears TWSTO without sending STOP.
            stop(i2cBus::Result::BusError);
            break;
    }
}

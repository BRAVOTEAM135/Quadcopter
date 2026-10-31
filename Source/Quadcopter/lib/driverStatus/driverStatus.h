#ifndef DRIVER_STATUS_H
#define DRIVER_STATUS_H

#include <stdint.h>

enum class DriverError : uint8_t { None, Nack, BusError, Timeout, WrongDevice, InvalidData };

// A snapshot only: getStatus() never starts or advances hardware activity.
struct DriverStatus {
    bool ready; // Initialization finished; a fresh measurement may still be pending.
    bool busy;  // Initialization or an operation is in progress.
    DriverError error;
};

#endif

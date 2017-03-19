/*
 * Header File Rules
 * -----------------
 * - DO included all function prototyes that are public. E.g. adc_init(), adc_config(), adc_read()
 * - DON'T  include private functions. E.g. adc_scale()
 * - DON'T include executable lines of code or variable declarations
 * - DON'T expose any variables in a header - pass variables by reference E.g. void swap(int *x, int *y);
 * - Comments in here are for the user of the libary
 * 
 * Header File Comments:
 * ---------------------
 *
 *
 *
 */
#include <Arduino.h>
/* Protect the header file from multiple inclusions */
 #ifndef RADIO_H_
 #define RADIO_H_

/*----- #include ... -----*/

/*----- Constants (#define) -----*/
// Assign your channel in pins
#define THRO_IN_PIN 2	//(default: 2) PCINT18
#define ALIE_IN_PIN 4	//(default: 4) PCINT20
#define ELEV_IN_PIN 7	//(default: 7) PCINT23
#define RUDD_IN_PIN 8	//(default: 8) PCINT0
#define GEAR_IN_PIN 9	//(default: 9) PCINT1
#define AUX1_IN_PIN 10	//(default: 10)PCINT2



// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THRO_FLAG 1  // Power
#define ALIE_FLAG 2  // Roll
#define ELEV_FLAG 4  // Pitch
#define RUDD_FLAG 8  // Yaw
#define GEAR_FLAG 16 // Gear
#define AUX1_FLAG 32 // AUX

/*----- Global Function Prototypes -----*/
void init_radio(); // initializes the pin mode, registers the intterupts and sets up the timer

float read_thro(); // indivual read of each axis and scaled
float read_rudd();
float read_elev();
float read_alie();
bool read_aux1();
bool read_gear();
bool thro_cut_off();

void read_radio(); // Reads the current value from the radio

float read_raw_thro(); /// Reads the raw value from each axis
float read_raw_rudd();
float read_raw_elev();
float read_raw_alie();
float read_raw_aux1();
float read_raw_gear();


//----- ISR's -----
void calcThro();
void calcRudd();
void calcElev();
void calcAlie();
void calcAux1();
void calcGear();

//----- LOCAL VARIABLES -----
// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
static volatile uint32_t ulThroStart;
static volatile uint32_t ulRuddStart;
static volatile uint32_t ulElevStart;
static volatile uint32_t ulAlieStart;
static volatile uint32_t ulAux1Start;
static volatile uint32_t ulGearStart;

//----- GLOBAL VARIABLES -----
// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
extern volatile uint16_t unThroInShared;
extern volatile uint16_t unRuddInShared;
extern volatile uint16_t unElevInShared;
extern volatile uint16_t unAlieInShared;
extern volatile uint16_t unAux1InShared;
extern volatile uint16_t unGearInShared;

// holds the update flags defined above
extern volatile uint8_t bUpdateFlagsShared;


/* Header protection close */
 #endif

 /* END OF FILE */

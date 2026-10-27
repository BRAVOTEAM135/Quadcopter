/*
 * Source File Rules:
 * ------------------
 * - Constants MUST be defined with capitals. E.g #define VALUE 10
 * - Functions MUST follow the following standard. E.g. adc_init(), adc_write()
 * - Variables Must follow the following standard. E.g. adcValueX, adcValueY
 * - Call by reference for variables. E.g.
 *
 * int a = 100;
 * int b = 200;
 * 
 * swap(&a, &b); //pass variable address in
 * 
 * Where
 * void swap(int *x, int *y){ // using the address will point to value (*x)
 * 	int temp;
 * 	temp = *x;
 * 	*x = *y;
 * 	*y = temp;
 * }
 * }
 * 
 * Source File Comments:
 * ---------------------
 * WARNING!!! - Ensure safety fail safe is enabled on the Hardware 
 * in case of signal loss or corruption
 * 			Low		Middle	High
 * Thro 	1120	 - 		1950
 * 
 * 			Left	Middle	Right
 * Rudd  	1908	1490	1078
 *
 *			Top		Middle	Bottom
 * Elev		1940	1504	1116	
 *
 *			Left	Middle	Right
 * Alie	 	1912	1488	1080	
 
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
#include "Radio.h"
#include "PinChangeInt.h"
#include "T1.h"
#include "Pt_By_Pt_Mean.h"

/*----- Constants (#define) -----*/
// Assign your channel in pins
#define THRO_IN_PIN 2
#define RUDD_IN_PIN 4
#define ELEV_IN_PIN 7
#define ALIE_IN_PIN 8


// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THRO_FLAG 1 // Power
#define RUDD_FLAG 2 // Yaw
#define ELEV_FLAG 4 // Pitch
#define ALIE_FLAG 8 // Roll



/*----- (Module) Local Variables (Static) -----*/
// create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThroIn;
  static uint16_t unRuddIn;
  static uint16_t unElevIn;
  static uint16_t unAlieIn;

  // local copy of update flags
  static uint8_t bUpdateFlags;
  
  
  // holds the update flags defined above
static volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
static volatile uint16_t unThroInShared;
static volatile uint16_t unRuddInShared;
static volatile uint16_t unElevInShared;
static volatile uint16_t unAlieInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
static volatile uint32_t ulThroStart;
static volatile uint32_t ulRuddStart;
static volatile uint32_t ulElevStart;
static volatile uint32_t ulAlieStart;


// variable returned after a read
static float throVal;
static float ruddVal;
static float elevVal;
static float alieVal;




/*----- Local Function -----*/
float scale_thro(float throVal) {
    //float THRO_scaled = (throVal - 1070)*(100/830);  // Scale the raw throttle val to percentage
    float THRO_scaled = (throVal - 1100)*(100.00/800.00);  // Scale the raw throttle val to percentage

	
	if (THRO_scaled >= 100){
	THRO_scaled = 100;
}
else if (THRO_scaled <= 0){
	THRO_scaled = 0;
}
  //return throVal;
 return THRO_scaled; 
 }


float scale_rudd(float rawRudd){
	float scaledRudd; // -45 to +45
	char range = 45;
	// Stick at rest
	if ((rawRudd <= 1505) & (rawRudd >= 1485)){
	scaledRudd = 0;
	} 
	
	// Stick move Left 
	else if ((rawRudd <= 1905) & (rawRudd > 1505)){
	//Range ==> 1905 - 1505 = 400
	//45 / 400 = 0.1125
	scaledRudd = ((range/400.00) * (1505 - rawRudd));
	}
	
	// Stick moved right
	else if ((rawRudd < 1485) & (rawRudd >= 1075)){
	//Range ==> 1485 - 1075 = 410
	// 45 / 410 = 0.10956
	scaledRudd = ((1485 - rawRudd) * (range/410.00));
	}
	
	//Left Out of limits
	else if (rawRudd > 1905){
		scaledRudd = ((-1) * range);
	}
	//Right out of Limits
	else if (rawRudd < 1075){
		scaledRudd = range;
	}
	
	
	return scaledRudd;
}

// Controls Pitch
float scale_elev(float rawElev){
	float scaledElev; // -45 to +45
	char range = 45;
	
	// Stick at rest
	if ((rawElev  <= 1495) & (rawElev >=1485)){
	scaledElev = 0;
	} 
	
	// Stick move Left 
	else if ((rawElev > 1495) & (rawElev <= 1915)){
	//Range ==> 1915 - 1495 = 420
	scaledElev = ((1495 - rawElev) * (range/420.00));
	}
	
	// Stick moved Right
	else if ((rawElev < 1485) & (rawElev >= 1075)){
	//Range ==> 1485 - 1075 = 410
	scaledElev = ((1485 - rawElev) * (range/410.00));
	}
	
	//Left Out of limits
	else if (rawElev > 1915){
		scaledElev = ((-1) * range);
	}
	//Right out of Limits
	else if (rawElev < 1075){
		scaledElev = range;
	}
	
	return scaledElev;
}

// Controls Roll
float scale_alie(float rawAlie){
	float scaledAlie; // -45 to +45
	char range = 45;
	
	// Stick at rest
	if ((rawAlie  <= 1495) & (rawAlie >=1485)){
	scaledAlie = 0;
	} 
	
	// Stick moved Forward
	else if ((rawAlie > 1495) & (rawAlie <= 1905)){
	//Range ==> 1905 - 1495 = 420
	scaledAlie = ((1495 - rawAlie) * (range/420.00));
	}
	
	// Stick moved Back
	else if ((rawAlie < 1485) & (rawAlie >= 1070)){
	//Range ==> 1485 - 1070 = 415
	scaledAlie = ((1485 - rawAlie) * (range/415.00));
	}
	
	//Forward Out of limits
	else if (rawAlie > 1905){
		scaledAlie = ((-1) * range);
	}
	
	//Backward out of Limits
	else if (rawAlie < 1070){
		scaledAlie = range;
	}
	
	return scaledAlie;
}


	


void read_radio() {

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
   
    if(bUpdateFlags & THRO_FLAG)
    {
      unThroIn = unThroInShared;
    }
   
    if(bUpdateFlags & RUDD_FLAG)
    {
      unRuddIn = unRuddInShared;
    }
   
    if(bUpdateFlags & ELEV_FLAG)
    {
      unElevIn = unElevInShared;
    }
    
    if(bUpdateFlags & ALIE_FLAG)
    {
      unAlieIn = unAlieInShared;
    }
    
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
   
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
 
  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop
 
  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.
 
 
  // we are checking to see if the channel value has changed, this is indicated 
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  if(bUpdateFlags & THRO_FLAG)
  {
    //Serial.print("THRO: ");
    //Serial.print(unThroIn);
	//throVal = scale_thro(unThroIn);
	throVal = unThroIn;

  }
 
  if(bUpdateFlags & RUDD_FLAG)
  {
    //Serial.print("  RUDD: ");
    //Serial.print(unRuddIn);
	//ruddVal = scale_rx(unRuddIn);
	ruddVal = unRuddIn;
  }
 
  if(bUpdateFlags & ELEV_FLAG)
  {
    //Serial.print("  ELEV: ");
    //Serial.print(unElevIn); 
	//elevVal = scale_rx(unElevIn);
	elevVal = unElevIn;

  }
  
  if(bUpdateFlags & ALIE_FLAG)
  {
    //Serial.print("  ALIE: ");
    //Serial.print(unAlieIn );  
	//alieVal = scale_rx(unAlieIn);
	alieVal = unAlieIn;
  }
 //Serial.println("");
  bUpdateFlags = 0;

}





/* ISR (Interrupt Service Routine) */
void calcThro() {
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  
  if((PIND & _BV(PD2)) == _BV(PD2)){ // PD2
	  
  //if(digitalRead(THRO_IN_PIN) == HIGH) {
    ulThroStart = get_count_t1();
	}
  else {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThroInShared = (uint16_t)(get_count_t1() - ulThroStart);
	
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THRO_FLAG;
	}
};

void calcRudd() {
 if((PIND & _BV(PD4)) == _BV(PD4)){ // PD4
  //if(digitalRead(RUDD_IN_PIN) == HIGH) {
    ulRuddStart = get_count_t1();
	}
  else {
    unRuddInShared = (uint16_t)(get_count_t1() - ulRuddStart);
    bUpdateFlagsShared |= RUDD_FLAG;
	}
};

void calcElev(){
if ((PIND & _BV(PD7)) == _BV(PD7)){ //PD7
  //if(digitalRead(ELEV_IN_PIN) == HIGH)	{
    ulElevStart = get_count_t1();
	}
  else  {
    unElevInShared = (uint16_t)(get_count_t1() - ulElevStart);
    bUpdateFlagsShared |= ELEV_FLAG;
	}
};

void calcAlie(){
	 if ((PINB & _BV(PB0)) == _BV(PB0)) { //PB0
  //if(digitalRead(ALIE_IN_PIN) == HIGH) {
    ulAlieStart = get_count_t1();
	}
  else {
    unAlieInShared = (uint16_t)(get_count_t1() - ulAlieStart);
    bUpdateFlagsShared |= ALIE_FLAG;
	}
};

/*----- Global Functions -----*/
void init_radio(){
  pinMode(THRO_IN_PIN, INPUT_PULLUP);
  pinMode(RUDD_IN_PIN, INPUT_PULLUP);
  pinMode(ELEV_IN_PIN, INPUT_PULLUP);
  pinMode(ALIE_IN_PIN, INPUT_PULLUP);
  

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THRO_IN_PIN, calcThro, CHANGE);
  PCintPort::attachInterrupt(RUDD_IN_PIN, calcRudd, CHANGE);
  PCintPort::attachInterrupt(ELEV_IN_PIN, calcElev, CHANGE);
  PCintPort::attachInterrupt(ALIE_IN_PIN, calcAlie, CHANGE);
  
  //PCintPort::detachInterrupt(3);
  
  //init_timer();
  init_t1();
}

float read_thro(){
	read_radio(); //ensure the latest value is returned
	//return average_thro(scale_thro(throVal/2));
	//return throVal/2;
	return scale_thro(throVal/2);
	}
	
float read_rudd(){
	read_radio();
	//return average_rudd(scale_rudd(ruddVal/2));
	//return ruddVal/2;
	return scale_rudd(ruddVal/2);
	}
	
float read_elev(){
	read_radio();
	//return average_elev(scale_elev(elevVal/2));
	//return elevVal/2;
	return scale_elev(elevVal/2);
	}
	
float read_alie(){
	read_radio();
	//return average_alie(scale_alie(alieVal/2));
	//return alieVal/2;
	return scale_alie(alieVal/2);
	}

	
	
float read_raw_thro(){
	return throVal/2;
	}

float read_raw_rudd(){
	return ruddVal/2;
	}

float read_raw_elev(){
	return elevVal/2;
	}

float read_raw_alie(){
	return alieVal/2;
	}

/* END OF FILE */
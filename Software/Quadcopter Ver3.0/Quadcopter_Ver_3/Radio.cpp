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
#include "T1.h"
#include "ISRs.h"


/*----- (Module) Local Variables (Static) -----*/
// create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThroIn;
  static uint16_t unRuddIn;
  static uint16_t unElevIn;
  static uint16_t unAlieIn;
  static uint16_t unAux1In;
  static uint16_t unGearIn;

  // local copy of update flags
  static uint8_t bUpdateFlags;
  
 // ----- Global Variables -----
  // holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
 volatile uint16_t unThroInShared;
 volatile uint16_t unRuddInShared;
 volatile uint16_t unElevInShared;
 volatile uint16_t unAlieInShared;
 volatile uint16_t unAux1InShared;
 volatile uint16_t unGearInShared;

// variable returned after a read
static float throVal;
static float ruddVal;
static float elevVal;
static float alieVal;
static float aux1Val;
static float gearVal;


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
	uint8_t range = 45;

	// 1850 <----- 1465 - 1460 -----> 1075

	// Stick at rest
	// Dead band of 5us
		if ((rawRudd >= 1465) & (rawRudd <= 1460)){
	scaledRudd = 0;
	} 
	
	// Stick move Left 
	else if ((rawRudd <= 1850) & (rawRudd > 1465)){
	//Range ==> 1850 - 1465 = 385
	//45 / 385 = 0.1169
	scaledRudd = ((range/385.00) * (1465 - rawRudd));
	}
	
	// Stick moved right
	else if ((rawRudd < 1460) & (rawRudd >= 1075)){
	//Range ==> 1460 - 1075 = 385
	// 45 / 385 = 0.1169
	scaledRudd = ((1460 - rawRudd) * (range/385.00));
	}
	
	//Left Out of limits
	else if (rawRudd > 1850){
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
	//5us dead band
	if ((rawElev  <= 1511) & (rawElev >=1507)){
	scaledElev = 0;
	} 
	
	// Stick move Forward
	else if ((rawElev > 1511) & (rawElev <= 1925)){
	//Range ==> 1925 - 1511 = 414
	scaledElev = ((1511 - rawElev) * (range/414.00));
	}
	
	// Stick moved Backward
	else if ((rawElev < 1507) & (rawElev >= 1093)){
	//Range ==> 1507 - 1093 = 414
	scaledElev = ((1507 - rawElev) * (range/414.00));
	}
	
	///Forward Out of limits
	else if (rawElev > 1925){
		scaledElev = ((-1) * range);
	}
	//Backward out of Limits
	else if (rawElev < 1093){
		scaledElev = range;
	}
	
	return scaledElev;
}

// Controls Roll
float scale_alie(float rawAlie){
	float scaledAlie; // -45 to +45
	uint8_t range = 45;
	
	// Stick at rest
	// 5us dead band
	if ((rawAlie  <= 1497 ) & (rawAlie >= 1493)){
	scaledAlie = 0;
	} 
	
	// Stick moved Left
	else if ((rawAlie > 1497) & (rawAlie <= 1910)){
	//Range ==> 1910 - 1497 = 413
	scaledAlie = ((1497 - rawAlie) * (range/413.00));
	}
	
	// Stick moved Right
	else if ((rawAlie < 1493) & (rawAlie >= 1072)){
	//Range ==> 1493 - 1072 = 421
	scaledAlie = ((1493 - rawAlie) * (range/421.00));
	}
	
	//Left Out of limits
	else if (rawAlie > 1910){
		scaledAlie = ((-1) * range);
	}
	
	//Right out of Limits
	else if (rawAlie < 1072){
		scaledAlie = range;
	}
	
	return scaledAlie;
}

bool scale_gear(float rawGear){
	bool GEAR_scaled;

	// Gear Toggled ON
	if (rawGear >= 1800){
		GEAR_scaled = true;
	}
	// Gear Toggled Off
	else {
		GEAR_scaled = false;
	}
	return GEAR_scaled;
}

bool scale_aux1(float rawAux1){
	bool AUX1_scaled;
	
	// AUX1 Toggled ON
	if (rawAux1 >= 1800){
		AUX1_scaled = true;
	}
	// AUX1 Toggled Off
	else {
		AUX1_scaled = false;
	}
	return AUX1_scaled;

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
    
    if(bUpdateFlags & AUX1_FLAG)
    {
      unAux1In = unAux1InShared;
    }

    if(bUpdateFlags & GEAR_FLAG)
    {
      unGearIn = unGearInShared;
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

  if(bUpdateFlags & AUX1_FLAG)
  {
    //Serial.print("  AUX1: ");
    //Serial.print(unAux1In );
	aux1Val = unAux1In;
  }


  if(bUpdateFlags & GEAR_FLAG)
  {
    //Serial.print("  GEAR: ");
    //Serial.print(unGearIn );
	gearVal = unGearIn;
  }
 //Serial.println("");
  bUpdateFlags = 0;

}


/*----- Global Functions -----*/
void init_radio(){

  //init_timer();
  init_t1();

  //initialise all the PCINTs for radio and configure the hardware
  init_radio_isr();
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

bool read_aux1(){
	read_radio();

	return scale_aux1(aux1Val/2);
	}

bool read_gear(){
	read_radio();

	return scale_gear(gearVal/2);
	}

bool thro_cut_off(){
	read_radio();
	bool THRO_cutoff;

	if (throVal/2 <= 1000){
		THRO_cutoff = true;
	}
	else {
		THRO_cutoff = false;
	}
	return THRO_cutoff;
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

float read_raw_aux1(){
	return aux1Val/2;
	}

float read_raw_gear(){
	return gearVal/2;
	}
/* END OF FILE */

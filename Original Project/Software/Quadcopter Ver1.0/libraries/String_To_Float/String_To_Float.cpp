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
 *
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "String_To_Float.h"
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>

/*----- Constants (#define) -----*/



/*----- (Module) Local Variables (Static) -----*/
static float val = 0.000;
static String inString;
static int flag = 0;

/*----- Local Function -----*/


/*----- Global Functions -----*/
float read_serial(){
	while(Serial.available() > 0){
		char incoming = Serial.read();
		inString += incoming; // instring is a string

		if(incoming == '\n'){	// Waits for the new line character to be received before looking at the string
			char newStr[inString.length() + 1]; //creates the buffer of char array

			inString.toCharArray(newStr, inString.length() + 1); // newstr now has the string as a char array

			//strcpy(newStr, inString);
			val = (float)atof(newStr);	// Converts the array of char to a double
			//Serial.println(val,6);	//This is inn for debugging really
			//return val;
			inString = "";	// Clear the buffer
			flag = 1; // FLag to handle the return
		}
	}
	// If flag raised there is a valid number to return
	if (flag == 1){ 
		flag = 0;
		return val;
	}
	// else no valid output to return
	else 
		return -1;
	
}



/* END OF FILE */
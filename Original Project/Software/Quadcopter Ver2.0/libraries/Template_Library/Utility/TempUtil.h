/****************************************************************************************************
* Title: Temp Utility Header File
* Description: 
* Last Edit: 
* Editior:
****************************************************************************************************/ 

// This is the include guard to prevent being included multiple time
#ifndef TempUtil_H
#define TempUtil_H
 
#include <Arduino.h> //Include any relavant headers that are need in the cpp

/****************************************************************************************************
* Define Constants e.g. #define
****************************************************************************************************/




/****************************************************************************************************
* Define Classes - Class Definition for the library
****************************************************************************************************/
 
 
/**************************************************
* Class_Name : This is an example class and should
* be replaced.
**************************************************/ 
class TempUtil{				// Define the class
public:							// Define public methods and variables
        TempUtil();			// Constructor
		~TempUtil();			// Deconstructor
		
		void openTempUtil();			
        void closeTempUtil();			
		int readTempUtil();				
		void writeTempUtil(uint16_t x);
        		
private:						// Define private methods and variables

};								// End of class
 
#endif				// Preprocessor directive

/****************************************************************************************************
* END OF FILE
****************************************************************************************************/
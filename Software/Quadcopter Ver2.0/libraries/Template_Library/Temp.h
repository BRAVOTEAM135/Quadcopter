/****************************************************************************************************
* Title: Template Header File
* Description: 
* Last Edit: 
* Editior:
****************************************************************************************************/ 

// This is the include guard to prevent being included multiple time
#ifndef Temp_H
#define Temp_H
 
#include <Arduino.h> //Include any relavant headers that are need in the cpp
#include "Utility/TempUtil.h"
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
class Temp{				// Define the class
public:							// Define public methods and variables
        Temp();			// Constructor
		~Temp();			// Deconstructor
		
		void openTemp();			
        void closeTemp();			
		int readTemp();				
		void writeTemp(uint16_t x);
        		
private:						// Define private methods and variables

};								// End of class
 
#endif				// Preprocessor directive

/****************************************************************************************************
* END OF FILE
****************************************************************************************************/
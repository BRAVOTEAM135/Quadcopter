/**************************************************
* Title: Example Sketch
* Description:
* Last Edit: 
* Editior:
**************************************************/ 

#include <Temp.h>

Temp temp; //initialize instance of class

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  temp.openTemp();			
  temp.closeTemp();			
  temp.readTemp();				
  temp.writeTemp(10);
}

/****************************************************************************************************
* END OF FILE
****************************************************************************************************/
/*
Structure of the UBX Packet Structure

Sync Char 1	|	Sync Char 2	|	Class	|	ID		|	Length	|	Payload	|	CK_A	| CK_B		|
1 Byte		|	1 Byte		|	1 Byte	|	1 Byte	|	2 Bytes	|			|	1 Byte	|	1 Byte	|
Example
NAV-POSLLH
0xB5	|	0x62	|	0x01	|	0x02	|	0x001C	|	iTOW,lon

Key
Sync Char 1  = DEC 181, HEX B5
Sync Char 2 = DEC 98, HEX 62
Class = 1 Byte Message Class
ID = 1 Byte Message ID
Length (Little Endian) = Length of the payload (2 bytes) excl. sync chars,class,ID, Length and checksum fields
Payload = Size dependin on length and/or class
Check sum = 2 bytes

Notes:
Check sum is calculated on:
|	Class	|	ID	|	Length	|	Payload	|
*/

#include <SoftwareSerial.h>

// Connect the GPS RX/TX to arduino pins 3 and 5
SoftwareSerial serial = SoftwareSerial(3,5);


const unsigned char UBX_HEADER[] = { 0xB5, 0x62 }; // this is the ublox header

struct NAV_POSLLH { // This is what the payload looks like for NAV_POSLLH. See data sheet. Cluster
  unsigned char cls; //8bits - 1Byte
  unsigned char id; // 8bits - 1Byte
  unsigned short len; //16bits - 2Bytes
  unsigned long iTOW; //32bits - 4Bytes
  long lon; //32bits - 4Bytes
  long lat;//32bits - 4Bytes
  long height; // 32bits - 4Bytes
  long hMSL; // 32bits - 4Bytes
  unsigned long hAcc; //32bits - 4Bytes
  unsigned long vAcc; // 32bits - 4Bytes
};

NAV_POSLLH posllh;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0; // thi is holding the position of the data stream
  static unsigned char checksum[2]; // initializing an array of size 2
  const int payloadSize = sizeof(NAV_POSLLH); // returns the number of bytes in the cluster 32 bytes

  while ( Serial.available() ) {
    byte c = Serial.read(); // reads 1 byte 
	
	//Check too ensure the ublox header is at the start of the data stream 
	// fpos = 0 => 0xB5, fpos = 1 => 0x62
    if ( fpos < 2 ) { // Will be true for first 2 runs
      if ( c == UBX_HEADER[fpos] ) // header efined at the top, check the first byte
        fpos++; // if first byte a match inctrement
      else
        fpos = 0; // if not match strt again
    }
	
	
	
	//fpos = 2, 0 < 32
    else { //this is now the data stream after the ubx header see the ublox data sheet 
      if ( (fpos - 2) < payloadSize ) // looking now at the payload size (cluster) element 0. 0 < 10
        
		((unsigned char*)(&posllh))[fpos-2] = c; //STILL NOT SURE WHAT THIS IS DOING set read byte to element in cluster 0x01 - cls, 0x02 - id
		// &posllh - retuns the memory location of posllh
		
		// it is saying that c is a char pointer to the first byte of the posllh 
		// structure, which is unsigned char cls

      fpos++; //increment fpos

	  
	  
/*
EXAMPLE

struct myStruct {
	unsigned char one;
	int two;
};

myStruct mine; // This declaring the struct as mine 

byte c = Serial.read(); // Serial Read returns 1 Byte

((unsigned char*)(&mine))[i] = c


When i = 0
place char into one

when i = 1 
place char into first byte of two

when i = 2 
place char into second byte of two

When i = 3
Will error as nothing there


*/	  
	  
	  //fpos = 3, PL = 32+2 = 34 - FALSE
      if ( fpos == (payloadSize+2) ) { 
        calcChecksum(checksum);
      }
	  
	  // fpos = 3, PL = 32+3 = 35 - FALSE
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
	  
	  // fpos = 3, PL = 32+4 = 36 - FALSE
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
	  
	  // fpos = 3, PL = 32+4 = 36 - FALSE
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

void setup() 
{
  Serial.begin(9600);
  serial.begin(9600);

}

void loop() {
  if ( processGPS() ) {
    // do something with GPS data
	Serial.print("iTOW:");      Serial.print(posllh.iTOW);
    Serial.print(" lat/lon: "); Serial.print(posllh.lat/10000000.0f); Serial.print(","); Serial.print(posllh.lon/10000000.0f);
    Serial.print(" height: ");  Serial.print(posllh.height/1000.0f);
    Serial.print(" hMSL: ");    Serial.print(posllh.hMSL/1000.0f);
    Serial.print(" hAcc: ");    Serial.print(posllh.hAcc/1000.0f);
    Serial.print(" vAcc: ");    Serial.print(posllh.vAcc/1000.0f);
    Serial.println();
  }
}

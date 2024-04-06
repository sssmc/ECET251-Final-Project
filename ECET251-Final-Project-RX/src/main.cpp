#include <Arduino.h>
#include <Manchester.h>

//Address to receive data on
#define ADDRESS 0x67

//Baudrate for serial connection
#define BAUDRATE 115200

//Pin definitions
#define RX_PIN 3

#define RED_LED 12
#define GREEN_LED 13

#define RGB_LED_RED 11
#define RGB_LED_GREEN 10
#define RGB_LED_BLUE 9

//Size of the buffer for the received data
#define BUFFER_SIZE 12

//Create receive buffer
uint8_t RXBuffer[BUFFER_SIZE];

uint8_t hammingDecodedData[6];

//Recevied packet count
uint8_t receivedPacketCount = 0;

//Function Prototypes

uint8_t CalculateChecksum(uint8_t data[5]);

void decodeHamming(uint8_t hammingData[12], uint8_t decodedData[6]);

void setup() {

  //Start the serial connection
  Serial.begin(BAUDRATE);

  //Pin Modes
  pinMode(RED_LED, OUTPUT); 
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RX_PIN, INPUT);

  pinMode(RGB_LED_RED, OUTPUT);
  pinMode(RGB_LED_GREEN, OUTPUT);
  pinMode(RGB_LED_BLUE, OUTPUT);

  //Turn off all LEDs to start
  digitalWrite(RED_LED, 0);
  digitalWrite(GREEN_LED, 0);
  digitalWrite(RGB_LED_RED, 0);
  digitalWrite(RGB_LED_GREEN, 0);
  digitalWrite(RGB_LED_BLUE, 0);

  //Setup the Manchester library to receive data
  man.setupReceive(RX_PIN, MAN_300 );
  man.beginReceiveArray(BUFFER_SIZE, RXBuffer);


}

void loop() {
	
	//If we have received a packet
  if (man.receiveComplete()) {
	  //Increment the received packet count
	  receivedPacketCount++;
    //Print the received packet count
	  Serial.print("Received... message # ");
	  Serial.println(receivedPacketCount);
	  digitalWrite(GREEN_LED, 1);
    //Print the received data
    man.beginReceiveArray(BUFFER_SIZE, RXBuffer);
    for(int i=0; i<BUFFER_SIZE; i++){
        Serial.print(RXBuffer[i], BIN);
    }
    Serial.println();
  
  //Decode the Hamming data
  Serial.println("Decoded Hamming Data:");
  decodeHamming(RXBuffer, hammingDecodedData);
  for(int i=0; i<6; i++){
    Serial.print(hammingDecodedData[i], BIN);
  }
  Serial.println();
  //Make an array to store the Checksum data
  uint8_t checksumData[5];
  //Copy the data from the RXBuffer to the checksumData
  for(int i=0; i<5; i++){
    checksumData[i] = hammingDecodedData[i];
  }
  //Check the checksum
  if (hammingDecodedData[5] == CalculateChecksum(checksumData)){
    Serial.println("Checksum OK, Received: " + String(hammingDecodedData[5]) + " Calculated: " + String(CalculateChecksum(checksumData)));
    if(hammingDecodedData[0] == ADDRESS){
      digitalWrite(RED_LED, 0);
      Serial.println("Address OK, Received: " + String(hammingDecodedData[0]) + " Expected: " + String(ADDRESS));
        Serial.println("Red: " + String(hammingDecodedData[2]) + " Green: " + String(hammingDecodedData[3]) + " Blue: " + String(hammingDecodedData[4]));
        analogWrite(RGB_LED_RED, hammingDecodedData[2]);
        analogWrite(RGB_LED_GREEN, hammingDecodedData[3]);
        analogWrite(RGB_LED_BLUE, hammingDecodedData[4]);
        
      } else{
      Serial.println("Address Not OK, Received: " + String(hammingDecodedData[0]) + " Expected: " + String(ADDRESS));
      digitalWrite(RED_LED, 1);

  } 
  }
  else 
  {
    Serial.println("Checksum Not OK, Received: " + String(hammingDecodedData[5]) + " Calculated: " + String(CalculateChecksum(checksumData)));
    digitalWrite(RED_LED, 1);
  }
  
  uint8_t calculatedChecksum = CalculateChecksum(hammingDecodedData);

  }
  delay(500);
  digitalWrite(GREEN_LED, 0);
  Serial.println("end");
}

uint8_t CalculateChecksum(uint8_t data[5])
{
  uint8_t checksum = 0;
  for (int i = 0; i < 5; i++)
  {
    checksum += data[i];
  }
  return checksum;
}

void decodeHamming(uint8_t hammingData[12], uint8_t decodedData[6]){


  //Array to hold the data nibbles
  uint8_t data[12] = {0, 0, 0, 0, 0, 0, 0, 0};

  //Variable to hold the position of the error
  uint8_t errorPosition = 0;

  //Reset decodedData to zeros
  for(int i = 0; i < 6; i++){
    decodedData[i] = 0;
  }

  //For each of the 12 hamming data bytes
  for (int i = 0; i < 12; i++)
  { 
    //Reset error position
    errorPosition = 0;

    //Calculate the three parity bits based off our data
    uint8_t parity1Calculated = ((hammingData[i] & 0x04) >> 2) ^ ((hammingData[i] & 0x10) >> 4) ^ ((hammingData[i] & 0x40) >> 6);
    uint8_t parity2Calculated = ((hammingData[i] & 0x04) >> 2) ^ ((hammingData[i] & 0x20) >> 5) ^ ((hammingData[i] & 0x40) >> 6);
    uint8_t parity3Calculated = ((hammingData[i] & 0x10) >> 4) ^ ((hammingData[i] & 0x20) >> 5) ^ ((hammingData[i] & 0x40) >> 6);

    //Check each calculated parity bit against the recevied parity bits and calculate the error position
    if (parity1Calculated != ((hammingData[i] & 0x01) >> 0)){
      errorPosition += 1;
    }
    if (parity2Calculated != ((hammingData[i] & 0x02) >> 1)){
      errorPosition += 2;
    }
    if (parity3Calculated != ((hammingData[i] & 0x08) >> 3)){
      errorPosition += 4;
    }

    //Flip the bit at the error position if needed
    switch (errorPosition)
    {
    case 3:
      hammingData[i] ^= 0x04;
      break;
    case 5:
      hammingData[i] ^= 0x10;
      break;
    case 6:
      hammingData[i] ^= 0x20;
      break;
    case 7:
      hammingData[i] ^= 0x40;
      break;
    }

    //Print error position if an error was detected
    if(errorPosition != 0){
      Serial.print("Error detected at position: ");
      Serial.println(errorPosition);
    }

    //Extract the data nibbles from the hamming data
    data[i] = ((hammingData[i] & 0x04) >> 2) | ((hammingData[i] & 0x70) >> 3);
  }

  //Reconstruct the data bytes from the data nibbles
  for (int i = 0; i < 6; i++){
    decodedData[i] = (data[i*2] << 4) | data[i*2+1];
  }
 
}

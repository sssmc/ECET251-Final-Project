#include <Arduino.h>
#include <Manchester.h>

//Address to receive data on
#define ADDRESS 0xA4

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
#define BUFFER_SIZE 6

//Create receive buffer
uint8_t RXBuffer[BUFFER_SIZE];

//Recevied packet count
uint8_t receivedPacketCount = 0;

//Function Prototypes

uint8_t CalculateChecksum(uint8_t data[5]);

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
  //Make an array to store the Checksum data
  uint8_t checksumData[5];
  //Copy the data from the RXBuffer to the checksumData
  for(int i=0; i<5; i++){
    checksumData[i] = RXBuffer[i];
  }
  //Check the checksum
  if (RXBuffer[5] == CalculateChecksum(checksumData)){
    Serial.println("Checksum OK, Received: " + String(RXBuffer[5]) + " Calculated: " + String(CalculateChecksum(checksumData)));
    if(RXBuffer[0] == ADDRESS){
      digitalWrite(RED_LED, 0);
      Serial.println("Address OK, Received: " + String(RXBuffer[0]) + " Expected: " + String(ADDRESS));
        analogWrite(RGB_LED_RED, RXBuffer[2]);
        analogWrite(RGB_LED_GREEN, RXBuffer[3]);
        analogWrite(RGB_LED_BLUE, RXBuffer[4]);
        
      } else{
      Serial.println("Address Not OK, Received: " + String(RXBuffer[0]) + " Expected: " + String(ADDRESS));
      digitalWrite(RED_LED, 1);

  } 
  }
  else 
  {
    Serial.println("Checksum Not OK, Received: " + String(RXBuffer[5]) + " Calculated: " + String(CalculateChecksum(checksumData)));
    digitalWrite(RED_LED, 1);
  }
  
  uint8_t calculatedChecksum = CalculateChecksum(RXBuffer);

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

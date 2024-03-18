#include <Arduino.h>

#include <Manchester.h>

/*

  Manchester Receiver example
  
  In this example receiver will receive array of 10 bytes per transmittion

  try different speeds using this constants, your maximum possible speed will 
  depend on various factors like transmitter type, distance, microcontroller speed, ...

  MAN_300 0
  MAN_600 1
  MAN_1200 2
  MAN_2400 3
  MAN_4800 4
  MAN_9600 5
  MAN_19200 6
  MAN_38400 7

*/

#define ADDRESS 0xA4

#define RX_PIN 3

#define RED_LED 12
#define GREEN_LED 13

#define RGB_LED_RED 11
#define RGB_LED_GREEN 10
#define RGB_LED_BLUE 9

#define BAUDRATE 115200


#define BUFFER_SIZE 6

uint8_t RXBuffer[BUFFER_SIZE];

uint8_t Rx_num=0;

uint8_t LEDColors[3] = {0,0,0};

uint8_t CalculateChecksum(uint8_t data[5]);

void setup() {
  pinMode(RED_LED, OUTPUT); 
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RX_PIN, INPUT);

  pinMode(RGB_LED_RED, OUTPUT);
  pinMode(RGB_LED_GREEN, OUTPUT);
  pinMode(RGB_LED_BLUE, OUTPUT);

  Serial.begin(BAUDRATE);

  digitalWrite(RED_LED, 0);
  digitalWrite(GREEN_LED, 0);
  digitalWrite(RGB_LED_RED, 0);
  digitalWrite(RGB_LED_GREEN, 0);
  digitalWrite(RGB_LED_BLUE, 0);

  man.setupReceive(RX_PIN, MAN_300 );
  man.beginReceiveArray(BUFFER_SIZE, RXBuffer);


}

void loop() {
	
	
  if (man.receiveComplete()) {
	  
	  Rx_num++;
	  Serial.print("Received... message # ");
	  Serial.println(Rx_num);
	  digitalWrite(RED_LED, 1);
	  
    
    man.beginReceiveArray(BUFFER_SIZE, RXBuffer);
    for(int i=0; i<BUFFER_SIZE; i++){
        Serial.print(RXBuffer[i], BIN);
    }
    Serial.println();

  uint8_t checksumData[5];

  for(int i=0; i<5; i++){
    checksumData[i] = RXBuffer[i];
  }

  if (RXBuffer[5] == CalculateChecksum(checksumData)){
    Serial.println("Checksum OK, Received: " + String(RXBuffer[5]) + " Calculated: " + String(CalculateChecksum(checksumData)));
    if(RXBuffer[0] == ADDRESS){
      Serial.println("Address OK, Received: " + String(RXBuffer[0]) + " Expected: " + String(ADDRESS));
        analogWrite(RGB_LED_RED, RXBuffer[2]);
        analogWrite(RGB_LED_GREEN, RXBuffer[3]);
        analogWrite(RGB_LED_BLUE, RXBuffer[4]);
        
      } else{
      Serial.println("Address Not OK, Received: " + String(RXBuffer[0]) + " Expected: " + String(ADDRESS));
    }
  } else {
    Serial.println("Checksum Not OK, Received: " + String(RXBuffer[5]) + " Calculated: " + String(CalculateChecksum(checksumData)));

  }

  uint8_t calculatedChecksum = CalculateChecksum(RXBuffer);

  }
  delay(500);
  digitalWrite(RED_LED, 0);
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

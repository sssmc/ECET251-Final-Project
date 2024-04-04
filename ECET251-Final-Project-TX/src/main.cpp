#include <Arduino.h>

#include "Manchester.h"

/*Packet Structure

00000000      00000000    00000000      00000000     00000000     00000000
|Address| |Packet Count| |Red Value| |Green Value| |Blue Value|  |Checksum|
    0            1             2             3           4            5
*/

//Packet Address
#define ADDRESS 0xA4

//Serial Baudrate
#define BAUDRATE 9600

//Serial Print Delay
#define PRINT_DELAY 1000

//Measurement Delay
#define MEASURE_DELAY 100

//Pin Definitions
#define TX_PIN  3

#define GREEN_LED  13         
#define RED_LED    12        

#define USER_BUTTON  8          

#define RED_POT A0
#define GREEN_POT A1
#define BLUE_POT A2

#define RGB_LED_RED  11
#define RGB_LED_GREEN  10
#define RGB_LED_BLUE  9

//Packet
uint8_t TXData[3] = {0, 0, 0};

uint8_t packetCount = 0;

//Delays

uint64_t currentMillis = 0;
uint64_t lastPrint = 0;
uint64_t lastMeasure = 0;

uint16_t LEDColors[3] = {0, 0, 0};

//Function Prototypes
uint8_t CalculateChecksum(uint8_t data[3]);

void TransmitPacket(uint8_t data[3]);

void setup()
{

  //Initialize the Serial Connection
  Serial.begin(BAUDRATE);

  //Print the project information
  Serial.println("ECET 251 - Final Project");
  Serial.println("RGB LED Controller");
  Serial.println("Sebastien Robitaille - Kayleb Stetsko");
  Serial.println("Spring 2024");
  Serial.println("-----------------------------");

  //Pin Setup
  pinMode(TX_PIN, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(USER_BUTTON, INPUT_PULLUP);

  pinMode(RED_POT, INPUT);
  pinMode(GREEN_POT, INPUT);
  pinMode(BLUE_POT, INPUT);

  //Turn off the LEDs to start
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  //Initialize the Manchester Library
  man.setupTransmit(TX_PIN, MAN_300);

}



void loop()
{
  //Get the current time
  currentMillis = millis();

  //If the measurement delay has passed
  if (lastMeasure + MEASURE_DELAY < currentMillis)
  {
    //Reset the measurement delay
    lastMeasure = currentMillis;

    //Read the potentiometers
    LEDColors[0] = analogRead(RED_POT);
    LEDColors[1] = analogRead(GREEN_POT);
    LEDColors[2] = analogRead(BLUE_POT);

    //Output the LED Colors to the RGB LED
    analogWrite(RGB_LED_RED, LEDColors[0] / 4);
    analogWrite(RGB_LED_GREEN, LEDColors[1] / 4);
    analogWrite(RGB_LED_BLUE, LEDColors[2] / 4);

    //Set the LED Colors in the TXPacket
    TXData[0] = LEDColors[0];
    TXData[1] = LEDColors[1];
    TXData[2] = LEDColors[2];

  }
  
  //If the print delay has passed
  if(lastPrint + PRINT_DELAY < currentMillis)
  {
    //Reset the print delay
    lastPrint = currentMillis;

    //Print the LED Colors to the Serial Monitor
    lastPrint = currentMillis;
    Serial.print("Red: ");
    Serial.print(LEDColors[0]);
    Serial.print(" Green: ");
    Serial.print(LEDColors[1]);
    Serial.print(" Blue: ");
    Serial.println(LEDColors[2]);
  }

  //If the button is pressed
  if(digitalRead(USER_BUTTON) == LOW)
  {
    Serial.println("Button Pressed");

    //Turn on the Red LED
    digitalWrite(RED_LED, HIGH);
    //Trasmit the packet
    TransmitPacket(TXData);
    //Delay to prevent multiple transmissions
    delay(1000);
    //Turn off the Red LED
    digitalWrite(RED_LED, LOW);
  }
  
}

void TransmitPacket(uint8_t data[3])
{

  //Array to hold the data checked by the checksum
  uint8_t ChecksumData[5] = {0, 0, 0, 0, 0};
  //Array to hold the full packet
  uint8_t TXPacket[6] = {0, 0, 0, 0, 0, 0};

  //Set the packet address
  ChecksumData[0] = ADDRESS;

  //Set the packet count
  ChecksumData[1] = packetCount;

  //Set the data from the potentiometers
  for (int i = 0; i < 3; i++)
  {
     ChecksumData[i + 2] = data[i];
  }

  //Put the checksum checked data into the final packet
  for(int i=0; i<5; i++)
  {
    TXPacket[i] = ChecksumData[i];
  }

  //Calculate and set the checksum in the final packet
  TXPacket[5] = CalculateChecksum(ChecksumData);

  //Print the data from the potentiometers
  Serial.print("Packet Data: ");
  for(int i = 0; i < 3; i++)
  {
    Serial.print(" - ");
    Serial.print(data[i], BIN);
  }
  Serial.println("");

  //Print the final packet
  Serial.println("Transmitting Packet: ");
  for(int i = 0; i < 6; i++)
  {
    if(i != 0){Serial.print(" - ");}
    Serial.print(TXPacket[i], BIN);
  }
  Serial.println("");

  //Transmit the packet
  man.transmitArray(6, TXPacket);

  //Increment the packet count and round it to 8 bits
  packetCount += 1;
  packetCount = packetCount % 256;

}

uint8_t CalculateChecksum(uint8_t data[5])
{
  uint8_t checksum = 0;

  //Sum the data bytes
  for (int i = 0; i < 5; i++)
  {
    checksum += data[i];
  }
  return checksum;
}

void encodeHamming(uint8_t data[4], uint8_t hammingData[8]){
  
  //Reset hammingData to zeros
  for(int i = 0; i < 8; i++){
    hammingData[i] = 0;
  }

  //For each of the 4 data bytes
  for (int i = 0; i < 4; i++)
  { 
    //Split the data byte into two nibbles
    uint8_t nibbles[2]  = {0, 0};
    //High nibble
    nibbles[0] = data[i] >> 4;
    //Low nibble
    nibbles[1] = data[i] & 0x0F;

    //For each nibble
    for (int j = 0; j < 2; j++)
    {
      //Hamming code structure
      //0  0    0   0  0  0  0  0
      //0  D4  D3 D2  P3  D1 P2 P1

      //Calculate the three parity bits based off our data
      uint8_t parity1 = (nibbles[j] & 0x01) ^ ((nibbles[j] & 0x02) >> 1) ^ ((nibbles[j] & 0x08) >> 3);
      uint8_t parity2 = (nibbles[j] & 0x01) ^ ((nibbles[j] & 0x04) >> 2) ^ ((nibbles[j] & 0x08) >> 3);
      uint8_t parity3 = ((nibbles[j] & 0x02) >> 1) ^ ((nibbles[j] & 0x04) >> 2) ^ ((nibbles[j] & 0x08) >> 3);

      //Construct the hamming data byte using the three parity bits and the data nibble
      hammingData[i*2+j] = ((nibbles[j] & 0x08) << 3) | ((nibbles[j] & 0x04) << 3) | ((nibbles[j] & 0x02) << 3) | (parity3 << 3) | ((nibbles[j] & 0x01) << 2) | (parity2 << 1) | parity1;

    }

  }
  
}


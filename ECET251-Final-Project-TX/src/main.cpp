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

  Serial.begin(BAUDRATE);

  //Print the project information
  Serial.println("ECET 251 - Final Project");
  Serial.println("RGB LED Controller");
  Serial.println("Sebastien Robitaille - Kayleb Stetsko");
  Serial.println("Spring 2024");
  Serial.println("-----------------------------");

  pinMode(TX_PIN, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(USER_BUTTON, INPUT_PULLUP);

  pinMode(RED_POT, INPUT);
  pinMode(GREEN_POT, INPUT);
  pinMode(BLUE_POT, INPUT);

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  man.setupTransmit(TX_PIN, MAN_300);

}



void loop()
{
  currentMillis = millis();

  if (lastMeasure + MEASURE_DELAY < currentMillis)
  {
    lastMeasure = currentMillis;
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

    lastMeasure = currentMillis;
  }
  
  if(lastPrint + PRINT_DELAY < currentMillis)
  {
    //Print the LED Colors to the Serial Monitor
    lastPrint = currentMillis;
    Serial.print("Red: ");
    Serial.print(LEDColors[0]);
    Serial.print(" Green: ");
    Serial.print(LEDColors[1]);
    Serial.print(" Blue: ");
    Serial.println(LEDColors[2]);

    lastPrint = currentMillis;
  }

  if(digitalRead(USER_BUTTON) == LOW)
  {
    Serial.println("Button Pressed");

    digitalWrite(RED_LED, HIGH);

    //Trasmit the packet
    TransmitPacket(TXData);
    delay(1000);

    digitalWrite(RED_LED, LOW);
  }
  
}

void TransmitPacket(uint8_t data[3])
{

  uint8_t ChecksumData[5] = {0, 0, 0, 0, 0};
  uint8_t TXPacket[6] = {0, 0, 0, 0, 0, 0};


  //Set the packet address
  ChecksumData[0] = ADDRESS;

  //Set the packet count
  ChecksumData[1] = packetCount;

  //Set the data
  for (int i = 0; i < 3; i++)
  {
     ChecksumData[i + 2] = data[i];
  }

  //Set the checksum data to the final packet
  for(int i=0; i<5; i++)
  {
    TXPacket[i] = ChecksumData[i];
  }

  //Calculate and set the checksum in the final packet
  TXPacket[5] = CalculateChecksum(ChecksumData);

  Serial.print("Packet Data: ");
  for(int i = 0; i < 3; i++)
  {
    Serial.print(" - ");
    Serial.print(data[i], BIN);
  }
  Serial.println("");

  Serial.println("Transmitting Packet: ");
  for(int i = 0; i < 6; i++)
  {
    if(i != 0){Serial.print(" - ");}
    Serial.print(TXPacket[i], BIN);
  }
  Serial.println("");

  //Transmit
  man.transmitArray(6, TXPacket);

  //Increment the packet count
  packetCount += 1;
  packetCount = packetCount % 256;

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


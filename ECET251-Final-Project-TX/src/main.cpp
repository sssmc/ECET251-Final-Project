#include <Arduino.h>

#include "Manchester.h"

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

//Delays

uint64_t currentMillis = 0;
uint64_t lastPrint = 0;
uint64_t lastMeasure = 0;

uint8_t LEDColors[3] = {0, 0, 0};

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

}



void loop()
{
  currentMillis = millis();

  if (currentMillis + MEASURE_DELAY > lastMeasure)
  {
    lastMeasure = currentMillis;
    LEDColors[0] = analogRead(RED_POT);
    LEDColors[1] = analogRead(GREEN_POT);
    LEDColors[2] = analogRead(BLUE_POT);
  }
  
  if(currentMillis + PRINT_DELAY > lastPrint)
  {
    //Output the LED Colors to the RGB LED
    analogWrite(RGB_LED_RED, LEDColors[0]);
    analogWrite(RGB_LED_GREEN, LEDColors[1]);
    analogWrite(RGB_LED_BLUE, LEDColors[2]);

    //Print the LED Colors to the Serial Monitor
    lastPrint = currentMillis;
    Serial.print("Red: ");
    Serial.print(LEDColors[0]);
    Serial.print(" Green: ");
    Serial.print(LEDColors[1]);
    Serial.print(" Blue: ");
    Serial.println(LEDColors[2]);
  }
  
}

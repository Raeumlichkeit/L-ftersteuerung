/*
  Ein Projekt um Lüfter Temperatur bestimmt zu steuern mit möglichkeit die Lüfter auch Manuell zu Steuern

  Last Edit: 07.09.2023

*/


// Library //
#include "DHT.h"
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Defineren der Pins //
#define DHTPIN 5
#define DHTTYPE DHT22
#define SensorPin = 3;
#define fanPin = 9;
#define potiPin = A1 ;
#define schalterpin = 4;
#define RST_PIN -1
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;
DHT dht(DHTPIN, DHTTYPE);

// Konstanten //
const int fanMinSpeed = 75;
const int tMin = 20;
const int tMax = 50;

// Variablen //
int InterruptCounter, rpm;
int fanSpeed = 0;
int potiVar = 0 ;
int fanMin = 20;
int fanOut = 1;
int schalterstate = 0;
float Temperatur;


void setup()
{
  TCCR1B = TCCR1B & 0b11111000 | 0x01;    // Damit der Lüfter nicht Fiebt
  Serial.begin(19200);                    // Beginn des Serialen Monitors
  pinMode(fanPin, OUTPUT) ;
  pinMode(potiPin, INPUT) ;
  pinMode(schalterpin, INPUT);
  dht.begin();                            // Temperatur fühler startet
  Wire.begin();                           // 1Wire library startet
  Wire.setClock(400000L);                 // Daten übertragungs geschwindigkeit wird festgelegt

  // Weiß nicht was das macht irgendwas fürs oled, ohne gehts nicht //
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(X11fixed7x14B);            // Wie es im namen ist stellt die Font ein
  oled.clear();                           // Leert den Cache vom oled

}


void loop()
{
  Temperatur = dht.readTemperature();    // Damit die Temperatur ausgabe immer geht
  oled.clear();                          // Leert den Cache vom oled
  oled.println(schalterstate);           // zeigt den Status an wie der schalter steht (1 = Manuell / 0 = Automatik)
  oled.print("RPM: ");
  oled.println(rpm);                     // Zeigt die Geschwindigkeit des Lüfters an
  oled.print(Temperatur);                // Zeigt die Temperatur des Fühlers an
  oled.print(" Grad");

  meassure();

  schalterstate = digitalRead(schalterpin); //Macht den Schalter "Digital" (ihm wird bei LOW 0 und bei HIGH 1 zugewiesen)
  if (schalterstate == LOW) {
    schalterstate = 0;
  } else {
    schalterstate = 1;
  }

  if (schalterstate == 1) {                //Umstellung des Schalters von Automatik zu Manuell und umgekehrt
    potie();
  }
  else if (schalterstate == 0 ) {

    temp();
  }
}

void meassure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING); // Reagiert auf jede Reisende Flanke vom Sensor Pin des Arduinos
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(SensorPin));   //Der interuppt wird abgegen
  rpm = (InterruptCounter / 2) * 60;                   //Berechnung der RPM(rounds per minute)
}

void countup() {
  InterruptCounter++; }  //Bei jeder Steigenden Flanke wird der Counter um 1 erhöht


void temp() {                                         // Die Automatisierung vom Modus 0
  Temperatur = dht.readTemperature();
  fanSpeed = map(Temperatur, tMin, tMax, 0, 255);     //Array zum festlegen der Daten

  if (fanSpeed < fanMin)
  {
    fanSpeed = 0;
    fanOut = 1;
  }

  /* Hauptpart der Automatisierung als Hysterese */

  if (fanOut == 1)
  {
    fanSpeed = 0;
  }

  if (Temperatur >= 20)
  {
    if (fanOut == 1)
    {
      fanOut = 0;
      analogWrite(fanPin, 255);                          //Werte auf den Pin geben
    }
  }

  if (fanSpeed > 255)
  {
    fanSpeed = 255;
  }


  analogWrite(fanPin, fanSpeed);
  delay(2000);
}
void potie() {                                           //Manuelle Steuerung des Lüfters
  potiVar = analogRead(potiPin) ;                        //Auslesen des Potis
  fanSpeed = map(potiVar, 51, 1023, fanMinSpeed, 255);   //Array zur Aufteilung der Werte vom Potis
  analogWrite(fanPin, fanSpeed);                         //Werte auf den Pin geben                         
}  

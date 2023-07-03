// Library //
#include "DHT.h"
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Defineren der Pins //
#define DHTPIN 5
#define DHTTYPE DHT22
#define I2C_ADDRESS 0x3C
#define RST_PIN -1

DHT dht(DHTPIN, DHTTYPE);
SSD1306AsciiWire oled;

// Konstanten //
const int SensorPin = 3; 
const int fanPin = 9;        
const int potiPin = A1 ;    
const int fanMinSpeed = 75;
const int schalterpin = 4;
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
  dht.begin();                            // Temperatur fühler start
  Wire.begin();                           // 1Wire library start
  Wire.setClock(400000L);                 // Clock eingestellt

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

  schalterstate = digitalRead(schalterpin);
if (schalterstate == LOW) {
  schalterstate = 0;
} else {
  schalterstate = 1;
}

if (schalterstate == 1){
 potie();
}
  else if (schalterstate == 0 ) {
   
  temp();
  }
}

void meassure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  rpm = (InterruptCounter / 2) * 60;
  display_rpm(); }

void countup() {
  InterruptCounter++; }

void display_rpm() {
  Serial.print(" RPM: ");
  Serial.println(rpm); }

void temp() { 
   Temperatur = dht.readTemperature();
    Serial.print("Temperatur: ");
  Serial.print(Temperatur);
  Serial.println(" Grad Celsius");
  fanSpeed = map(Temperatur, tMin, tMax, 0, 255);    

  if (fanSpeed < fanMin)
  {
    fanSpeed = 0;
    fanOut = 1;
  }
   
  // Hysterese
  if (fanOut == 1)
  {
    fanSpeed = 0;
  }
   
  if(Temperatur >= 20)
  {
    if(fanOut == 1)
    {
      fanOut = 0;
      analogWrite(fanPin, 255);
    }
  }
     
  if (fanSpeed > 255)
  { 
    fanSpeed = 255;
  }
   
   
  analogWrite(fanPin, fanSpeed);     
delay(2000);
}
void potie() {
  potiVar = analogRead(potiPin) ;                       
  fanSpeed = map(potiVar, 51, 1023, fanMinSpeed, 255);  
    analogWrite(fanPin, fanSpeed);
    }
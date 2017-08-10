


///define all Pins
///------------------------------------------------------------------------------///srf04
////deefine Pins of srf04

#define trigPin1 13
#define echoPin1 12
#define ledr1 11
//#define ledg1 

#define trigPin2 10
#define echoPin2 9
#define ledr2 8
//#define ledg2

#define trigPin3 7
#define echoPin3 6
#define ledr3 5
//#define ledg3

#define trigPin4 4
#define echoPin4 3
#define ledr4 2
//#define ledg4 

////end of define Pins of srf04
///------------------------------------------------------------------------------///mq2
///mq2 ints

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int FanPin = A2;                 // Fan connected to digital pin 1
int sensorValue = 0; 

///end of mq2 ints
///-----------------------------------------------------------------------------///dht11
///dht11 ints

#include <dht.h>
#define dht_apin A1 // Analog Pin sensor is connected to 
dht DHT;

///end of dht11 ints
///----------------------------------------------------------------------------///rc5r22
///rc522 ints

#include <RFID.h>

/*
* Read a card using a mfrc522 reader on your SPI interface
* Pin layout should be as follows (on Arduino Uno):
* MOSI: Pin 11 / ICSP-4
* MISO: Pin 12 / ICSP-1
* SCK: Pin 13 / ISCP-3
* SS/SDA: Pin 10
* RST: Pin 9
*/

#include <SPI.h>
#include <RFID.h>
#define SS_PIN 10
#define RST_PIN 9

RFID rfid(SS_PIN,RST_PIN);


int led = 7;
int power = 8; 
int serNum[5];
int cards[][5] = {
  {0, 0 ,0, 0 ,0
} ///can be change
  
};
bool access = false;
///end of rc522 ints
///----------------------------------------
///----------------------------------------///setup

void setup() {
  ///----///srf04
  Serial.begin (9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(ledr1, OUTPUT);
 // pinMode(ledg1, OUTPUT);
  
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(ledr2, OUTPUT);
 // pinMode(ledg2, OUTPUT);
  
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(ledr3, OUTPUT);
 // pinMode(ledg3, OUTPUT);
  
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(ledr4, OUTPUT);
 // pinMode(ledg4, OUTPUT);
  ///----///end

  ///----///mq2
  pinMode(FanPin, OUTPUT);
 ///----///end
 
 ///----///dht11
  delay(100);//Delay to let system boot
  Serial.println("DHT11 Humidity & temperature Sensor\n\n");
  delay(200);//Wait before accessing Sensor
 ///----///end

 ///----///rc522
    SPI.begin();
    rfid.init();
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
 ///----///end
}


///--------------------------------------///loop

void loop() {

  ///-------------------------------------------------------------------------------///start codes of srf04_xx
  ///--------------------------------///srf04_01 & turn on/off LED
   long duration1, distance1;
  digitalWrite(trigPin1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1/2) / 29.1;
  if (distance1 < 10) {  // This is where the LED On/Off happens
    digitalWrite(ledr1,HIGH); // When this parking is full red led is turn on
}
  else {
    digitalWrite(ledr1,LOW);
  }
  if (distance1 >= 200 || distance1 <= 0){
    Serial.println("Out of range1");
  }
  else {
    Serial.print("distance1=");
    Serial.print(distance1);
    Serial.println(" cm");
  }
  //delay(50);

  

    ///------------------------------///srf04_02 & turn on/off LED
    long duration2, distance2;
  digitalWrite(trigPin2, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2/2) / 29.1;
  if (distance2 < 10) {  // This is where the LED On/Off happens
    digitalWrite(ledr2,HIGH); // When this parking is full red led is turn on
}
  else {
    digitalWrite(ledr2,LOW);
  }
  if (distance2 >= 200 || distance2 <= 0){
    Serial.println("Out of range2");
  }
  else {
    Serial.print("distance2=");
    Serial.print(distance2);
    Serial.println(" cm");
  }
  //delay(50);

  
    
    ///------------------------------///srf04_03 & turn on/off LED
    long duration3, distance3;
  digitalWrite(trigPin3, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3/2) / 29.1;
  if (distance3 < 10) {  // This is where the LED On/Off happens
    digitalWrite(ledr3,HIGH); // When this parking is full red led is turn on
}
  else {
    digitalWrite(ledr3,LOW);
  }
  if (distance3 >= 200 || distance3 <= 0){
    Serial.println("Out of range3");
  }
  else {
    Serial.print("distance3=");
    Serial.print(distance3);
    Serial.println(" cm");
  }
  //delay(50);
  
  

    ///------------------------------///srf04_04 & turn on/off LED
    long duration4, distance4;
  digitalWrite(trigPin4, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  distance4 = (duration4/2) / 29.1;
  if (distance4 < 10) {  // This is where the LED On/Off happens
    digitalWrite(ledr4,HIGH); // When this parking is full red led is turn on
    //digitalWrite(ledg4,LOW);
}
  else {
    digitalWrite(ledr4,LOW);
  }
  if (distance4 >= 200 || distance4 <= 0){
    Serial.println("Out of range4");
  }
  else {
    Serial.print("distance4=");
    Serial.print(distance4);
    Serial.println(" cm");
  }
  //delay(50);
  ///-------------------------------------------------///end of srf04_xx


  ///-------------------------------------------------------------------------------///start codes of mq2
   sensorValue = analogRead(analogInPin);            
  // determine alarm status
  if (sensorValue >= 100)
  {
    digitalWrite(FanPin, HIGH);   // sets the Fan on
  }
  else
  {
  digitalWrite(FanPin, LOW);    // sets the Fan off
  }

  // print the results to the serial monitor:
  Serial.print("smoke = " );                       
  Serial.println(sensorValue);     
  //delay(100);
   ///------------------------------------------------///end of mq2

   //--------------------------------------------------------------------------------///start codes of dht11 
  DHT.read11(dht_apin);

    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C  ");
 
  //delay(100);//Wait 1 seconds before accessing sensor again.
  //-----------------------------------------------------///end of dht11
 
  //--------------------------------------------------------------------------------///start codes of rc522
  if(rfid.isCard()){
    
        if(rfid.readCardSerial()){
            Serial.print(rfid.serNum[0]);
            Serial.print(" ");
            Serial.print(rfid.serNum[1]);
            Serial.print(" ");
            Serial.print(rfid.serNum[2]);
            Serial.print(" ");
            Serial.print(rfid.serNum[3]);
            Serial.print(" ");
            Serial.print(rfid.serNum[4]);
            Serial.println("");
            
            for(int x = 0; x < sizeof(cards); x++){
              for(int i = 0; i < sizeof(rfid.serNum); i++ ){
                  if(rfid.serNum[i] != cards[x][i]) {
                      access = false;
                      break;
                  } else {
                      access = true;
                  }
              }
              if(access) break;
            }
           
        }
        
       if(access){
          Serial.println("Welcome!");
           digitalWrite(led, HIGH); 
           delay(1000);
           digitalWrite(led, LOW);
           digitalWrite(power, HIGH);
           delay(1000);
           digitalWrite(power, LOW);
           
      } else {
           Serial.println("Not allowed!"); 
           digitalWrite(led, HIGH);
           delay(500);
           digitalWrite(led, LOW); 
           delay(500);
           digitalWrite(led, HIGH);
           delay(500);
           digitalWrite(led, LOW);         
       }        
    }
 
    rfid.halt();
  //-----------------------------------------------------///end of rc522
  
}


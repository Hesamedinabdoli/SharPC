///This Code is for Arduino MEGA 2560


///define all Pins
///------------------------------------------------------------------------------///srf04
////deefine Pins of srf04s

#define trigPin1 22
#define echoPin1 23
#define led1 24

#define trigPin2 25
#define echoPin2 26
#define led2 27

#define trigPin3 28
#define echoPin3 29
#define led3 30

#define trigPin4 31
#define echoPin4 32
#define led4 33

#define trigPin5 34
#define echoPin5 35
#define led5 36

#define trigPin6 37
#define echoPin6 38
#define led6 39

#define trigPinen 40
#define echoPinen 41

#define trigPinex 43
#define echoPinex 44

////end of define Pins of srf04
///------------------------------------------------------------------------------///mq2
///mq2 ints

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int FanPin = 2;                 // Fan connected to digital pin 1
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

#include <Servo.h>
Servo servoen;  // create servo object to control a servo
Servo servoex;
int posen = 0;    // variable to store the servo position
int posex = 0;


/*
PINOUT:
RC522 MODULE    Uno/Nano     MEGA
SDA             D10          D9
SCK             D13          D52
MOSI            D11          D51
MISO            D12          D50
IRQ             N/A          N/A
GND             GND          GND
RST             D9           D8
3.3V            3.3V         3.3V
*/

#include <SPI.h>
#include <RFID.h>

#define SS_PINEN 10
#define RST_PINEN 9

#define SS_PINEX 12
#define RST_PINEX 11

RFID rfiden(SS_PINEN,RST_PINEN);
RFID rfidex(SS_PINEX,RST_PINEX);

int leden = 48;
int ledex = 49;

int serNum[5];
int cards[5][5] = {
  {133 ,191, 241, 194 ,9},
  {0 ,0 ,0 ,0 ,0},
  {0 ,0 ,0 ,0 ,0},
  {0 ,0 ,0 ,0 ,0},
  {0 ,0 ,0 ,0 ,0},
  
};
bool access = false;

///end of rc522 ints
///---------------------------------------------------------------------------///Zigbee
///Zigbee ints

char ss1=0xfd;
char ss2=0x03;
char ss3=0x00;
char ss4=0x00;
byte recieve[20];

///end of Zigbee

void setup() {///------------------------------------------------------------------------------------------------------------///start of setup
  ///---------------------------------------///srf04
  Serial.begin (9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(led1, OUTPUT);
  
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(led2, OUTPUT);
  
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(led3, OUTPUT);
  
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(led4, OUTPUT);

  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
  pinMode(led5, OUTPUT);
  
  pinMode(trigPin6, OUTPUT);
  pinMode(echoPin6, INPUT);
  pinMode(led6, OUTPUT);

  pinMode(trigPinen, OUTPUT);
  pinMode(echoPinen, INPUT);
  
  pinMode(trigPinex, OUTPUT);
  pinMode(echoPinex, INPUT);
  
  ///------------------------------///end

  ///------------------------------------///mq2
  pinMode(FanPin, OUTPUT);
 ///-------------------------------///end
 
 ///-------------------------------------///dht11
  delay(100);//Delay to let system boot
  Serial.println("DHT11 Humidity & temperature Sensor\n\n");
  delay(200);//Wait before accessing Sensor
 ///-------------------------------///end

 ///-------------------------------------///rc522
    servoen.attach(46);
    servoex.attach(47);
    SPI.begin();
    rfiden.init();
    rfidex.init();
    pinMode(leden, OUTPUT);
    pinMode(ledex, OUTPUT);
    digitalWrite(leden, LOW);
    digitalWrite(ledex, LOW);
 ///-------------------------------///end
 
 ///-------------------------------------///ZigBee
 Serial1.begin(9600);
 ///------------------------------///end
 
}


///--------------------------------------///loop

void loop() {///---------------------------------------------------------------------------------------------------------------///start of loop

  ///-------------------------------------------------------------------------------///start codes of srf04_xx
  ///--------------------------------///srf04_01 & turn on/off LED
   long duration1, distance1;
   int s1;
  digitalWrite(trigPin1, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1/2) / 29.1;
  if (distance1 < 5) {  // This is where the LED On/Off happens
    digitalWrite(led1,HIGH); // When this parking is full red led is turn on
    s1=1;
  }
  else {
    digitalWrite(led1,LOW);
    s1=0;
  }
  if (distance1 >= 300 || distance1 <= 0){
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
    int s2;
  digitalWrite(trigPin2, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2/2) / 29.1;
  if (distance2 < 5) {  // This is where the LED On/Off happens
    digitalWrite(led2,HIGH); // When this parking is full red led is turn on
    s2=1;
  }
  else {
    digitalWrite(led2,LOW);
    s2=0;
  }
  if (distance2 >= 300 || distance2 <= 0){
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
    int s3;
  digitalWrite(trigPin3, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3/2) / 29.1;
  if (distance3 < 5) {  // This is where the LED On/Off happens
    digitalWrite(led3,HIGH); // When this parking is full red led is turn on
    s3=1;
  }
  else {
    digitalWrite(led3,LOW);
    s3=0;
  }
  if (distance3 >= 300 || distance3 <= 0){
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
    int s4;
  digitalWrite(trigPin4, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  distance4 = (duration4/2) / 29.1;
  if (distance4 < 5) {  // This is where the LED On/Off happens
    digitalWrite(led4,HIGH); // When this parking is full red led is turn on
    s4=1;
  }
  else {
    digitalWrite(led4,LOW);
    s4=0;
  }
  if (distance4 >= 300 || distance4 <= 0){
    Serial.println("Out of range4");
  }
  else {
    Serial.print("distance4=");
    Serial.print(distance4);
    Serial.println(" cm");
  }
  //delay(50);


      ///------------------------------///srf04_05 & turn on/off LED
    long duration5, distance5;
    int s5; 
  digitalWrite(trigPin5, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin5, LOW);
  duration5 = pulseIn(echoPin5, HIGH);
  distance5 = (duration5/2) / 29.1;
  if (distance5 < 5) {  // This is where the LED On/Off happens
    digitalWrite(led5,HIGH); // When this parking is full red led is turn on
    s5=1;
  }
  else {
    digitalWrite(led5,LOW);
    s5=0;
  }
  if (distance5 >= 300 || distance5 <= 0){
    Serial.println("Out of range5");
  }
  else {
    Serial.print("distance5=");
    Serial.print(distance5);
    Serial.println(" cm");
  }
  //delay(50);


      ///------------------------------///srf04_06 & turn on/off LED
    long duration6, distance6;
    int s6;
  digitalWrite(trigPin6, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin6, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin6, LOW);
  duration6 = pulseIn(echoPin6, HIGH);
  distance6 = (duration6/2) / 29.1;
  if (distance6 < 5) {  // This is where the LED On/Off happens
    digitalWrite(led6,HIGH); // When this parking is full red led is turn on
    s6=1;
  }
  else {
    digitalWrite(led6,LOW);
    s6=0;
  }
  if (distance6 >= 300 || distance6 <= 0){
    Serial.println("Out of range6");
  }
  else {
    Serial.print("distance6=");
    Serial.print(distance6);
    Serial.println(" cm");
  }
  //delay(50);

    ///------------------------------///srf04_en & turn on/off LED
    long durationen, distanceen;
    int seren;
  digitalWrite(trigPinen, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinen, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinen, LOW);
  durationen = pulseIn(echoPinen, HIGH);
  distanceen = (durationen/2) / 29.1;
  if (distanceen < 5) {  // This is where the LED On/Off happens
    seren=1;
    }

  else {
    seren=0;
  }
  if (distanceen >= 300 || distanceen <= 0){
    Serial.println("Out of range en");
  }
  else {
    Serial.print("distanceen=");
    Serial.print(distanceen);
    Serial.println(" cm");
  }
  //delay(50);

      ///------------------------------///srf04_ex & turn on/off LED
    long durationex, distanceex;
    int serex;
  digitalWrite(trigPinex, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinex, HIGH);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinex, LOW);
  durationex = pulseIn(echoPinex, HIGH);
  distanceex = (durationex/2) / 29.1;
  if (distanceex < 5) {  // This is where the LED On/Off happens
    serex=1;
  }
  else {
    serex=0;
  }
  if (distanceex >= 300 || distanceex <= 0){
    Serial.println("Out of range ex");
  }
  else {
    Serial.print("distanceex=");
    Serial.print(distanceex);
    Serial.println(" cm");
  }
  //delay(50);

  ///-------------------------------------------------///end of srf04_xx


  ///-------------------------------------------------------------------------------///start codes of mq2
   int s7,s8;
   sensorValue = analogRead(analogInPin);            
  // determine alarm status
  if (sensorValue >= 200)
  {
    digitalWrite(FanPin, HIGH);   // sets the Fan on
  }
  else
  {
  digitalWrite(FanPin, LOW);    // sets the Fan off
  }

  // print the results to the serial monitor:
  Serial.print("smoke = " );
  s7=sensorValue&0xff;
  s8=(sensorValue>>8)&0xff;
  Serial.println(sensorValue);
  //delay(100);
   ///------------------------------------------------///end of mq2

   //--------------------------------------------------------------------------------///start codes of dht11 
  DHT.read11(dht_apin);
    int s9,s10;
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.println("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C  ");
    s9=DHT.humidity;
    s10=DHT.temperature;
    if (DHT.temperature >= 35 || DHT.humidity >= 50)
  {
    digitalWrite(FanPin, HIGH);   // sets the Fan on
  }
  else
  {
  digitalWrite(FanPin, LOW);    // sets the Fan off
  }
 
  //delay(100);//Wait 1 seconds before accessing sensor again.
  //-----------------------------------------------------///end of dht11

 
  //--------------------------------------------------------------------------------///start codes of rc522 entrance
  int s11_15[5],s11,s12,s13,s14,s15;
if(rfiden.isCard()){
    
        if(rfiden.readCardSerial()){
            Serial.print(rfiden.serNum[0]);
            Serial.print(" ");
            Serial.print(rfiden.serNum[1]);
            Serial.print(" ");
            Serial.print(rfiden.serNum[2]);
            Serial.print(" ");
            Serial.print(rfiden.serNum[3]);
            Serial.print(" ");
            Serial.print(rfiden.serNum[4]);
            Serial.println("");
            
            for(int x = 0; x < sizeof(cards); x++){
              access = true;
              for(int i = 0; i < sizeof(rfiden.serNum); i++ ){
                  if(rfiden.serNum[i] != cards[x][i]) {
                      s11_15[i]=rfiden.serNum[i];
                      access = false;
                      break;
                  }
              }
              if(access){ 
                s11=s11_15[0];
                s12=s11_15[1];
                s13=s11_15[2];
                s14=s11_15[3];
                s15=s11_15[4];
                break;
                }
            }
           
        }
        
       if(access){
          Serial.println("Welcome!");
          /////////////Interrupt--------------------------------------------/////////////////////////////////----------------------//////////////
           digitalWrite(leden, HIGH); 
           //delay(1000);
           digitalWrite(leden, LOW);
           ///servo control
           for (posen = 0; posen <= 90; posen += 1) { // goes from 0 degrees to 90 degrees
            // in steps of 1 degree
            servoen.write(posen);              // tell servo to go to position in variable 'pos'
            delay(30);                       // waits 30ms for the servo to reach the position
           }
            
           
      } else {
           Serial.println("Not allowed!"); 
           //digitalWrite(leden, HIGH);
           //delay(500);
           //digitalWrite(leden, LOW); 
           //delay(500);
           //digitalWrite(leden, HIGH);
           //delay(500);
           //digitalWrite(leden, LOW);         
       }        
    }
    
    
    
    rfiden.halt();
  //-----------------------------------------------------///end of rc522 entrance
  //waiting for entering car
  /*if(seren = 1){
    for (posen =90; posen > 2; posen -= 1) { // goes from 90 degrees to 0 degrees
    servoen.write(posen);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 30ms for the servo to reach the position
    }
  }*/
  //--------------------------------------------------------------------------------///start codes of rc522 exit
  int s16_20[5],s16,s17,s18,s19,s20;
if(rfidex.isCard()){
    
        if(rfidex.readCardSerial()){
            Serial.print(rfidex.serNum[0]);
            Serial.print(" ");
            Serial.print(rfidex.serNum[1]);
            Serial.print(" ");
            Serial.print(rfidex.serNum[2]);
            Serial.print(" ");
            Serial.print(rfidex.serNum[3]);
            Serial.print(" ");
            Serial.print(rfidex.serNum[4]);
            Serial.println("");
            
            for(int x = 0; x < sizeof(cards); x++){
              access = true;
              for(int i = 0; i < sizeof(rfidex.serNum); i++ ){
                  if(rfidex.serNum[i] != cards[x][i]) {
                      access = false;
                      break;
                  }
              }
              if(access) {
                s16=s16_20[0];
                s17=s16_20[1];
                s18=s16_20[2];
                s19=s16_20[3];
                s20=s16_20[4];
                break;
                }
            }
           
        }
        
       if(access){
          Serial.println("See you soon!");
          /////////////Interrupt--------------------------------------------/////////////////////////////////----------------------//////////////
           digitalWrite(ledex, HIGH); 
           //delay(1000);
           digitalWrite(ledex, LOW);
           ///digitalWrite(power, HIGH);
           ///delay(1000);
           ///digitalWrite(power, LOW);
           ///servo control
           for (posex = 0; posex <= 90; posex += 1) { // goes from 0 degrees to 90 degrees
            // in steps of 1 degree
            servoex.write(posex);              // tell servo to go to position in variable 'pos'
            delay(30);                       // waits 30ms for the servo to reach the position
            }
           
      } else {
           Serial.println("Not allowed!"); 
           //digitalWrite(ledex, HIGH);
           //delay(500);
           //digitalWrite(ledex, LOW); 
           //delay(500);
           //digitalWrite(ledex, HIGH);
           //delay(500);
           //digitalWrite(ledex, LOW);         
       }        
    }
    
    
    
    rfidex.halt();
  //-----------------------------------------------------///end of rc522 exit
  ///*waiting for exiting car
  /*if(serex = 1);{
    for (posex =90; posex > 2; posex -= 1) { // goes from 90 degrees to 0 degrees
      servoex.write(posex);              // tell servo to go to position in variable 'pos'
      delay(30);                       // waits 30ms for the servo to reach the position
    }
  }*/
  ///---------------------------------------------------///ZigBee
  ///------------------///sending
  Serial1.print(ss1);
  Serial1.print(ss2);
  Serial1.print(ss3);
  Serial1.print(ss4);
  Serial1.print(s1);
  Serial1.print(s2);
  Serial1.print(s3);
  Serial1.print(s4);
  Serial1.print(s5);
  Serial1.print(s6);
  Serial1.print(s7);
  Serial1.print(s8);
  Serial1.print(s9);
  Serial1.print(s10);
  Serial1.print(s11);
  Serial1.print(s12);
  Serial1.print(s13);
  Serial1.print(s14);
  Serial1.print(s15);
  Serial1.print(s16);
  Serial1.print(s17);
  Serial1.print(s18);
  Serial1.print(s19);
  Serial1.print(s20);

  ///-------------------///receiving
  if (Serial1.available() > 0) {
    byte fd = Serial1.read();
    Serial.println(int(fd));
    if(fd==253){ 
    
      for(int i=0 ; i<10 ; i++){
        if(Serial1.available()>0)
          Serial.println(int(Serial1.read()));
        else
          i--;
      }
    }
  }
  
  ///-------------end of ZigBee
}

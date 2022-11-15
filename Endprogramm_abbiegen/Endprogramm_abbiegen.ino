//Magnetometer Kofiguration
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);
// Initialize variables

//Lunar Konfiguration
#include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0
TFMPlus tfmP;         // Create a TFMini Plus object

#include "printf.h"

#include <SoftwareSerial.h> //header file of software serial port
SoftwareSerial Serial2(12,11); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX


//Globale Variablen: Lidar
int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package

//Globale Varaiblen: Magnetometer
float heading; //Magnetometer Wert
float sHeading;
float angle;
float angle_2;

//mittelwert
float angle_m;

//Motor Konfiguration
  // Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
  // Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;


void setup() {
  //LIDAR Setup
    Serial.begin(115200);   // Intialize terminal serial port
    delay(20);               // Give port time to initalize
    printf_begin();          // Initialize printf.
    printf("\r\nTFMPlus Library Example - 10SEP2021\r\n");

    Serial2.begin(115200);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    tfmP.begin( &Serial2);   // Initialize device library object and...
                             // pass device serial port to the object.

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - - - - - - - - - -
    printf( "Soft reset: ");
    if( tfmP.sendCommand( SOFT_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP.printReply();
  
    delay(500);  // added to allow the System Rest enough time to complete

  // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP.sendCommand( GET_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.", tfmP.version[ 0]); // print three single numbers
        printf( "%1u.", tfmP.version[ 1]); // each separated by a dot
        printf( "%1u\r\n", tfmP.version[ 2]);
    }
    else tfmP.printReply();
    // - - Set the data frame-rate to 20Hz - - - - - - - -
    printf( "Data-Frame rate: ");
    if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_20))
    {
        printf( "%2uHz.\r\n", FRAME_20);
    }
    else tfmP.printReply();


  delay(500);            // And wait for half a second.

      // Magnetometer Setup
  Serial.begin(115200);
  Serial.println("Magnetometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  
  
  //Motor Setup
    // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  //Anfangswinkel festelegen
  magnetometer();
  sHeading = angle;
  
  //New_Coordinate aktivieren lassen
  /*new_coordinate();
  for(int j=1;j<30;j++){
    new_coordinate();
    Serial.println(angle_2);
    delay(100);
  }*/
}

void lane_keeper() {
  new_coordinate();
  Serial.print(sHeading);
  Serial.print("\t");
  Serial.print(angle);
  Serial.print("\t");
  Serial.println(angle_2);
  if (angle_2 > 180.0 && angle_2 < 356.0){
    turnright();
    new_coordinate();
  }
  else if (angle_2 > 4.0 && angle_2 <= 180.0 ){
    turnleft();
    new_coordinate();
  }
  else{
    aidroidForward();
    new_coordinate();
  }
}


#define anzahlMittelWerte 20
int werte[anzahlMittelWerte], zaehlerMittelWerte=0;

float mittelWert(int neuerWert){
  float mittel, summe =0;
  if (neuerWert <= (sHeading+15.0) or neuerWert >= (sHeading -15.0)){
    if(neuerWert < 180)neuerWert += 360.0;
    werte[zaehlerMittelWerte] = neuerWert;
    for(int k=0; k<anzahlMittelWerte; k++) summe += werte[k];
    mittel=(float)summe/anzahlMittelWerte;
    zaehlerMittelWerte++;
    if(zaehlerMittelWerte >= anzahlMittelWerte) zaehlerMittelWerte=0;
  }
  else{
    mittel=neuerWert;
  }
  return fmod(mittel,360.0);
}

float circle_mod(float x){     // damit immer zwischen 0 und 360 bleibt.
  return fmod(x+360.0, 360.0);
}
float new_coordinate_einzel(float x){ //Einzel umwandeln: Momentan unbrauchbar
  return x-sHeading;
}
void new_coordinate(){ // Ganze Koordinatensystem in einem neuen umwandeln, wobei der Anfangswinkel gleich 0 ist. 
  magnetometer();
  angle_2 = circle_mod(angle-sHeading);
}

int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

void lunarRead()
{
    delay(50);   // Loop delay to match the 20Hz data frame rate

    if( tfmP.getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
    {
      dist=tfDist;
    }
    else                  // If the command fails...
    {
      tfmP.printFrame();  // display the error and HEX data
    }
}

void aidroidForward(){
  //rightwheel: in4 , in3 (enB)
  //leftwheel: in1 and in2 (enA)
  analogWrite(enA,255); //Spannungregler(0-255) 200 //Ohne Magnetometer 230 und 185 (160)
  analogWrite(enB,255); // vielleicht 143
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void aidroidStop(){
  analogWrite(enA,255); //200
  analogWrite(enB,255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnright(){  //überprüfen ob es stimmt, also turn right and trun left
  analogWrite(enA,255); //Batterie 180
  analogWrite(enB,255); //Batterie 150
  digitalWrite(in1, HIGH);//Linkes Rad vorwärts 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); //rechtes Rad rückwärts HIGH
}

void turnleft(){
  analogWrite(enA,255); 
  analogWrite(enB,255);//Batterie 180
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); //linkes rad rückwärts HIGH
  digitalWrite(in3, HIGH);//rechtes Rad vorwärts
  digitalWrite(in4, LOW);
}

void magnetometer(){
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  if (heading > 0 && heading <= 360) angle = heading;  //Störung 0.00 weglassen
  delay(50);
}

int j=0; //Anzahlschritte zu speichern
void checking1(){
  do{
    do{
      turnright();
      magnetometer();
    }
    while (angle < circle_mod(sHeading+87.0) or angle > circle_mod(sHeading+93.0));
    aidroidForward();
    delay(2000);
    j++;
    aidroidStop();
    delay(500);
    do{
      turnleft();
      magnetometer();
    }
    while(angle < circle_mod(sHeading-3.0) or angle > circle_mod(sHeading+3.0)); //+- 3 vom sHeading
    aidroidStop();
    delay(1000);
    lunarRead();
  }
  while(dist < 40);
}

void checking2(){
  do{
    do{
      turnright();
      magnetometer();
    }
    while (angle < circle_mod(sHeading-3.0) or angle > circle_mod(sHeading+3.0));
    aidroidForward();
    delay(1000);
    aidroidStop();
    delay(500);
    do{
      turnleft();
      magnetometer();
    }
    while(angle  < circle_mod(sHeading-93.0) or angle > circle_mod(sHeading-87.0));
    aidroidStop();
    delay(1000);
    lunarRead();
  }
  while(dist < 40);
}
void obstacle_avoid(){
  aidroidStop();
  delay(1000);
  checking1();
  Serial.println("Checking 1 finished");
  aidroidStop();
  delay(1000);
  aidroidForward(); //Überholung
  delay(3000);
  aidroidStop();
  delay(1000);
  do{
    turnleft();
    magnetometer();
  }
  while(angle < circle_mod(sHeading-93.0) or angle > circle_mod(sHeading-87.0));
  checking2();
  Serial.println("Checking 2 finished");
  aidroidForward();
  delay(2000*j);
  do{
    turnright();
    magnetometer();
  }
  while(angle < circle_mod(sHeading-3.0) or angle > circle_mod(sHeading+3.0));
}

void loop(){
  Serial.print("Hallo");
  lunarRead();
  if (dist < 40){
    obstacle_avoid();
  }
  else{
    aidroidForward();
  }
}


/*  lunarRead();
  while (dist < 60){
    aidroidStop();
    turnleft();
    lunarRead();
  }
  aidroidStop();
  magnetometer();
  float tilt_angle = angle; //aktueller Winkel
  float interval = circle_mod(sHeading-tilt_angle); //unterschied berechnen
  Serial.println(interval);
  */

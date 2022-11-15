//Magnetometer Kofiguration
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);


//Lunar Konfiguration
#include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0
TFMPlus tfmP;         // Create a TFMini Plus object

#include "printf.h"

#include <SoftwareSerial.h> //header file of software serial port
SoftwareSerial Serial2(12,11); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX

int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package

//mag
float heading; //Magnetometer Wert
float sheading;
float angle;
float angle_2;


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
    Serial.begin(115200);   // Intialize terminal serial port
    delay(20);               // Give port time to initalize
    printf_begin();          // Initialize printf.
    printf("\r\nTFMPlus Library Example - 10SEP2021\r\n");  // say 'hello'

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

  // Magnetometer
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

  //Anfangswinkel festelegen schon im Void setup()
  magnetometer();
  sheading = angle;
}

// Initialize variables
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

// Use the 'getData' function to pass back device data.

void loop(){ //pwm 130 => delay(390)
  lunarRead();
  if (dist < 40){
    overtake();
    lunarRead();
  }
  else{
    aidroidForward();
    lunarRead();
  }
}
void overtake(){
  aidroidStop();
  delay(2000);
  long int t1 = millis();
  do{
    turnright();
    lunarRead();
  }
  while(dist < 100);
  long int t2 = millis();
  int t3= t2-t1;
  Serial.println(t3);
  aidroidStop();
  delay(500);
  turnright(); //noch einwenig weiter drehen
  delay(800);
  aidroidStop();
  delay(500);
  aidroidForward();
  delay(2000);
  aidroidStop();
  delay(1000);
  turnleft();
  delay(t3+800);
  aidroidForward();
  delay(2000);
  aidroidStop();
  delay(500);
  turnleft();
  delay(t3+800);
  aidroidStop();
  delay(500);
  aidroidForward();
  delay(2000);
  aidroidStop();
  delay(500);
  turnright();
  delay(t3+800);
  aidroidStop();
  delay(500);
}


  /*
}
}
  lunarRead();
  if (dist < 20){
    aidroidStop();
    delay(500);
    obstacle_avoid();
    aidroidForward();
    delay(5000);
    aidroidStop();
    delay(500);
    after_correction();
    lunarRead();
  }
  
  else{
    aidroidForward();
    lunarRead();
  }
}
*/
/*
void obstacle_avoid(){
  lunarRead();
  while (dist < 20){
    obstacle_maneuver();
    i+=1;
    lunarRead();
  }
}
*/
/*void after_correction(){
  aidroidStop();
  delay(500);
  turnright();
  delay(250);
  aidroidStop();
  delay(500);
  aidroidForward();
  delay(500*i);
  aidroidStop();
  delay(500);
  turnleft();
  delay(250);
  aidroidStop();
  delay(500);
}
*/

/*void obstacle_maneuver(){
  aidroidStop();
  delay(500);
  turnleft();
  delay(250);
  aidroidForward();
  delay(500);   // vorwärtsfahren
  aidroidStop();
  delay(500);
  turnright();
  delay(250);
  aidroidStop();
  delay(500);
}
*/
void lane_keeper() {
  new_coordinate();
  Serial.print(sheading);
  Serial.print("\t");
  Serial.println(angle_2);
  if (angle_2 > 180.0 && angle_2 < 356.0){
    aidroidStop();
    delay(10);
    turnright();
    delay(10);
    aidroidStop();
    new_coordinate();
  }
  else if (angle_2 > 4.0 && angle_2 <= 180.0 ){
    aidroidStop();
    delay(10);
    turnleft();
    delay(10);
    aidroidStop();
    new_coordinate();
  }
  else{
    aidroidForward();
    new_coordinate();
  }
}

/*
  aidroidForward();
  while (angle < (fmod(sheading-7.0 + 360.0,360.0))){  //+360.0 Korrektur damit keine negative Wert ausgibt.
    aidroidStop();
    delay(100);
    turnright();
    delay(100);
    aidroidStop();
    magnetometer();
  }
  while (angle > (fmod(sheading + 7.0,360.0))){ //fmod(), modulo von 360 mit Float
    aidroidStop();
    delay(100);
    turnleft();
    delay(100);
    aidroidStop();
    magnetometer();
  }
}

*/
float circle_mod(float x){     // bleibt immer zwischen 0 und 360.
  return fmod(x+360.0, 360.0);
}
float new_coordinate_einzel(float x){ //Einzel umwandeln: Momentan unbrauchbar
  return x-sheading;
}
void new_coordinate(){ // Ganze Koordinatensystem in einem neuen umwandeln, wobei der Anfangswinkel gleich 0 ist. 
  magnetometer();
  angle_2 = circle_mod(angle-sheading);
}

void lunarRead()
{
    delay(50);   // Loop delay to match the 20Hz data frame rate

    if( tfmP.getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
    {
      dist=tfDist;
      /*
      printf( "Dist:%04icm ", tfDist);   // display distance,
      printf( "Flux:%05i ",   tfFlux);   // display signal strength/quality,
      printf( "Temp:%2i%s",  tfTemp, "C");   // display temperature,
      printf( "\r\n");   */                // end-of-line.
    }
    else                  // If the command fails...
    {
      tfmP.printFrame();  // display the error and HEX dataa
    }
}

void aidroidForward(){
  //rightwheel: in4 , in3
  //leftwheel: in1 and in2
  analogWrite(enA,200); //Spannungregler(0-255) 200
  analogWrite(enB,200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void aidroidStop(){
  analogWrite(enA,200);
  analogWrite(enB,200);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnright(){  //überprüfen ob es stimmt, also turn right and trun left
  analogWrite(enA,255); //Batterie 180
  analogWrite(enB,255); //Batterie 150
  digitalWrite(in1, HIGH); //Linkes Rad vorwärts
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); //rechtes Rad rückwärts HIGH
}

void turnleft(){
  analogWrite(enA,255); 
  analogWrite(enB,255);//Batterie 180
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);//linkes rad rückwärts
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
  if (heading > 0 && heading <= 360) angle= heading;
}

/*
 * Author: James Howard
 * Email: james@outoftolerance.com
 * Website: www.outoftolerance.com
 * Released under the MIT Licence
 */

/*
 * Command packet description
 * 6 chars total length
 * 1st char is the @ symbol, denotes a new command
 * 2nd char is an alphabetical character which describes the type of command
 * 3rd - 6th chars are the payload for that command
 */

/*
 * Types of commands:
 * s = speed update, will send speed in percentage form from 0 to 100 (chars 4th - 6th) with +/- sign in 3rd char
 * c = callibrate, executes the callibrate function, no payload (zeros)
 */

//required libraries from Arduino
#include <SoftwareSerial.h>
#include <Servo.h>

//version statement
#define VERSION 1.0

//define the type of ESC(0 = car, 1 = aircraft). 
//Car ESCs call 1000ms neutral and can go in reverse, aircraft ones cannot
#define ESC_TYPE 1

#define BUFFER_SIZE 64                    //buffer size for serial, kinda arbitrary...
#define MESSAGE_SIZE 6                    //number of chars in a total message
#define PAYLOAD_SIZE 5                    //size of the message payload in chars length
#define RX 13                             //receive software serial port
#define TX 12                             //send software serial port
#define ESC 11                            //ESC port
#define BT_STATUS 10                      //Pin which the bluetooth connection status is shown on
#define SERIAL_TYPE 1                     //define which serial port to use (0 = USB, 1 = bluetooth)
#define DAMPING 0.2                       //damping ratio for the filter, want this small so it doesn't take too long to follow command

//create a software serial instance for the bluetooth module
SoftwareSerial blueSerial(RX, TX);

//create a servo object for the speed controller
Servo esc;

//some variables we are going to need later
String serial_buffer = "";                //for storing the serial bytes from the buffer
String command_string = "";               //for storing the command string extracted from the buffer
int command_percent = 0;                  //commanded percent from the controller
int command_speed = 0;                    //speed commanded by the controller
int esc_speed = 0;                        //current speed sent to the ESC (after smoothing)

//Function prototypes
String serialRead(void);
void serialWriteLn(String message);
void serialWrite(String message);

void setup() {
  //start serial and print message
  Serial.begin(9600);  
  blueSerial.begin(9600);
  blueSerial.print("Skateboard Controller V");
  blueSerial.println(VERSION);

  //attach ESC to correct pin
  esc.attach(ESC);

  //set ESC PWM to neutral before starting the program
  if(ESC_TYPE == 0) {
    esc.writeMicroseconds(1000);     //aircraft ESC to zero
  }
  else if(ESC_TYPE == 1) {
    esc.writeMicroseconds(1500);  //Car ESC to 1500 (middle)
  }
}

//OK LEEEROY LET'S DO THIS!
void loop() {
  //get the latest info from the serial port
  serial_buffer = readSerial();

  //search the buffer for the command symbol (@)
  if(serial_buffer.indexOf('@') != -1) {
    //new command found! Get the index of the command start
    command_string = serial_buffer.substring(serial_buffer.indexOf('@'), serial_buffer.length());
    writeSerial("Command detected!");
  }

  //depending on what the command was we need to do stuff now
  switch(command_string[1]) {
    case 's':
      //speed update for the controller
      updateSpeed(command_string.substring(2, command_string.length()));
      break;
  }

  //execute the skateboard controll loop
  skateControl(esc_speed, command_speed, DAMPING);

  //read the sensors on board
  //readSensors();

  //clear out the serial buffer
  serial_buffer = "";
  command_string = "";

  //delay a wee bit
  delay(50);
}

//skateboard controller
void skateControl(int actual, int commanded, int damping) {
  //update the esc speec based on control speed and damping
  actual = expFilt(damping, actual, commanded);

  //write the speed to the esc
  esc.writeMicroseconds(actual);
}

//function updates the speed from the controller to the ESC
void updateSpeed(String payload) {
  //convert number from hex to decimal
  command_percent = payload.toInt();

  //limit the percent incase of base packet
  if(ESC_TYPE == 0) {
    //car type ESC
    command_percent = endLimit(command_percent, -100, 100);         //limit so it doesn't go so far
    command_speed = map(command_percent, -100, 100, 1000, 2000);    //convert from percent to the microseconds
  }
  else if(ESC_TYPE == 1) {
    //airplane type ESC
    command_percent = endLimit(command_percent, 0, 100);            //limit so it doesn't go so far
    command_speed = map(command_percent, 0, 100, 1000, 2000);       //convert from percent to the microseconds
  }  

  String temp_message;
  temp_message = "Updated speed to: ";
  temp_message += command_percent;
  
  writeSerial(temp_message);
}

//read messages from the serials
String readSerial() {
  char inData[BUFFER_SIZE];
  int index = 0;
  
  while(Serial.available()){
    inData[index] = Serial.read();
    index++;
  }
  while(blueSerial.available()) {
    inData[index] = Serial.read();
    index++;
  }

  return String(inData);
}

//print a message to the serials
void writeSerial(String message) {
  Serial.println(message);
  blueSerial.println(message);
}

//function that reads each of the in board sensors
void readSensors() {
  readAccel();
  readGyro();
  readMag();
}

//reads the on board accelerometer
void readAccel(){
  
}

//reads the on board gyroscope
void readGyro() {
  
}

//reads the on board magnetometer
void readMag() {
  
}

//exponential moving average filter
int expFilt(int damping, int current, int raw) {
  return ((damping * raw) + ((1 - damping) * current));
}

//limits a number to end limits
int endLimit(int val, int minimum, int maximum) {
  if(val < minimum)
    return minimum;
  else if(val > maximum)
    return maximum;
  else
    return val;
}


//Written By: Severin Zaluzec
//other stuff

//Velocity pin definition
double x=0; //Velocity reading; Outgoing message
double xvel;
double zth;  //Steering angle; Outgoing message
//Driver pinrs
int LPWM = 5;
int RPWM = 6;
int R_IS = 8;
int L_IS = 7;
int L_EN = 4;
int R_EN = 12; //R_EN changed from pin 2 because interrupt
         
//Testing outgoing messages
int test1 = 0;
int test2 = 0;
//Timer to check if a acceleration command has been recieved
unsigned long timer=millis();




//this is encoder stuff
#include "Ticker.h"
#include <Encoder.h>
bool getSensor();
void movemotor();
void printvelocity();
void stopmotor();

#define MEASURE_VELOCITY_INTERVAL  200 // Interval between calls for the encoder function
#define MOTOR_COMMAND_INTERVAL  3      // Interval betweeen calls for the steering function
#define PRINT_VELOCITY_INTERVAL 200   // Interval between calls for the print function

Ticker timer1(getSensor, MEASURE_VELOCITY_INTERVAL, 0, MILLIS); // Get encoder data every 200ms
Ticker timer2(movemotor, MOTOR_COMMAND_INTERVAL, 0, MILLIS); // Send position commands for steeriing every 3ms
Ticker timer3(printvelocity,PRINT_VELOCITY_INTERVAL,0,MILLIS); // Write velocity and steering angle over serial every 200ms


int Pos1; //Position encoder 1
int Pos2; //Position encoder 2
Encoder myEnc1 (2,3); //Encoder 1
Encoder myEnc2 (21,20); //Encoder 2

#define PPR 1000       // Pulses Per Revolution (adjust based on your encoder)
#define WHEEL_RADIUS 0.191 // Radius of the wheel in meters (adjust as needed)
#define WHEEL_GEAR_RATIO 1 // Placeholder, ratio of encoder counts to wheel rotation

const int l = 5; //Size of rolling average

long sum1 = 0; //Sum used in averaging "left" wheel
long sum2 = 0; //Sum used in averaging "right" wheel
long zthsum = 0; //Sum used in averaging steering angle readings
int pulseslist1 [l]; //List used for rolling average of "left" wheel
int pulseslist2 [l]; //List used for rolling average of "right" wheel
int zthlist [l]; //List used in rolling average of steering
int i = 0; //Position in array of rolling 
float average1 = 0;
float average2 = 0;
float averagezth = 0;

float linearSpeed1 = 0;
float linearSpeed2 = 0;
float steeringPosRad = 0;

bool print_velocity = false;

//PID
#include <PID_v1_bc.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3
#define MAX_INPUT  12


//PID values need tuning
double Setpoint, Input, Output;
double Kp=0.5, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//Motor
int dir;
#include "BasicStepperDriver.h"
#define MOTOR_STEPS 1600
#define MOTOR_ACCEL 10000
#define MOTOR_DECEL 10000

#define RPM 750
#define MICROSTEPS 1
#define DIR 10
#define STEP 11
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);
















//These functions read the serial data
void processIncomingByte (); 
void process_data ();

char * commands[2];

void setup() {
  //this is velocity stuff
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(LPWM, OUTPUT);
  pinMode (RPWM, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode (R_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode (R_EN, OUTPUT);
  
  digitalWrite(L_IS, LOW);
  digitalWrite(R_IS, LOW);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  Setpoint = 512;
  //this is encoder stuff
  timer1.start();
  timer2.start(); 
  timer3.start();
  // timer4.start();
  pinMode(A0, INPUT);
  stepper.begin(RPM, MICROSTEPS);
  //initialize the variables we're linked to
  Input = analogRead(A0);
  Setpoint = 512;
  myPID.SetOutputLimits(0,100);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  stepper.enable();

}

void loop() {
  //this is encoder stuff
  timer1.update();
  timer2.update();
  timer3.update();
  // timer4.update();


// //  //this is velocity stuff
  while (Serial.available () > 0) //Check if there are items in the buffer
  {
    processIncomingByte(Serial.read()); //read velocity and steering commands
    timer=millis();
    
  }
 if (print_velocity == true)
 {
  // Serial.println(String(linearSpeed1) + "," + String(linearSpeed2) + "," + String(averagezth)); // in case encoder 2 is replaced
  Serial.println(String(linearSpeed1) + "," + String(steeringPosRad)); //print encoder data

  print_velocity = false; //reset flag
 }

 // auto sleep check
 if (millis()-timer >500) //set velocity to 0 if no velocity is sent after 500ms
  {
    timer=millis(); //reset timer
    analogWrite(LPWM,0); //0 to forward
    analogWrite(RPWM,0); //0 to backwards
    commands[0] = 0;  //set velocity command to 0
    // Serial.println(millis()-timer);
  }

  // control motor from serial input
  x = String(commands[0]).toDouble(); //cast velocity command as double
  double velocity= x*255.0;
  int y  = (velocity);
  //limit output for motor drivers to between 0 and 255
  if (y>255)
  {
    y=255;
  }

  if(y<-255)
  {
    y = -255;
  }



  if (y<0) //Backwards commands
  {
    analogWrite(LPWM,0);
    analogWrite(RPWM,abs(y));
  }
  if (y>=0) //Forward commands
  {
    analogWrite(RPWM,0);
    analogWrite(LPWM,abs(y));
 
  }

 }

//Average across the "l" most recent encoder values. In this case, l=5.
void rollingaverage (int pulses1, int pulses2, int steerPosAna) {
  sum1 = sum1 - pulseslist1[i];
  sum2 = sum2 - pulseslist2[i];
  zthsum = zthsum - zthlist[i];
  //  Serial.println("pulses1: " + String(pulses1));
  //  Serial.println(rpmlist[i]);

  pulseslist1[i] = pulses1;
  pulseslist2[i] = pulses2;
  zthlist[i] = steerPosAna;

  sum1 = sum1 + pulseslist1[i];
  sum2 = sum2 + pulseslist2[i];
  zthsum = zthsum + zthlist[i];

  i = i + 1;
  if (i >= l)
  {
    i = 0;
  }
  average1 = (float)sum1 / l;
  average2 = (float)sum2/l;
  averagezth= (float)zthsum/l;

} 

//Read encoder values
bool getSensor() {
  // read pulses from both rear encoders, and the analog reading from hall sensor
  int pulses2 = myEnc2.readAndReset();
  int pulses1 = pulses2; // use myEnc1.readAndReset(); when encoder 1 is fixed
  int steerPosAna = analogRead(A0);

  // average of pulses1 and pulses2
  rollingaverage(pulses1*WHEEL_GEAR_RATIO, pulses2*WHEEL_GEAR_RATIO, steerPosAna);
    // Calculate rotational wheel speed
    float RPS1 = average1 / PPR; // Revolutions per second
    float RPS2 = average2 / PPR; // Revolutions per second
    // Calculate linear car speed
    linearSpeed1 = RPS1 * (2 * PI * WHEEL_RADIUS); // v = RPS * circumference
    linearSpeed2 = RPS2 * (2 * PI * WHEEL_RADIUS); // v = RPS * circumference
    // Calculate position in rad
    steeringPosRad = ((averagezth - 512) * 2.0 * PI) / 1024.0; // Convert to radians

    return true;
}
 

// bool getSensor() {
//   Pos1 = myEnc1.read();
//   Pos2 = myEnc2.read();
//   //Serial.println("Pos1: " + String(Pos1));
//   //Serial.println("Pos2: " + String(Pos2));
//   rollingaverage(float(Pos1)*60/2400,float(Pos2)*60/2400);  
//   myEnc1.write(0);
//   myEnc2.write(0);
//   return true;
// }

//Set state of flag to determine when endocer values are published
void printvelocity()
{
  print_velocity = true;
}


//PID control loop for steering servo
void movemotor(){
  // Serial.println("moved motor");
  stepper.stop();
  stepper.disable();
  stepper.enable();
  // Serial.println(String(commands[1]).toDouble());
  if (String(commands[1]).toDouble()!=0.00 && String(commands[1]).toDouble()>450 && String(commands[1]).toDouble()<600 ) //Check to makesure setpoint is within acceptable range.
  {
    Setpoint = String(commands[1]).toDouble();
    // Serial.println("inif");
  }
//  Serial.println("Setpoint: " + String(Setpoint));
 Input = analogRead(A0); //Read steering encoder
 if (100<Input<800)
 {
     if ((Setpoint - Input) < 0)
   {
     myPID.SetControllerDirection(REVERSE);
     dir = -1;
   }

   if ((Setpoint - Input) > 0)
   {
     myPID.SetControllerDirection(DIRECT);
     dir = 1;
   }
   myPID.Compute();
  //  Serial.println(dir*Output);
  // Serial.println("Output: " + String(dir*Output));
  //  stepper.startMove(dir*Output);
  
  //  stepper.nextAction(); 
  stepper.move(dir*Output);
   
 }
 

}


// After receiving a full message based on max message size, process that message and fill its data into the commands array
void process_data (char * data)
  {
    char * token;
    //char * val1;
    //const int c1 = data.indexOf(',');
    //val1 = strtok(data,",");
    token = strtok(data, ",");
    int i = 0;

    while (token != NULL) {
      
      if (i <=1) 
      {
        commands[i] = token;
      }
      token=strtok(NULL, ",");
      i++;

   }
  }  // end of process_data



//Read serrial message
void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;
//  Serial.print("In byte: ");
//  Serial.println(inByte);
//  Serial.print("Input pos: ");
//  Serial.println(input_pos);

  switch (inByte)
    {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte
      
      // terminator reached! process input_line here ...
      process_data (input_line);
      
      // reset buffer for next time
      input_pos = 0;  
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch
   
  } // end of processIncomingByte



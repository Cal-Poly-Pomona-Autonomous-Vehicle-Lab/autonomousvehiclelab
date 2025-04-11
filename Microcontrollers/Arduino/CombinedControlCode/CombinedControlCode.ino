//Written By: Severin Zaluzec, Matthew Jabson, Abhishek Vishwakarma
//other stuff

// Matthew tried to make fixes to the code in a few hours to fix it, don't judge me

//Velocity pin definition
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

#define PI 3.14


//this is encoder stuff
#include <Ticker.h>
#include <Encoder.h>
bool getSensor();
void movesteermotor();
void printvelocity();
void stopmotor();
void updateVelocityControl();

#define MEASURE_VELOCITY_INTERVAL  200 // Interval between calls for the encoder function
#define MOTOR_COMMAND_INTERVAL  3      // Interval betweeen calls for the steering function
#define PRINT_VELOCITY_INTERVAL 200   // Interval between calls for the print function

Ticker timer1(getSensor, MEASURE_VELOCITY_INTERVAL, 0, MILLIS); // Get encoder data every 200ms
Ticker timer2(movesteermotor, MOTOR_COMMAND_INTERVAL, 0, MILLIS); // Send position commands for steeriing every 3ms
Ticker timer3(printvelocity,PRINT_VELOCITY_INTERVAL,0,MILLIS); // Write velocity and steering angle over serial every 200ms
Ticker motorVelocityTicker(updateVelocityControl,MOTOR_COMMAND_INTERVAL,0, MILLIS);

int Pos1; //Position encoder 1
int Pos2; //Position encoder 2
Encoder myEnc1 (2,3); //Encoder 1
Encoder myEnc2 (21,20); //Encoder 2

#define PPR 600       // Pulses Per Revolution (adjust based on your encoder)
#define WHEEL_RADIUS 0.191 // Radius of the wheel in meters (adjust as needed)
#define WHEEL_GEAR_RATIO 5 // Placeholder, ratio of encoder counts to wheel rotation

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

double requested_radian_position = 0; //cast radian command as double
// double requested_velocity_position = 0; //cast velocity command as double
double current_velocity_position = 0.0;
double target_velocity_position = 0.0;


bool print_velocity = false;

//PID
#include <PID_v1_bc.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3
#define MAX_INPUT  12


//PID values need tuning
double SetpointSteer, InputSteer, OutputSteer;
double stKp=0.5, stKi=0.0001, stKd=0;
PID SteerPID(&InputSteer, &OutputSteer, &SetpointSteer, stKp, stKi, stKd, DIRECT);

//PID values need tuning
double SetpointSpeed, InputSpeed, OutputSpeed;
double spKp=50, spKi=90, spKd=5;
PID SpeedPID(&InputSpeed, &OutputSpeed, &SetpointSpeed, spKp, spKi, spKd, DIRECT);

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

  SetpointSteer = 512;
  SetpointSpeed = 0;
  //this is encoder stuff
  timer1.start();
  timer2.start();
  timer3.start();
  motorVelocityTicker.start();
  pinMode(A0, INPUT);
  stepper.begin(RPM, MICROSTEPS);
  //initialize the variables we're linked to
  InputSteer = analogRead(A0);
  SetpointSteer = 512;
  SteerPID.SetOutputLimits(0,100);
  //turn the PID on
  SteerPID.SetMode(AUTOMATIC);
  stepper.enable();

  SpeedPID.SetMode(AUTOMATIC);
  SpeedPID.SetOutputLimits(-255, 255);
  


}

void loop() {
  //this is encoder stuff
  timer1.update();
  timer2.update();
  timer3.update();
  motorVelocityTicker.update();
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

  print_velocity = false; //reset flag  Serial.println("Basic Encoder Test:");
 }

 // auto sleep check
 if (millis()-timer >500) //set velocity to 0 if no velocity is sent after 500ms
  {
    timer=millis(); //reset timer
    analogWrite(LPWM,0); //0 to forward
    analogWrite(RPWM,0); //0 to backwards
    target_velocity_position = 0.0;  //set velocity command to 0
    // Serial.println(millis()-timer);
  }

  // control motor from serial input
//   int y  = (int) (target_velocity_position*255);

//   // Note from Matt: the velocity is coming as m/s, the pwm does not mean speed, you need a new PID to control the speed instead
//   // someone with more time please put that fix in

//   //limit output for motor drivers to between 0 and 255
//   if (y>255)
//  {
//     y=255;
//   }

//   if(y<-255)
//   {
//     y = -255;
//   }


//   // Serial.println("PWM value: "+String(y));
//   if (y<0) //Backwards commands
//   {
//     analogWrite(LPWM,0);
//     analogWrite(RPWM,abs(y));
//   }
//   if (y>=0) //Forward commands  requested_velocity_position = String(commands[0]).toDouble(); //cast velocity command as double
//  //limit output for motor drivers to between 0 and 255
//   if (y>255)
// …  // Serial.println("PWM value: "+String(y));
//   if (y<0) //Backwards commands
//   {
//     analogWrite(LPWM,0);
//     a
//   {
//     analogWrite(RPWM,0);
//     analogWrite(LPWM,abs(y));
 
//   }

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
  Serial.println("Pulses from encoder: "+String(pulses2));
  int pulses1 = pulses2; // use myEnc1.readAndReset(); when encoder 1 is fixed
  int steerPosAna = analogRead(A0);

  // average of pulses1 and pulses2
  rollingaverage(pulses1, pulses2, steerPosAna);
    // Calculate rotational wheel speed
    float CPR = PPR * 4 * WHEEL_GEAR_RATIO; // quadrature encoder
    const float interval_sec = MEASURE_VELOCITY_INTERVAL/1000.0; 
    float RPS1 = (average1 / CPR) / interval_sec; // Revolutions per second
    float RPS2 = (average2 / CPR) / interval_sec; // Revolutions per second
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

int radiansToAnalog(double radians) {
  int analogValue = int((radians * 1024.0) / (2.0 * PI) + 512.0);
  
  // Clamp value to 0–1023 just in case
  if (analogValue < 0) analogValue = 0;
  if (analogValue > 1023) analogValue = 1023;

  return analogValue;
}

//PID control loop for steering servo
void movesteermotor(){
  // Serial.println("moved motor");
  stepper.stop();
  stepper.disable();
  stepper.enable();

  SetpointSteer = radiansToAnalog(requested_radian_position);
  // Soft limit, check to makesure setpoint is within acceptable steering range
  // Serial.println("ReqRad        " + String(requested_radian_position) + "        SetpointSteerAfter: " + String(SetpointSteer));
  if (SetpointSteer < 452) SetpointSteer = 452;
  if (SetpointSteer > 572) SetpointSteer = 572;
  // Serial.println("SetpointSteer After: " + String(SetpointSteer));

  InputSteer = analogRead(A0); //Read steering encoder
  if ((SetpointSteer - InputSteer) < 0)
  {
    SteerPID.SetControllerDirection(REVERSE);
    dir = -1;
  }

  if ((SetpointSteer - InputSteer) > 0)
  {
    SteerPID.SetControllerDirection(DIRECT);
    dir = 1;
  }
  SteerPID.Compute();
  // Serial.println("Output with direction" + String(dir*Output));
  stepper.move(dir*OutputSteer); 

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

   // convert to doubles
   requested_radian_position = String(commands[1]).toDouble(); //cast radian command as double
   target_velocity_position = String(commands[0]).toDouble(); //cast velocity command as double
  
  }  // end of process_data

// Global variable to store last PWM value
int previous_pwm = 0;

void rampVelocity() {
  const double max_velocity_step = 0.01;  // Max delta per control cycle (~100ms)
  double diff = target_velocity_position - current_velocity_position;
  // Serial.print("TargetVel: "); Serial.print(target_velocity_position);
  if (abs(diff) > max_velocity_step) {
    current_velocity_position += (diff > 0 ? max_velocity_step : -max_velocity_step);
  } else {
    current_velocity_position = target_velocity_position;
  }
}

// Ramping function to prevent sudden PWM changes
int rampedPWM(int targetPWM, int maxStep) {
  int diff = targetPWM - previous_pwm;
  if (abs(diff) > maxStep) {
    targetPWM = previous_pwm + (diff > 0 ? maxStep : -maxStep);
  }
  previous_pwm = targetPWM;
  return targetPWM;
}

double calculateTargetPWM(double targetSpeed) {             
  const double QUADRATURE = 4.0;        // x4 decoding      // Motor to wheel gear ratio  // meters
  const double SCALING_FACTOR = 150.3;  // pulses/sec per PWM (measured at PWM=76.5)

  // Compute counts per wheel revolution
  double countsPerRev = PPR * QUADRATURE * WHEEL_GEAR_RATIO;

  // Compute wheel circumference
  double circumference = 2 * PI * WHEEL_RADIUS;

  // Convert m/s → rev/sec → counts/sec
  double requiredPulsesPerSec = (targetSpeed / circumference) * countsPerRev;

  // Convert pulses/sec → PWM
  double targetPWM = requiredPulsesPerSec / SCALING_FACTOR;

  // Clamp to max usable PWM (based on testing)
  if (targetPWM > 76.5) targetPWM = 76.5;
  if (targetPWM < 0) targetPWM = 0;

  return targetPWM;
}

void updateVelocityControl() {
  // === CONFIGURATION ===
  const int max_pwm = 125;                    // 30% of 255
  const int ramp_step = 2;                   // Max PWM change per cycle

  rampVelocity();

  // === 1. Zero-speed cutoff ===
  if (abs(current_velocity_position) < 0.01) {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    SpeedPID.SetMode(MANUAL);     // Disable PID
    OutputSpeed = 0;
    previous_pwm = 0;             // Reset ramp state
    return;
  } else {
    SpeedPID.SetMode(AUTOMATIC);  // Resume PID control
  }

  
  SetpointSpeed = current_velocity_position;
  InputSpeed = linearSpeed1;

  // === 4. Run PID Control ===
  SpeedPID.Compute();  // OutputSpeed will be updated here

  // === 5. Clamp and ramp PWM ===
  int pwmVal = (int)(OutputSpeed);
  pwmVal = constrain(pwmVal, -255, 255);
  pwmVal = rampedPWM(pwmVal, ramp_step);  // Smooth transitions

  // Only kickstart if the motor is trying to move but speed is stuck
  // if (abs(InputSpeed) < 0.05 && abs(pwmVal) > 0 && pwmVal < 20) {
  //   pwmVal = (pwmVal > 0)? 20 : -20;
  // }

  // === 6. Apply PWM to motor based on direction ===
  if (pwmVal >= 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwmVal);
  } else {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, abs(pwmVal));
  }

  // === 7. Debug Output (optional) ===
  Serial.print("TargetVel: "); Serial.print(current_velocity_position);
  Serial.print("Setpoint velocity: "); Serial.print(SetpointSpeed);
  Serial.print(" | Actual Velocity: "); Serial.print(InputSpeed);
  Serial.print(" | PWM: "); Serial.println(pwmVal);
}

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



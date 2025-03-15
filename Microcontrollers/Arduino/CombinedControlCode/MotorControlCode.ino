#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <BasicStepperDriver.h>

// Configuration namespace for constants
namespace Config {
    // Motor driver pins
    constexpr int LPWM_PIN = 5;       // Left PWM pin for motor speed
    constexpr int RPWM_PIN = 6;       // Right PWM pin for motor speed
    constexpr int L_EN_PIN = 4;       // Left enable pin for motor driver
    constexpr int R_EN_PIN = 12;      // Right enable pin (changed from 2 to avoid interrupt conflict)
    constexpr int L_IS_PIN = 7;       // Left current sense pin
    constexpr int R_IS_PIN = 8;       // Right current sense pin

    // Encoder pins
    constexpr int ENC1_A = 2;         // Encoder 1, channel A
    constexpr int ENC1_B = 3;         // Encoder 1, channel B
    constexpr int ENC2_A = 21;        // Encoder 2, channel A
    constexpr int ENC2_B = 20;        // Encoder 2, channel B

    // Stepper motor settings
    constexpr int STEPPER_DIR = 10;   // Direction pin for steering stepper
    constexpr int STEPPER_STEP = 11;  // Step pin for steering stepper
    constexpr int MOTOR_STEPS = 1600; // Steps per revolution for stepper
    constexpr int RPM = 750;          // Desired RPM for stepper
    constexpr int MICROSTEPS = 1;     // Microstepping setting

    // General settings
    constexpr int AVG_SIZE = 5;       // Size of rolling average buffer for encoders
    constexpr int SERIAL_BAUD = 115200; // Serial baud rate
    constexpr int PWM_MAX = 255;      // Maximum PWM value for motor control
    constexpr unsigned long TIMEOUT_MS = 500; // Timeout (ms) to stop motor if no command received
}

// MotorController class to encapsulate all functionality
class MotorController {
private:
    // Motor driver pin constants, set in constructor
    const int lpwmPin;
    const int rpwmPin;
    const int lEnPin;
    const int rEnPin;
    const int lIsPin;
    const int rIsPin;

    // Encoder objects for wheel velocity
    Encoder enc1;  // First encoder (left wheel)
    Encoder enc2;  // Second encoder (right wheel)

    // Rolling average buffers for smoothing encoder and steering data
    int rpm1Buffer[Config::AVG_SIZE] = {0};  // Buffer for left wheel RPM
    int rpm2Buffer[Config::AVG_SIZE] = {0};  // Buffer for right wheel RPM
    int zthBuffer[Config::AVG_SIZE] = {0};   // Buffer for steering angle
    int bufferIndex = 0;                     // Current index in circular buffer

    // State variables
    double velocity = 0.0;         // Motor velocity command (0 to 1, scaled to PWM)
    double setpoint = 512.0;       // Desired steering position (PID setpoint)
    double pidInput = 0.0;         // Current steering position (PID input)
    double pidOutput = 0.0;        // PID output for stepper control
    int averageRpm1 = 0;           // Averaged RPM for left wheel
    int averageRpm2 = 0;           // Averaged RPM for right wheel
    int averageZth = 0;            // Averaged steering angle
    unsigned long lastCommandTime = 0; // Timestamp of last serial command

    // PID controller for steering
    PID pid;  // PID object linked to input, output, and setpoint

    // Stepper motor for steering
    BasicStepperDriver stepper;  // Stepper driver object

    // Serial parsing buffer
    static constexpr int MAX_INPUT = 12; // Max length of serial command
    char inputBuffer[MAX_INPUT];         // Buffer for incoming serial data
    int inputPos = 0;                    // Current position in serial buffer

    // Updates motor PWM based on velocity
    void updateMotor() {
        // Scale velocity (0 to 1) to PWM range (0 to 255) and cast to integer
        int pwm = static_cast<int>(velocity * Config::PWM_MAX);
        // Constrain PWM to valid range (-255 to 255) to prevent overflow
        pwm = constrain(pwm, -Config::PWM_MAX, Config::PWM_MAX);
        if (pwm >= 0) {  // Forward direction
            analogWrite(rpwmPin, 0);      // Disable reverse
            analogWrite(lpwmPin, pwm);    // Set forward speed
        } else {         // Reverse direction
            analogWrite(lpwmPin, 0);      // Disable forward
            analogWrite(rpwmPin, -pwm);   // Set reverse speed (positive value)
        }
    }

    // Computes rolling average for encoder and steering data
    void computeRollingAverage(int enc1Reading, int enc2Reading, int zthReading) {
        // Convert encoder counts to RPM (assuming 2400 counts per revolution)
        rpm1Buffer[bufferIndex] = enc1Reading * 60 / 2400;
        rpm2Buffer[bufferIndex] = enc2Reading * 60 / 2400;
        zthBuffer[bufferIndex] = zthReading;  // Steering angle (raw analog value)

        // Increment buffer index, wrapping around with modulo
        bufferIndex = (bufferIndex + 1) % Config::AVG_SIZE;

        // Calculate averages
        int sum1 = 0, sum2 = 0, sumZth = 0;
        for (int i = 0; i < Config::AVG_SIZE; i++) {
            sum1 += rpm1Buffer[i];
            sum2 += rpm2Buffer[i];
            sumZth += zthBuffer[i];
        }
        averageRpm1 = sum1 / Config::AVG_SIZE;
        averageRpm2 = sum2 / Config::AVG_SIZE;
        averageZth = sumZth / Config::AVG_SIZE;
    }

    // Parses serial command (e.g., "0.5,512") into velocity and setpoint
    void parseCommand(const char* data) {
        char buffer[MAX_INPUT];
        strncpy(buffer, data, MAX_INPUT);  // Copy data to avoid modifying original
        char* token = strtok(buffer, ","); // Split by comma
        if (token) {
            velocity = atof(token);        // Convert first token to velocity (double)
            token = strtok(nullptr, ",");  // Get next token (nullptr continues parsing)
            if (token) {
                double newSetpoint = atof(token);  // Convert to setpoint
                // Validate setpoint range for steering safety
                if (newSetpoint > 450 && newSetpoint < 600) {
                    setpoint = newSetpoint;
                }
            }
        }
    }

public:
    // Constructor with initializer list for efficient setup
    MotorController() 
        : lpwmPin(Config::LPWM_PIN), rpwmPin(Config::RPWM_PIN),
          lEnPin(Config::L_EN_PIN), rEnPin(Config::R_EN_PIN),
          lIsPin(Config::L_IS_PIN), rIsPin(Config::R_IS_PIN),
          enc1(Config::ENC1_A, Config::ENC1_B),  // Initialize encoders
          enc2(Config::ENC2_A, Config::ENC2_B),
          pid(&pidInput, &pidOutput, &setpoint, 0.5, 0, 0, DIRECT), // PID with tuning params
          stepper(Config::MOTOR_STEPS, Config::STEPPER_DIR, Config::STEPPER_STEP) { // Stepper setup
        
        // Configure motor driver pins
        pinMode(lpwmPin, OUTPUT);
        pinMode(rpwmPin, OUTPUT);
        pinMode(lEnPin, OUTPUT);
        pinMode(rEnPin, OUTPUT);
        pinMode(lIsPin, OUTPUT);
        pinMode(rIsPin, OUTPUT);
        digitalWrite(lIsPin, LOW);  // Disable current sensing (if not used)
        digitalWrite(rIsPin, LOW);
        digitalWrite(lEnPin, HIGH); // Enable motor driver
        digitalWrite(rEnPin, HIGH);

        // Configure PID
        pid.SetOutputLimits(0, 100); // Limit PID output to reasonable stepper steps
        pid.SetMode(AUTOMATIC);      // Enable PID control

        // Configure stepper
        stepper.begin(Config::RPM, Config::MICROSTEPS); // Set RPM and microstepping
        stepper.enable();                                // Enable stepper motor
    }

    // Main update function, called in loop()
    void update() {
        // Process incoming serial commands
        while (Serial.available() > 0) {
            char inByte = Serial.read();
            if (inByte == '\n') {  // End of command
                inputBuffer[inputPos] = '\0';  // Null-terminate string
                parseCommand(inputBuffer);     // Parse velocity and setpoint
                inputPos = 0;                  // Reset buffer position
                lastCommandTime = millis();    // Update timestamp
            } else if (inputPos < MAX_INPUT - 1) { // Add byte if buffer not full
                inputBuffer[inputPos++] = inByte;
            }
        }

        // Stop motor if no command received within timeout
        if (millis() - lastCommandTime > Config::TIMEOUT_MS) {
            velocity = 0.0;  // Reset velocity
            updateMotor();   // Apply zero speed
        } else {
            updateMotor();   // Apply current velocity
        }

        // Read and average encoder and steering data
        int enc1Reading = enc1.read();
        int enc2Reading = enc2.read();
        int zthReading = analogRead(A0);  // Steering position from analog pin
        computeRollingAverage(enc1Reading, enc2Reading, zthReading);
        enc1.write(0);  // Reset encoders
        enc2.write(0);

        // Update steering with PID
        pidInput = zthReading;            // Set current position as PID input
        pid.Compute();                    // Calculate PID output
        int direction = (setpoint > pidInput) ? 1 : -1; // Determine direction
        stepper.move(direction * pidOutput); // Move stepper based on PID

        // Send telemetry over serial
        Serial.print(averageRpm1);    // Left wheel RPM
        Serial.print(",");
        Serial.print(averageRpm2);    // Right wheel RPM
        Serial.print(",");
        Serial.println(averageZth);   // Steering angle
    }
};

// Global instance of MotorController
MotorController motor;

void setup() {
    // Initialize serial communication
    Serial.begin(Config::SERIAL_BAUD);
}

void loop() {
    // Continuously update motor controller
    motor.update();
    delay(200); // Temporary delay for demo; replace with timers in production
}

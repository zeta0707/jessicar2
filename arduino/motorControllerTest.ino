/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 * motor_controller_diff_drive_2.ino.
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

#define FIXV_ROTATE 0

#define JGA25370 0
#define JGB520 1
#define MOTORTYPE JGB520

#define USE_PID 1
#if USE_PID == 1
double trackAdjustValue = 0.0;
double trackSetpoint = 0.0;
double trackError = 0.0;

double Kp = 12.0;  //Determines how aggressively the PID reacts to the current amount of error (Proportional)
double Ki = 2.0;   //Determines how aggressively the PID reacts to error over time (Integral)
double Kd = 1.0;   //Determines how aggressively the PID reacts to the change in error (Derivative)

PID trackPID(&trackError, &trackAdjustValue, &trackSetpoint, Kp, Ki, Kd, DIRECT);
#endif

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
#define ENC_IN_RIGHT_B 4

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;

// Time interval for measurements in milliseconds
const int interval = 500;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////

// Motor A connections, Left
const int enA = 10;
const int in1 = 12;
const int in2 = 13;
// Motor B connections, Right
const int enB = 9;
const int in3 = 7;
const int in4 = 6;

// TB6612 Chip control pins
#define TB6612_STBY 8

// Number of ticks per wheel revolution. We won't use this in this code.

// Wheel radius in meters. We won't use this in this code.
#define WHEEL_RADIUS (0.033)
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2)
// Distance from center of the left tire to the center of the right tire in m. We won't use this in this code.
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
#if MOTORTYPE == JGB520
#define TICKS_PER_REVOLUTION (1720.0)
#else
#define TICKS_PER_REVOLUTION (102.0)
#endif

#define TICKS_PER_METER (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS))
#define WHEEL_BASE (0.140)

#if MOTORTYPE == JGB520
#define K_P 1160.0
#define K_b 10.0
#define PWM_MIN 80.0   // about 0.07 m/s
#define PWM_MAX 200.0  // about 0.14 m/s
#define K_bias 0.0     // left is much slow
#else
#define K_P 125
#define K_b 35
#define PWM_MIN 40
#define PWM_MAX 80
#endif

#define PWM_TURN (PWM_MIN)
// How much the PWM value can change each cycle
#define PWM_INCREMENT 1
// Correction multiplier for drift. Chosen through experimentation.
#define DRIFT_MULTIPLIER 80.0

// Set linear velocity and PWM variable values for each wheel
float velLeftWheel = 0.0;
float velRightWheel = 0.0;
float pwmLeftReq = 0.0;
float pwmRightReq = 0.0;

// Record the time that the last velocity command was received
float lastCmdVelReceived = 0.0;

/////////////////////// Tick Data Publishing Functions ////////////////////////

// Increment the number of ticks
void right_wheel_tick() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false;  // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    } else {
      right_wheel_tick_count.data++;
    }
  } else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    } else {
      right_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void left_wheel_tick() {

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == HIGH) {
    Direction_left = true;  // Reverse
  } else {
    Direction_left = false;  // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    } else {
      left_wheel_tick_count.data++;
    }
  } else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    } else {
      left_wheel_tick_count.data--;
    }
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////
// Calculate the left wheel linear velocity in m/s every time a
// tick count message is rpublished on the /left_ticks topic.
void calc_vel_left_wheel() {

  // Previous timestamp
  static float prevTime = 0.0;

  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 30000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velLeftWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;

  // Update the timestamp
  prevTime = (millis() / 1000.0);
  Serial.print("L:");  Serial.print(velLeftWheel, 3);
  Serial.print(",");  Serial.print(numOfTicks);
}

// Calculate the right wheel linear velocity in m/s every time a
// tick count message is published on the /right_ticks topic.
void calc_vel_right_wheel() {

  // Previous timestamp
  static float prevTime = 0.0;

  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 30000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velRightWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
  prevRightCount = right_wheel_tick_count.data;

  prevTime = (millis() / 1000.0);

  Serial.print("R:"); Serial.print(velRightWheel, 3);
  Serial.print(","); Serial.print(numOfTicks);
}

#if FIXV_ROTATE == 1
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(float linearx, float angularz) {

  if (linearx >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmLeftReq = K_P * linearx + K_b + K_bias;
    pwmRightReq = K_P * linearx + K_b;
  } else {
    pwmLeftReq = K_P * linearx - K_b - K_bias;
    pwmRightReq = K_P * linearx - K_b;
  }

  // Turn left
  if (angularz > 0.4) {
    pwmLeftReq = -PWM_TURN;
    pwmRightReq = PWM_TURN;
    goStraight = false;
  }
  // Turn right
  else if (angularz < -0.4) {
    pwmLeftReq = PWM_TURN;
    pwmRightReq = -PWM_TURN;
    goStraight = false;
  } else {
    goStraight = true;
  }

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }
#else
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(float linearx, float angularz) {

  float vLeft, vRight;

  vLeft = (2.0 * linearx - angularz * WHEEL_BASE) / 2.0;
  vRight = (2.0 * linearx + angularz * WHEEL_BASE) / 2.0;

  if (vLeft >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmLeftReq = K_P * vLeft + K_b;
  } else {
    pwmLeftReq = K_P * vLeft - K_b;
  }
  if (vRight >= 0.0) {
    // Calculate the PWM value given the desired velocity
    pwmRightReq = K_P * vRight + K_b;
  } else {
    pwmRightReq = K_P * vRight - K_b;
  }

  if (angularz == 0.0)
    goStraight = true;

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }
#endif

  Serial.print("cPWM:");
  Serial.print(pwmLeftReq);
  Serial.print(":");
  Serial.println(pwmRightReq);
}

void set_pwm_values() {

  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;

  //go straight, make PID for velocity loop
  if (goStraight == true) {
    trackError = velRightWheel - velLeftWheel;
    trackError *= 10;

    if (trackPID.Compute())  //true if PID has triggered
      pwmRightReq += trackAdjustValue;

    Serial.print("COM:");
    Serial.print(trackError, 3);
    Serial.print(":");
    Serial.println(trackAdjustValue);
  }

  // Set the direction of the motors
  if (pwmLeftReq > 0) {  // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {  // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwmRightReq > 0) {  // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (pwmRightReq < 0) {  // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else if (pwmRightReq == 0 && pwmRightOut == 0) {  // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  } else {  // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  Serial.print("sReq:"); Serial.print(pwmLeftReq);
  Serial.print(":"); Serial.println(pwmRightReq);

  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  } else {
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  } else {
  }

  // Conditional operator to limit PWM output at the maximum
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;

  Serial.print("sOut:"); Serial.print(pwmLeftOut);
  Serial.print(":"); Serial.println(pwmRightOut);
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  // Set the PWM value on the pins
  analogWrite(enA, pwmLeftOut);
  analogWrite(enB, pwmRightOut);
}

void setup() {

  Serial.begin(115200);
  Serial.println("Motor vel_cmd Test");

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor control pins are outputs
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

  pinMode(TB6612_STBY, OUTPUT);  // TB6612 Enable control pin configuration output
  digitalWrite(TB6612_STBY, HIGH);

  // Set the motor speed
  analogWrite(enA, 0);
  analogWrite(enB, 0);

#if USE_PID == 1
  trackAdjustValue = 0;
  trackSetpoint = 0;
  trackError = 0;

  trackPID.SetMode(AUTOMATIC);
  trackPID.SetSampleTime(200);
  trackPID.SetOutputLimits(-20, 20);
#endif
}

void loop() {
  float linearx, angularz;

  // Record the time
  currentMillis = millis();

  //input like "0.15 -0.5"
  if (Serial.available() > 0) {
    linearx = Serial.parseFloat(SKIP_ALL, '\n');
    angularz = Serial.parseFloat(SKIP_ALL, '\n');

    //prints the received float number
    Serial.println(linearx);
    Serial.println(angularz);
    calc_pwm_values(linearx, angularz);
  }

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();
  }

  set_pwm_values();
}

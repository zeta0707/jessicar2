/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

#if (DEBUG == 1)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTF(x, y) Serial.print(x, y)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)
#endif

////////////////// Tick Data Publishing Variables and Constants ///////////////
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
#define ENC_IN_RIGHT_B 4

////////////////// Motor Controller Variables and Constants ///////////////////
// Motor A connections
#define TB6612_PWMA 9
#define TB6612_AIN1 7
#define TB6612_AIN2 6
// Motor B connections
#define TB6612_PWMB 10
#define TB6612_BIN1 12
#define TB6612_BIN2 13
// TB6612 Chip control pins
#define TB6612_STBY 8

// Minumum and maximum values for 16-bit integers
// Range of 65,535
#define ENCODER_MIN -32768
#define ENCODER_MAX 32767

// RGB Color lamp control pin
#define RLED A0
#define GLED A1
#define BLED A2

/***********************Enumeration variable***********************/
enum COLOR {
  RED = 0,  // red
  GREEN,    // green
  BLUE,     // blue
  YELLOW,   // yellow
  PURPLE,   // purple
  CYAN,     // cyan
  WHITE,    // white
  ALL_OFF   // off(black)
};

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Time interval for measurements in milliseconds
#define INTERVAL 30
long previousMillis = 0;
long currentMillis = 0;

// How much the PWM value can change each cycle
#define PWM_INCREMENT 1
// Number of ticks per wheel revolution. We won't use this in this code.
#define TICKS_PER_REVOLUTION 620
// Wheel radius in meters
#define WHEEL_RADIUS 0.033
// Distance from center of the left tire to the center of the right tire in m
#define WHEEL_BASE 0.17
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
#define TICKS_PER_METER 3100  // Originally 2880

// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
#define K_P 278
// Y-intercept for the PWM-Linear Velocity relationship for the robot
#define K_b 52
// Correction multiplier for drift. Chosen through experimentation.
#define DRIFT_MULTIPLIER 120
// Turning PWM output (0 = min, 255 = max for PWM values)
#define PWM_TURN 80
// Set maximum and minimum limits for the PWM values
#define PWM_MIN 80   // about 0.1 m/s
#define PWM_MAX 100  // about 0.172 m/s

// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

bool blinkState = false;

// Increment the number of ticks
void right_wheel_tick() {

  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == HIGH) {
    Direction_right = false;  // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == ENCODER_MAX) {
      right_wheel_tick_count.data = ENCODER_MIN;
    } else {
      right_wheel_tick_count.data++;
    }
  } else {
    if (right_wheel_tick_count.data == ENCODER_MIN) {
      right_wheel_tick_count.data = ENCODER_MAX;
    } else {
      right_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void left_wheel_tick() {

  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true;  // Reverse
  } else {
    Direction_left = false;  // Forward
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == ENCODER_MAX) {
      left_wheel_tick_count.data = ENCODER_MIN;
    } else {
      left_wheel_tick_count.data++;
    }
  } else {
    if (left_wheel_tick_count.data == ENCODER_MIN) {
      left_wheel_tick_count.data = ENCODER_MAX;
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
  static double prevTime = 0;
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;
  // Update the timestamp
  prevTime = (millis() / 1000);
}

// Calculate the right wheel linear velocity in m/s every time a
// tick count message is published on the /right_ticks topic.
void calc_vel_right_wheel() {
  // Previous timestamp
  static double prevTime = 0;
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velRightWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);
  prevRightCount = right_wheel_tick_count.data;
  prevTime = (millis() / 1000);
}

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {

  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis() / 1000);
  // Calculate the PWM value given the desired velocity
  pwmLeftReq = K_P * cmdVel.linear.x + K_b;
  pwmRightReq = K_P * cmdVel.linear.x + K_b;

  // Check if we need to turn
  if (cmdVel.angular.z != 0.0) {
    // Turn left
    if (cmdVel.angular.z > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
    // Turn right
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  // Go straight
  else {
    // Remove any differences in wheel velocities
    // to make sure the robot goes straight
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel;
    double avgDifference = (prevDiff + prevPrevDiff + currDifference) / 3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;

    // Correct PWM values of both wheels to make the vehicle go straight
    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }

  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }
}

void set_pwm_values() {

  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) || (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  // Set the direction of the motors
  if (pwmLeftReq > 0) {  // Left wheel forward
    digitalWrite(TB6612_AIN1, HIGH);
    digitalWrite(TB6612_AIN2, LOW);
  } else if (pwmLeftReq < 0) {  // Left wheel reverse
    digitalWrite(TB6612_AIN1, LOW);
    digitalWrite(TB6612_AIN2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  // Left wheel stop
    digitalWrite(TB6612_AIN1, LOW);
    digitalWrite(TB6612_AIN2, LOW);
  } else {  // Left wheel stop
    digitalWrite(TB6612_AIN1, LOW);
    digitalWrite(TB6612_AIN2, LOW);
  }

  if (pwmRightReq > 0) {  // Right wheel forward
    digitalWrite(TB6612_BIN1, HIGH);
    digitalWrite(TB6612_BIN2, LOW);
  } else if (pwmRightReq < 0) {  // Right wheel reverse
    digitalWrite(TB6612_BIN1, LOW);
    digitalWrite(TB6612_BIN2, HIGH);
  } else if (pwmRightReq == 0 && pwmRightOut == 0) {  // Right wheel stop
    digitalWrite(TB6612_BIN1, LOW);
    digitalWrite(TB6612_BIN2, LOW);
  } else {  // Right wheel stop
    digitalWrite(TB6612_BIN1, LOW);
    digitalWrite(TB6612_BIN2, LOW);
  }

  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }

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

  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  // Set the PWM value on the pins
  analogWrite(TB6612_PWMA, pwmLeftOut);
  analogWrite(TB6612_PWMB, pwmRightOut);
}

void RGB(enum COLOR color) {
  switch (color) {
    case RED:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, HIGH);
      break;
    case GREEN:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, HIGH);
      break;
    case BLUE:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, LOW);
      break;
    case YELLOW:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, HIGH);
      break;
    case PURPLE:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, LOW);
      break;
    case CYAN:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, LOW);
      break;
    case WHITE:
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, LOW);
      digitalWrite(BLED, LOW);
      break;
    default:
      digitalWrite(RLED, HIGH);
      digitalWrite(GLED, HIGH);
      digitalWrite(BLED, HIGH);
      break;
  }
}

void ledcb(const std_msgs::Int16& msg) {
  RGB(msg.data);  // set the pin state to the message data
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);
ros::Subscriber<std_msgs::Int16> subLed("rgbled", &ledcb);

void setup() {

#if (DEBUG == 1)
  Serial.begin(115200);
  while (!Serial)
    ;
    // wait for Leonardo enumeration, others continue immediately
#endif

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  pinMode(TB6612_STBY, OUTPUT);  // TB6612 Enable control pin configuration output
  digitalWrite(TB6612_STBY, HIGH);

  // Motor control pins are outputs
  pinMode(TB6612_PWMA, OUTPUT);
  pinMode(TB6612_PWMB, OUTPUT);
  pinMode(TB6612_AIN1, OUTPUT);
  pinMode(TB6612_AIN2, OUTPUT);
  pinMode(TB6612_BIN1, OUTPUT);
  pinMode(TB6612_BIN2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(TB6612_AIN1, LOW);
  digitalWrite(TB6612_AIN2, LOW);
  digitalWrite(TB6612_BIN1, LOW);
  digitalWrite(TB6612_BIN2, LOW);

  // Set the motor speed
  analogWrite(TB6612_PWMA, 0);
  analogWrite(TB6612_PWMB, 0);

  pinMode(RLED, OUTPUT);  // RGB color lights red control pin configuration output
  pinMode(GLED, OUTPUT);  // RGB color light green control pin configuration output
  pinMode(BLED, OUTPUT);  // RGB color light blue control pin configuration output
  RGB(100);               // RGB LED all off

#if (DEBUG == 1)
  while (Serial.available() && Serial.read())
    ;
  // empty buffer
  Serial.end();
#endif
  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  nh.subscribe(subLed);
}

void loop() {

  nh.spinOnce();

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > INTERVAL) {

    previousMillis = currentMillis;

    // Publish tick counts to topics
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);

    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}
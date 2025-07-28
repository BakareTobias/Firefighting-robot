#include <Servo.h>

// Motor driver pins
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define ENA 10
#define ENB 5

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 11

// Servo motor pin
#define SERVO_PIN 2

// Flame sensor pins
#define FLAME_SENSOR_LEFT 46
#define FLAME_SENSOR_FRONT  50
#define FLAME_SENSOR_RIGHT 48

// Define MQ5 gas pin
int MQ5_pin = A0;

//Define Relay pin
#define  relay 32

// Define buzzer pin
#define buzzer_pin 13

Servo servoMotor; // Create servo objectz

int speed = 140; // Speed range: 0 to 255

//define turn duration(seconds)
int turn_duration = 1;

int turn_speed = 75;

int gas_threshold = 500;

int fire_turn_speed = 75;

int fire_turn_duration = 1;

int fire_move_forward_speed= 85;

void setup() {
  Serial.begin(9600);  // For debugging

  // Motor setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  


  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Flame sensor setup 
  pinMode(FLAME_SENSOR_FRONT, INPUT);
  pinMode(FLAME_SENSOR_LEFT,  INPUT);
  pinMode(FLAME_SENSOR_RIGHT, INPUT);

  // Servo setup
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90); // Center position

  //buzzer setup 
  pinMode(buzzer_pin, OUTPUT);

  //relay setup 
  pinMode(relay, OUTPUT);


}

void loop() {
  digitalWrite(relay, HIGH); //keep pump off by default
  analogWrite(buzzer_pin, 0);//kee[ buzzer off by default


  checkFlameSensors();  // Check for fire 
  checkGas();           // Check for gas/smoke
  moveForward(speed);



  int distance = getDistance();
  Serial.print("Forward Distance: ");
  Serial.println(distance);

 
  if (distance < 20) { // Obstacle detected
    stopMotors();
    delay(25);

    int leftDistance = scanLeft();
    int rightDistance = scanRight();

    Serial.print("Left: ");
    Serial.print(leftDistance);
    Serial.print(" cm | Right: ");
    Serial.print(rightDistance);
    Serial.println(" cm");

    if (leftDistance > rightDistance) {
      turnLeft(turn_speed, turn_duration);
      Serial.println("Turning LEFT");
      delay(500);


    } else {    
      turnRight(turn_speed, turn_duration);
      Serial.println("Turning RIGHT");
      delay(500);

    } 
  }




}

//HELPER FUNCTIONS

// Move forward
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Move backward
void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Turn left
void turnLeft(int speed, float duration) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  delay((int)(duration * 1000));  // Ensure integer delay
  stopMotors();
}

// Turn right
void turnRight(int speed, float duration) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  delay((int)(duration * 1000));  // Ensure integer delay
  stopMotors();
}

// Stop motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(300);
  
}

// Measure distance using ultrasonic sensor
int getDistance(){

  digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(4); // wait for 4 ms to avoid
    // collision in serial monitor

    digitalWrite( TRIG_PIN,HIGH); // turn on the Trigger to generate pulse
    delayMicroseconds(10); // keep the trigger "ON" for 10 ms to generate pulse for 10 ms.

    digitalWrite(TRIG_PIN, LOW); // Turn off the pulse trigger to stop pulse generation

    // If pulse reached the receiver ECHO_PIN
    // become high Then pulseIn() returns the
    // time taken by the pulse to reach the
    // receiver

    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.0344 / 2; // Expression to calculate
                                 // distance using time

    return distance;

}

// Scan left
int scanLeft() {
  servoMotor.write(180); // Rotate servo left
  delay(500);
  int distance = getDistance();
  servoMotor.write(90);  // Reset to center
  delay(500);
  return distance;
}

// Scan right
int scanRight() {
  servoMotor.write(0);  // Rotate servo right
  delay(500);
  int distance = getDistance();
  servoMotor.write(90);  // Reset to center
  delay(500);
  return distance;
}

//Check for fire
void checkFlameSensors() {
    int frontFlame = digitalRead(FLAME_SENSOR_FRONT);
    int leftFlame = digitalRead(FLAME_SENSOR_LEFT);
    int rightFlame = digitalRead(FLAME_SENSOR_RIGHT);

    if (leftFlame == LOW) { 
        Serial.println("🔥 Fire detected on the LEFT! Turning left...");
        raiseAlarm();
        //turnLeft(fire_turn_speed, fire_turn_duration); // Small left turn
        //ACTIVATE PUMP
        checkFlameSensors();  // Check for fire 

    }

    if (rightFlame == LOW) {
        Serial.println("🔥 Fire detected on the RIGHT! Turning right...");
        raiseAlarm();
        //turnRight(fire_turn_speed, fire_turn_duration); // Small right turn
        //ACTIVATE PUMP
        checkFlameSensors();  // Check for fire 

    }

    if (frontFlame == LOW) {
        stopMotors();
        Serial.println("🔥 Fire detected at the FRONT! Moving closer...");
        raiseAlarm();
        //moveForward(fire_move_forward_speed);
        //delay(1000);  // Move forward for 1 second (adjust as needed)
        //stopMotors();
        //ACTIVATE PUMP
        digitalWrite(relay, LOW);  // turn the LED on (HIGH is the voltage level)
        delay(2000);                      // wait for a second
        digitalWrite(relay, HIGH);   // turn the LED off by making the voltage LOW
        checkFlameSensors();  // Check for fire 

        
    }
}

//check MQ5 for gas leaks
void checkGas() {

  float gas_val = analogRead(MQ5_pin);
  //Serial.println(gas_val);
  if (gas_val >= gas_threshold) {
    raiseAlarm();
    Serial.print("Gas Detected: ");
    Serial.println(gas_val);
  }
}

void raiseAlarm() {
  analogWrite(buzzer_pin, 100);
  delay(1000);
  analogWrite(buzzer_pin, 0);

}

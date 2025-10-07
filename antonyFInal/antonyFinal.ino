#include <Wire.h>
#include <ArduinoBLE.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "Arduino_LED_Matrix.h"
#include "mp3tf16p.h"  // MP3 player library

// Speaker
const int maxVolume = 30;
MP3Player mp3(A3, A2);  // RX, TX

// LED Matrix
ArduinoLEDMatrix matrix;
const uint32_t animation[][4] = {
  { 0x600603f, 0xc3fc1f80, 0xf0060000, 66 },  // Forward
  { 0x10030070, 0xfff7003, 0x100000, 66 },    // Left
  { 0x800c00, 0xefff00e0, 0xc008000, 66 },    // Right
  { 0x2081100a, 0x400a01, 0x10208000, 66 },   // Stop
  { 0x400e01f, 0x3f87fc0, 0x40040040, 66 }    // Backward
};

// I2C LCD
hd44780_I2Cexp lcd;

// Sensors
#define TRIG_PIN 8
#define ECHO_PIN 9
#define LEFT_IR_SENSOR 6
#define RIGHT_IR_SENSOR 7
#define HALL_SENSOR_PIN 2
#define BUZZER 4

// Motor Driver
#define ENABLE_LEFT 3
#define ENABLE_RIGHT 5
#define LEFT_MOTOR_PIN1 10
#define LEFT_MOTOR_PIN2 11
#define RIGHT_MOTOR_PIN1 12
#define RIGHT_MOTOR_PIN2 13

// Constants
#define NUM_SAMPLES 5
#define MAGNET_COUNT 1
#define WHEEL_DIAMETER 5.5  // cm
int motorSpeed = 195;
int turningSpeed = 215;

// BLE Service
BLEService obstacleService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic obstacleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify, 20);

// Global Variables
bool isAutonomousMode = true;
bool bleConnected = false;
float distance;
float distanceSamples[NUM_SAMPLES] = {100, 100, 100, 100, 100};
int sampleIndex = 0;
volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0.0;
float speed_cms = 0.0;
String lastObstacleState = "Clear";
unsigned long lastBleUpdate = 0;

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial1.begin(38400);  // HC-05
  Serial.begin(115200);

  //MP3
  mp3.initialize();

  mp3.playTrackNumber(1, maxVolume, false); // Startup sound
  delay(2000);
  mp3.playTrackNumber(2, maxVolume, false); // line following sound

  // Initialize I2C
  Wire.begin();
  
  // Initialize pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Starting...");
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    lcd.clear();
    lcd.print("BLE Init Fail");
    while(1);
  }
  
  // BLE Configuration
  BLE.setLocalName("ObstacleBot");
  BLE.setDeviceName("ObstacleBot");
  BLE.setAdvertisedService(obstacleService);
  obstacleService.addCharacteristic(obstacleCharacteristic);
  BLE.addService(obstacleService);
  
  // BLE Event Handlers
  BLE.setEventHandler(BLEConnected, onBLEConnect);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnect);
  
  // Start advertising
  BLE.advertise();
  Serial.println("BLE Advertising Started");
  lcd.clear();
  lcd.print("BLE Ready");
  
  // Initialize LED Matrix
  matrix.begin();
  
  // Hall Effect Sensor Interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), countPulse, FALLING);
}

void loop() {
  // Handle BLE connection
  BLEDevice central = BLE.central();
  
  // Check obstacle status
  checkObstacle();
  
  // Mode switching
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("MODE:")) {
      isAutonomousMode = (data.substring(5) == "AUTO");
      stopMotors();
      Serial.println(isAutonomousMode ? "AUTO MODE" : "REMOTE MODE");
      lcd.clear();
      lcd.print(isAutonomousMode ? "AUTO MODE" : "REMOTE MODE");
      // Play mode change sound
      if (isAutonomousMode) {
        mp3.playTrackNumber(2, maxVolume, false);  // AUTO mode sound
      } else {
        mp3.playTrackNumber(3, maxVolume, false);  // REMOTE mode sound
      }
      return;
    }

    if (!isAutonomousMode) {
      processRemoteCommand(data);
    }
  }

  if (isAutonomousMode) {
    autonomousMode();
  }
  
  // Update BLE status
  updateBLE();
  
  // Update RPM every second
  updateRPM();
}

void autonomousMode() {
  if (distance > 20) {
    if (digitalRead(LEFT_IR_SENSOR)) {
      tankTurnRight();
      matrix.loadFrame(animation[2]);
    } else if (digitalRead(RIGHT_IR_SENSOR)) {
      tankTurnLeft();
      matrix.loadFrame(animation[1]);
    } else {
      moveForward();
      matrix.loadFrame(animation[0]);
    }
  } else {
    stopMotors();
    matrix.loadFrame(animation[3]);
  }
}

void processRemoteCommand(String data) {
  int xIndex = data.indexOf('X');
  int yIndex = data.indexOf('Y');
  if (xIndex != -1 && yIndex != -1) {
    int xValue = data.substring(xIndex + 1, yIndex).toInt();
    int yValue = data.substring(yIndex + 1).toInt();
    
    if (xValue > 700) {
      moveBackward();
      matrix.loadFrame(animation[4]);
    } else if (xValue < 300) {
      moveForward();
      matrix.loadFrame(animation[0]);
    } else if (yValue > 700) {
      turnLeft();
      matrix.loadFrame(animation[1]);
    } else if (yValue < 300) {
      turnRight();
      matrix.loadFrame(animation[2]);
    } else {
      stopMotors();
      matrix.loadFrame(animation[3]);
    }
  }
}

void checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  distance = pulseIn(ECHO_PIN, HIGH, 30000) * 0.0343 / 2;

  // Moving average filter
  distanceSamples[sampleIndex] = distance;
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
  float avgDistance = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) avgDistance += distanceSamples[i];
  avgDistance /= NUM_SAMPLES;
  distance = avgDistance;

  // Update LCD and buzzer
  lcd.setCursor(0, 1);
  if (avgDistance <= 10) {
    stopMotors();
    digitalWrite(BUZZER, HIGH);
    lcd.print("Obstacle! ");
  } else {
    digitalWrite(BUZZER, LOW);
    lcd.print("Clear     ");
  }
}

void onBLEConnect(BLEDevice central) {
  bleConnected = true;
  Serial.print("Connected to central: ");
  Serial.println(central.address());
  lcd.setCursor(0, 1);
  lcd.print("BLE Connected ");
}

void onBLEDisconnect(BLEDevice central) {
  bleConnected = false;
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  lcd.setCursor(0, 1);
  lcd.print("BLE Disconnected");
  BLE.advertise(); // Restart advertising
}

void updateBLE() {
  static unsigned long lastUpdate = 0;
  unsigned long currentMillis = millis();
  
  // Only update if state changed or every 2 seconds (for reliability)
  if ((distance < 10 && lastObstacleState != "Obstacle!") || 
      (distance >= 10 && lastObstacleState != "Clear") ||
      (currentMillis - lastUpdate >= 2000)) {
    
    String obstacleState = (distance < 10) ? "Obstacle!" : "Clear";
    
    // Only send if state actually changed
    if (obstacleState != lastObstacleState) {
      obstacleCharacteristic.writeValue(obstacleState);
      lastObstacleState = obstacleState;
      lastUpdate = currentMillis;
      
      Serial.print("BLE Update: ");
      Serial.println(obstacleState);
    }
  }
}

void updateRPM() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) {
    lastTime = currentTime;
    
    noInterrupts();
    rpm = (pulseCount * 60.0) / MAGNET_COUNT;
    float wheelCircumference = 3.1416 * WHEEL_DIAMETER;
    speed_cms = (wheelCircumference * rpm) / 60.0;
    pulseCount = 0;
    interrupts();

    lcd.setCursor(0, 0);
    lcd.print("Speed:");
    lcd.print(speed_cms);
    lcd.print("cm/s ");
  }
}

// Motor control functions (unchanged from your original)
void moveForward() {
  analogWrite(ENABLE_LEFT, motorSpeed);
  analogWrite(ENABLE_RIGHT, motorSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void moveBackward() {
  analogWrite(ENABLE_LEFT, motorSpeed);
  analogWrite(ENABLE_RIGHT, motorSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void turnLeft() {
  analogWrite(ENABLE_LEFT, turningSpeed);
  analogWrite(ENABLE_RIGHT, turningSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void turnRight() {
  analogWrite(ENABLE_LEFT, turningSpeed);
  analogWrite(ENABLE_RIGHT, turningSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void tankTurnLeft() {
  analogWrite(ENABLE_LEFT, turningSpeed);
  analogWrite(ENABLE_RIGHT, turningSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void tankTurnRight() {
  analogWrite(ENABLE_LEFT, turningSpeed);
  analogWrite(ENABLE_RIGHT, turningSpeed);
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  analogWrite(ENABLE_LEFT, 0);
  analogWrite(ENABLE_RIGHT, 0);
}

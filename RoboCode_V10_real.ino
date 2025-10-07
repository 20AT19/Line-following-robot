#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <ArduinoBLE.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "Arduino_LED_Matrix.h"  // Include the LED Matrix library

// LED Matrix
ArduinoLEDMatrix matrix;

// Define the animation frames for forward, left, right, and stop
const uint32_t animation[][4] = {
  { 0x600603f, 0xc3fc1f80, 0xf0060000, 66 },  // Forward
  { 0x800c00, 0xefff00e0, 0xc008000, 66 },   // Left
  { 0x10030070, 0xfff7003, 0x100000, 66 },   // Right
  { 0x2081100a, 0x400a01, 0x10208000, 66 }   // Stop
};

// OLED Display Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C LCD Setup
hd44780_I2Cexp lcd;

// Ultrasonic Sensor
#define TRIG_PIN 8
#define ECHO_PIN 9
float distance;

// Buzzer
#define BUZZER 4  // Changed from pin 2 to pin 4

// IR Sensors
#define LEFT_IR_SENSOR 6
#define RIGHT_IR_SENSOR 7

// Motor Driver (L293D) Pins
#define ENABLE_LEFT 3
#define ENABLE_RIGHT 5
#define INPUT1 10 // Left Motor Forward
#define INPUT2 11 // Left Motor Backward
#define INPUT3 12 // Right Motor Forward
#define INPUT4 13 // Right Motor Backward

// Motor Speed
int motorSpeed = 195; 
int turningSpeed = 215;

// Moving Average Filter for Ultrasonic Sensor
#define NUM_SAMPLES 5
float distanceSamples[NUM_SAMPLES] = {100, 100, 100, 100, 100}; // Initialize buffer
int sampleIndex = 0;

// Bluetooth (BLE) Setup
BLEService obstacleService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic obstacleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify, 20);
String lastObstacleState = "No Obstacle";

// Hall Effect Sensor
#define HALL_SENSOR_PIN 2
#define MAGNET_COUNT 1
#define WHEEL_DIAMETER 5.5  // Wheel diameter in cm
volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0.0;
float speed_cms = 0.0;

void countPulse() {
    pulseCount++;  // Increment when the magnet is detected
}

void setup() {
  Serial.begin(115200);
  
  // Pin Modes
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  pinMode(INPUT3, OUTPUT);
  pinMode(INPUT4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("Obstacle Detection");
  BLE.setAdvertisedService(obstacleService);
  obstacleService.addCharacteristic(obstacleCharacteristic);
  BLE.addService(obstacleService);
  BLE.advertise();

  Serial.println("BLE Ready!");

  // Initialize LED Matrix
  matrix.begin();

  // Attach interrupt for hall effect sensor
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), countPulse, FALLING);
}

void loop() {
  checkObstacle();  // Check for obstacles
  
  // Run movement logic **regardless of BLE connection**
  if (distance > 20) {
    moveForward();
    matrix.loadFrame(animation[0]);  // Display forward animation
  } 
  else {
    stopMotors();
    matrix.loadFrame(animation[3]);  // Display stop animation
  }

  while (digitalRead(LEFT_IR_SENSOR) == 1 && distance > 20) {
    tankTurnRight();
    matrix.loadFrame(animation[2]);  // Display right animation
  }

  while (digitalRead(RIGHT_IR_SENSOR) == 1 && distance > 20) {
    tankTurnLeft();
    matrix.loadFrame(animation[1]);  // Display left animation
  }

  // Handle BLE without stopping movement
  BLEDevice central = BLE.central();
  if (central) { 
    handleBLE(distance);
  }

  // Calculate RPM and speed every second
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) {  // Every 1 second
    lastTime = currentTime;

    // Calculate RPM
    noInterrupts();
    rpm = (pulseCount * 60.0) / MAGNET_COUNT;
    float wheelCircumference = 3.1416 * WHEEL_DIAMETER;  // Circumference = π * diameter
    speed_cms = (wheelCircumference * rpm) / 60.0;  // Speed in cm/s
    pulseCount = 0;  // Reset pulse count
    interrupts();

    // Update LCD with RPM and speed
    updateLCD(rpm, speed_cms);
  }
}

// **Obstacle Detection with Moving Average Filter**
void checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  distance = pulseIn(ECHO_PIN, HIGH, 30000) * 0.0343 / 2; // Convert to cm

  // Moving Average Filter
  distanceSamples[sampleIndex] = distance;
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
  
  float avgDistance = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) avgDistance += distanceSamples[i];
  avgDistance /= NUM_SAMPLES;

  Serial.print("Distance: ");
  Serial.println(avgDistance);

  // Update OLED Display
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Distance: ");
  display.print(avgDistance);
  display.println(" cm");
  display.display();

  // Buzzer & LCD Alert
  lcd.setCursor(0, 1);
  if (avgDistance > 12) {
    digitalWrite(BUZZER, LOW);
    lcd.print("Dist: ");
    lcd.print(avgDistance);
    lcd.print(" cm    ");
  } else if (avgDistance <= 10) {
    stopMotors();
    digitalWrite(BUZZER, HIGH);
    lcd.print("Obstacle!   ");
    lcd.print(avgDistance);
    lcd.print(" cm    ");
  }
}

// **Bluetooth Handling Without Blocking Other Tasks**
void handleBLE(float distance) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 500) { // Send BLE data every 500ms
    previousMillis = currentMillis;

    String obstacleState = (distance < 10) ? "Obstacle Detected!" : "No Obstacle";
    if (obstacleState != lastObstacleState) {
      obstacleCharacteristic.setValue(obstacleState.c_str());
      lastObstacleState = obstacleState;
    }
  }
}

// **LCD Update**
void updateLCD(float rpm, float speed_cms) {
  lcd.setCursor(0, 0);
  lcd.print("RPM: ");
  lcd.print(rpm);
  lcd.print("    ");
  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  lcd.print(speed_cms);
  lcd.print(" cm/s");
}

// **Motor Functions**
void moveForward() {
  analogWrite(ENABLE_LEFT, motorSpeed);
  analogWrite(ENABLE_RIGHT, motorSpeed);
  digitalWrite(INPUT1, HIGH);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, HIGH);
  digitalWrite(INPUT4, LOW);
}

void stopMotors() {
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, LOW);
  digitalWrite(INPUT4, LOW);
  analogWrite(ENABLE_LEFT, 0);
  analogWrite(ENABLE_RIGHT, 0);
}

void tankTurnLeft() {
  analogWrite(ENABLE_LEFT, turningSpeed);
  analogWrite(ENABLE_RIGHT, turningSpeed);
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, HIGH);
  digitalWrite(INPUT3, HIGH);
  digitalWrite(INPUT4, LOW);
}

void tankTurnRight() {
  analogWrite(ENABLE_LEFT, turningSpeed);
  analogWrite(ENABLE_RIGHT, turningSpeed);
  digitalWrite(INPUT1, HIGH);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, LOW);
  digitalWrite(INPUT4, HIGH);
}

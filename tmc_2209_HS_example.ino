#include <Arduino.h>
#include <TMCStepper.h>

// Pin definitions
const uint8_t ENABLE_PIN = 4;
const uint8_t UART_RX = 1;
const uint8_t UART_TX = 0;

// Rotary Encoder pins
const uint8_t ENCODER_CLK = 26;
const uint8_t ENCODER_DT = 27;
const uint8_t ENCODER_SW = 28;

// RGB LED pins
const uint8_t LED_R = 6;
const uint8_t LED_G = 7;
const uint8_t LED_B = 8;

// Motor parameters
const uint16_t STEPS_PER_REVOLUTION = 200;
const uint16_t MICROSTEPS = 16;
const int32_t MAX_SPEED = 72000; // steps per second
const int32_t MIN_SPEED = -72000; // steps per second
const uint8_t SPEED_LEVELS = 24;
const int32_t SPEED_PER_LEVEL = 3000;

// Acceleration parameters
int32_t acceleration = 3000; // steps per second^2
const int32_t MIN_ACCELERATION = 100;
const int32_t MAX_ACCELERATION = 10000;

// TMC2209 parameters
const float R_SENSE = 0.11f;
const uint8_t DRIVER_ADDRESS = 0b00;

// Current settings
const uint16_t MIN_CURRENT = 500;  // 500mA
const uint16_t MAX_CURRENT = 1500; // 1500mA
const uint16_t CURRENT_STEP = 100; // 100mA per step
uint16_t currentSetting = 600;     // Start with 600mA (you can adjust this)

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

volatile int encoderValue = 0;
volatile bool motorEnabled = false;
int32_t currentSpeed = 0;
int32_t targetSpeed = 0;
int8_t currentSpeedLevel = 0;
bool isSpreadCycle = true; // Start in SpreadCycle mode

unsigned long lastSpeedUpdateTime = 0;

// New enum for rotation direction
enum RotationDirection {
  CW,
  CCW,
  BOTH
};

// Global variable to store current rotation direction
RotationDirection currentDirection = CW;

void encoderISR() {
  static int lastCLK = 0;
  int CLK = digitalRead(ENCODER_CLK);
  int DT = digitalRead(ENCODER_DT);
  
  if (CLK != lastCLK) {
    if (DT != CLK) {
      encoderValue++;
    } else {
      encoderValue--;
    }
  }
  lastCLK = CLK;
}

void resetMotorState() {
  currentSpeed = 0;
  targetSpeed = 0;
  currentSpeedLevel = 0;
  encoderValue = 0;
  driver.VACTUAL(0); // Stop the motor
}

void enableMotor(bool enable) {
  digitalWrite(ENABLE_PIN, enable ? LOW : HIGH);  // LOW enables, HIGH disables
  if (enable) {
    Serial.println("Motor driver enabled");
  } else {
    Serial.println("Motor driver disabled");
  }
}

void updateLED() {
  if (!motorEnabled) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  } else {
    // Motor is ON, always light up the red LED
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    
    // If rotating, also light up blue for CCW
    if (currentSpeed < 0) {
      digitalWrite(LED_B, HIGH);
    } else {
      digitalWrite(LED_B, LOW);
    }
  }
}

void buttonISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    motorEnabled = !motorEnabled;
    if (motorEnabled) {
      enableMotor(true);
      // Reset motor state when enabling
      resetMotorState();
    } else {
      enableMotor(false);
      resetMotorState();
    }
    updateLED();
    lastInterruptTime = interruptTime;
  }
}

void setCurrent(uint16_t current) {
  currentSetting = constrain(current, MIN_CURRENT, MAX_CURRENT);
  driver.rms_current(currentSetting);
  Serial.print("Motor current set to ");
  Serial.print(currentSetting);
  Serial.println("mA");
}

void setSpreadCycle(bool enabled) {
  if (enabled) {
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(true);
    isSpreadCycle = true;
    Serial.println("SpreadCycle mode enabled");
  } else {
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    isSpreadCycle = false;
    Serial.println("StealthChop mode enabled");
  }
}

void setAcceleration(int32_t newAcceleration) {
  acceleration = constrain(newAcceleration, MIN_ACCELERATION, MAX_ACCELERATION);
  Serial.print("Acceleration set to ");
  Serial.print(acceleration);
  Serial.println(" steps/s^2");
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  enableMotor(false);
  resetMotorState();
  updateLED();
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  
  // Use attachInterrupt with a lambda function for the button
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW), []() {
    buttonISR();
  }, FALLING);
  
  Serial.begin(115200);
  Serial1.setRX(UART_RX);
  Serial1.setTX(UART_TX);
  Serial1.begin(115200);
  
  driver.begin();
  driver.toff(5);
  setCurrent(currentSetting);  // Set initial current
  driver.microsteps(MICROSTEPS);
  
  // Enable SpreadCycle mode
  setSpreadCycle(true);
  
  // Enable internal indexer
  driver.VACTUAL(0);
  driver.shaft(false); // Set the direction (CW)
  
  Serial.println("Stepper motor control system initialized");
  Serial.println("Current rotation direction: CW");
  Serial.println("Use 'c+' to increase current, 'c-' to decrease current");
  Serial.println("Use 's' to toggle between SpreadCycle and StealthChop modes");
  Serial.println("Use 'a+' to increase acceleration, 'a-' to decrease acceleration");
}

void loop() {
  static int lastEncoderValue = 0;
  
  if (motorEnabled) {
    if (encoderValue != lastEncoderValue) {
      int change = encoderValue - lastEncoderValue;
      
      // Apply direction constraints and update speed
      switch (currentDirection) {
        case CW:
          currentSpeedLevel = constrain(currentSpeedLevel + change, 0, SPEED_LEVELS);
          break;
        case CCW:
          currentSpeedLevel = constrain(currentSpeedLevel + change, -SPEED_LEVELS, 0);
          break;
        case BOTH:
          currentSpeedLevel = constrain(currentSpeedLevel + change, -SPEED_LEVELS, SPEED_LEVELS);
          break;
      }
      
      targetSpeed = currentSpeedLevel * SPEED_PER_LEVEL;
      lastEncoderValue = encoderValue;
    }
    
    // Apply acceleration
    unsigned long currentTime = millis();
    if (currentTime - lastSpeedUpdateTime >= 10) { // Update every 10ms
      int32_t speedDiff = targetSpeed - currentSpeed;
      int32_t maxSpeedChange = acceleration / 100; // acceleration per 10ms
      
      if (abs(speedDiff) > maxSpeedChange) {
        currentSpeed += (speedDiff > 0) ? maxSpeedChange : -maxSpeedChange;
      } else {
        currentSpeed = targetSpeed;
      }
      
      driver.VACTUAL(currentSpeed);
      lastSpeedUpdateTime = currentTime;
      
      // Print debug information
      Serial.print("Speed Level: ");
      Serial.print(currentSpeedLevel);
      Serial.print("/");
      Serial.print(SPEED_LEVELS);
      Serial.print(", Current Speed: ");
      Serial.print(currentSpeed);
      Serial.print(" steps/s / ");
      Serial.print(currentSpeed * 60 / (STEPS_PER_REVOLUTION * MICROSTEPS));
      Serial.print(" rpm, Target Speed: ");
      Serial.print(targetSpeed);
      Serial.print(" steps/s, Direction: ");
      Serial.println(currentSpeed > 0 ? "CW" : (currentSpeed < 0 ? "CCW" : "Stopped"));
      
      updateLED();
    }
  } else {
    // Reset motor state when disabled
    resetMotorState();
    lastEncoderValue = 0;
  }
  
  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "c+") {
      setCurrent(currentSetting + CURRENT_STEP);
    } else if (input == "c-") {
      setCurrent(currentSetting - CURRENT_STEP);
    } else if (input == "s") {
      setSpreadCycle(!isSpreadCycle);
    } else if (input == "a+") {
      setAcceleration(acceleration + 100);
    } else if (input == "a-") {
      setAcceleration(acceleration - 100);
    }
  }
}
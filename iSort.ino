// iSort — Arduino-based color-sorting conveyor belt
// Buttons are active-HIGH
// IRs are active-LOW.
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
// define pins
const int MOTOR_PIN1 = 10;
const int MOTOR_PIN2 = 9;
const int SERVO_PIN = 2;
const int START_PIN = 4;
const int REVERSE_PIN = 39;
const int RED_LED = 29;
const int BLUE_LED = 25;
const int GREEN_LED = 33;
const int IR_PIN = 51;
const int BUZZER_PIN = 5;
const int CLEAR_ANGLE = 18;
const int RED_ANGLE = 145;
const int BLUE_ANGLE = 65;
const int BLOCK_ANGLE = 97;
// different intervals for proper timing
const unsigned long DEBOUNCE_TIME = 30;
const unsigned long STARTUP_LED_TIME = 500;
const unsigned long STARTUP_BEEP_TIME = 500;
const unsigned long COLOR_WAIT_TIME = 1500;
const unsigned long SERVO_MOVE_TIME = 500;
const unsigned long REVERSE_TIME = 3500;
const unsigned long BEEP_TIME = 200;
const unsigned long SENSOR_CHECK_TIME = 5;
const unsigned long SENSOR_TIMEOUT = 250;
// color classification threshholds
const float MIN_COLOR_AMOUNT = 0.45;
const float MIN_COLOR_DIFFERENCE = 0.10;
const float ROUNDING_ERROR = 0.000001;
const float BELT_RED_MIN = 0.4;
const float BELT_GREEN_MIN = 0.4;
const float BELT_BLUE_MAX = 0.5;
const int MIN_LIGHT = 50;
const int MAX_LIGHT = 21504;

// I2C communication with 50.4 ms integration (0xEB), 4x gain (0x01) for the color sensor
const int SENSOR_ADDRESS = 0x29;
const int COMMAND_BYTE = 0x80;
const int AUTO_INCREMENT = 0x20;
const int ENABLE_REGISTER = 0x00;
const int TIME_REGISTER = 0x01;
const int GAIN_REGISTER = 0x0F;
const int ID_REGISTER = 0x12;
const int STATUS_REGISTER = 0x13;
const int CLEAR_REGISTER = 0x14;
const int POWER_ON = 0x01;
const int SENSOR_ON = 0x02;
const unsigned long SENSOR_POWER_TIME = 3;
const unsigned long SENSOR_READ_TIME = 51;

enum State {
  SENSOR_POWER, SENSOR_WARMUP, STARTUP_LIGHTS, STARTUP_BEEP,
  IDLE, SENSING, CLASSIFIED_WAIT, REJECT_BEEPS, REJECT_REVERSE, FLAP_SETTLE, FAULT
};

Servo servo; // flap
// Store each button's last reading.
bool lastStartReading = false;
bool startButtonPressed = false;
unsigned long startChangeTime = 0;
bool lastReverseReading = false;
bool reverseButtonPressed = false;
unsigned long reverseChangeTime = 0;
State state = SENSOR_POWER;
unsigned long stateStartTime = 0;
unsigned long pauseTime = 0;
unsigned long lastReadTime = 0;
bool isRunning = false;
bool readyForObject = true;
bool isReversing = false;
int colorNumber = -1;
int buzzerTone = 0;

void changeState(State newState, unsigned long currentTime);
bool isSortingState();
void setLeds(bool redOn, bool greenOn, bool blueOn);
void setBuzzer(int toneValue);
void motorStop();
void motorBrake();
void motorForward();
void motorReverse();
void applyMotor();
void logMessage(const char message[]);
void sensorFault(const char message[], unsigned long currentTime);
bool sensorWrite(int registerNumber, int newValue);
bool sensorRead(int registerNumber, byte readings[], int dataLength);
void initializeSensor(unsigned long currentTime);
int classifyColor(float redValue, float greenValue, float blueValue);
void readAndClassify(unsigned long currentTime);
void applyClassification(unsigned long currentTime);
void updateState(unsigned long currentTime);
bool readButtons(unsigned long currentTime);

void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  motorStop();
  servo.attach(SERVO_PIN);
  servo.write(CLEAR_ANGLE);
  pinMode(START_PIN, INPUT);
  pinMode(REVERSE_PIN, INPUT);
  lastStartReading = digitalRead(START_PIN) == HIGH;
  startButtonPressed = lastStartReading;
  lastReverseReading = digitalRead(REVERSE_PIN) == HIGH;
  reverseButtonPressed = lastReverseReading;
  startChangeTime = millis();
  reverseChangeTime = millis();
  pinMode(IR_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  setLeds(false, false, false);
  setBuzzer(0);
  Serial.begin(9600);
  Wire.begin();  // Pins on Arudino Mega are DA = 20, SCL = 21.
  Wire.setWireTimeout(3000, true); 
  initializeSensor(millis());
}

void loop() {
  unsigned long currentTime = millis();
  bool startPressed = readButtons(currentTime);
  if (state == FAULT) {
    if (startPressed) {
      initializeSensor(currentTime);
    }
  } else if (isSortingState()) {
    if (startPressed) {
      isRunning = !isRunning;
      if (isRunning) {
        stateStartTime += currentTime - pauseTime;  // Freeze timed progress.
      } else {
        pauseTime = currentTime;
        setBuzzer(0);
      }
    }
    // The reverse button has to be pressed and held to move the belt backwards while paused
    bool reversePressed = !isRunning && reverseButtonPressed;
    if (reversePressed && !isReversing) {
      changeState(IDLE, currentTime);
      readyForObject = true;
      colorNumber = -1;
      setLeds(false, false, false);
      setBuzzer(0);
    }
    isReversing = reversePressed;
    if (state == IDLE && digitalRead(IR_PIN) == HIGH) {
      readyForObject = true;
    }
  }
  updateState(millis());
  applyMotor(); 
}


bool readButtons(unsigned long currentTime) {
  bool startPressed = false;
  bool startReading = digitalRead(START_PIN) == HIGH;
  bool reverseReading = digitalRead(REVERSE_PIN) == HIGH;

  if (startReading != lastStartReading) {
    lastStartReading = startReading;
    startChangeTime = currentTime;
  }
  if (currentTime - startChangeTime >= DEBOUNCE_TIME &&
      startButtonPressed != lastStartReading) {
    startButtonPressed = lastStartReading;
    startPressed = startButtonPressed;
  }

  if (reverseReading != lastReverseReading) {
    lastReverseReading = reverseReading;
    reverseChangeTime = currentTime;
  }
  if (currentTime - reverseChangeTime >= DEBOUNCE_TIME &&
      reverseButtonPressed != lastReverseReading) {
    reverseButtonPressed = lastReverseReading;
  }
  return startPressed;
}

void changeState(State newState, unsigned long currentTime) {
  state = newState;
  stateStartTime = currentTime;
}

bool isSortingState() {
  return state == IDLE || state == SENSING ||
         state == CLASSIFIED_WAIT || state == REJECT_BEEPS ||
         state == REJECT_REVERSE || state == FLAP_SETTLE;
}

// audio output function
void setBuzzer(int toneValue) {
  if (toneValue == buzzerTone) return;
  buzzerTone = toneValue;
  if (toneValue == 0) {
    noTone(BUZZER_PIN);
  } else {
    tone(BUZZER_PIN, toneValue);
  }
}
// function to light up the right led
void setLeds(bool redOn, bool greenOn, bool blueOn) {
  digitalWrite(RED_LED, redOn);
  digitalWrite(GREEN_LED, greenOn);
  digitalWrite(BLUE_LED, blueOn);
}

// motor controls
void motorStop() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
}
void motorBrake() {
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, HIGH);
}
void motorForward() {
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);
}
void motorReverse() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, HIGH);
}

void applyMotor() {
  if (!isSortingState()) {
    motorStop();
  } else if (!isRunning) {
    if (isReversing) {
      motorReverse();
    } else {
      motorStop();
    }
  } else if (state == IDLE) {
    motorForward();
  } else if (state == REJECT_REVERSE) {
    motorReverse();
  } else {
    motorBrake();
  }
}

void logMessage(const char message[]) {
  int dataLength = strlen(message);
  if (Serial.availableForWrite() >= dataLength + 1) {
    Serial.print(message);
    Serial.write('\n');
  }
}

void sensorFault(const char message[], unsigned long currentTime) {
  isRunning = false;
  isReversing = false;
  colorNumber = -1;
  setBuzzer(0);
  setLeds(false, false, false);
  motorStop();
  changeState(FAULT, currentTime);
  logMessage(message);
}

bool sensorWrite(int registerNumber, int newValue) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(COMMAND_BYTE | registerNumber);
  Wire.write(newValue);
  int errorCode = Wire.endTransmission();
  return errorCode == 0 && !Wire.getWireTimeoutFlag();
}

bool sensorRead(int registerNumber, byte readings[], int dataLength) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(COMMAND_BYTE | AUTO_INCREMENT | registerNumber);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return false;
  if (Wire.requestFrom(SENSOR_ADDRESS, dataLength) != dataLength ||
      Wire.getWireTimeoutFlag()) return false;
  for (int i = 0; i < dataLength; i++) {
    readings[i] = Wire.read();
  }
  return true;
}

void initializeSensor(unsigned long currentTime) {
  isRunning = false;
  isReversing = false;
  readyForObject = true;
  colorNumber = -1;
  motorStop();
  setBuzzer(0);
  setLeds(false, false, false);
  Wire.clearWireTimeoutFlag();
  byte sensorId;
  if (!sensorRead(ID_REGISTER, &sensorId, 1) ||
      (sensorId != 0x44 && sensorId != 0x4D && sensorId != 0x10) ||
      !sensorWrite(ENABLE_REGISTER, 0) || !sensorWrite(TIME_REGISTER, 0xEB) ||
      !sensorWrite(GAIN_REGISTER, 0x01) || !sensorWrite(ENABLE_REGISTER, POWER_ON)) {
    sensorFault("Sensor initialization failed; START retries.", currentTime);
    return;
  }
  changeState(SENSOR_POWER, millis());
}

// RED = 1, BLUE = 2, GREEN = 3, BELT = 4, UNKNOWN = -1.
// Other ties/weak colors are UNKNOWN
// There is a condition to ignore the neoprene belt (no objects) and keep the belt running
// The classification works only for the 3D printed blocks we had. No callibration was performed due to time constraints.
int classifyColor(float redValue, float greenValue, float blueValue) {
  if (!isfinite(redValue) || !isfinite(greenValue) || !isfinite(blueValue) ||
      redValue < 0 || greenValue < 0 || blueValue < 0) return -1;

  float totalRgb = redValue + greenValue + blueValue;
  if (!isfinite(totalRgb) || totalRgb <= 0) return -1;

  float highestValue = 0;
  float secondValue = 0;
  int bestColor = -1;
  if (redValue > greenValue && redValue > blueValue) {
    highestValue = redValue;
    secondValue = max(greenValue, blueValue);
    bestColor = 1;
  } else if (greenValue > redValue && greenValue > blueValue) {
    highestValue = greenValue;
    secondValue = max(redValue, blueValue);
    bestColor = 3;
  } else if (blueValue > redValue && blueValue > greenValue) {
    highestValue = blueValue;
    secondValue = max(redValue, greenValue);
    bestColor = 2;
  }  

  float colorAmount = highestValue / totalRgb;
  float colorDifference = (highestValue - secondValue) / totalRgb;
  if (bestColor != -1 && colorAmount + ROUNDING_ERROR >= MIN_COLOR_AMOUNT &&
      colorDifference + ROUNDING_ERROR >= MIN_COLOR_DIFFERENCE) {
    return bestColor;
  }

  if (redValue > BELT_RED_MIN && greenValue > BELT_GREEN_MIN &&
      blueValue < BELT_BLUE_MAX) {
    return 4;
  }
  return -1;
}

void readAndClassify(unsigned long currentTime) {
  byte readings[8];
  if (!sensorRead(CLEAR_REGISTER, readings, sizeof(readings))) {
    sensorFault("Sensor read failed; START retries.", currentTime);
    return;
  }
  // Each sensor reading arrives as two bytes, which are joined into one integer below:
  unsigned int clearRaw = readings[0] | ((unsigned int)readings[1] << 8);
  unsigned int redRaw = readings[2] | ((unsigned int)readings[3] << 8);
  unsigned int greenRaw = readings[4] | ((unsigned int)readings[5] << 8);
  unsigned int blueRaw = readings[6] | ((unsigned int)readings[7] << 8);
  // if brightness is invalid, then there is a fault in the sensor
  if (clearRaw <= MIN_LIGHT) {
    sensorFault("Clear reading <= 50; START retries.", currentTime);
    return;
  }
  if (clearRaw >= MAX_LIGHT || redRaw >= MAX_LIGHT ||
      greenRaw >= MAX_LIGHT || blueRaw >= MAX_LIGHT) {
    sensorFault("Sensor saturated; reduce light. START retries.", currentTime);
    return;
  }
  float redValue = (float)redRaw / clearRaw;
  float greenValue = (float)greenRaw / clearRaw;
  float blueValue = (float)blueRaw / clearRaw;
  colorNumber = classifyColor(redValue, greenValue, blueValue);

  if (Serial.availableForWrite() >= 40) {
    Serial.print("RGBC: ");
    Serial.print(redRaw);
    Serial.print(' ');
    Serial.print(greenRaw);
    Serial.print(' ');
    Serial.print(blueRaw);
    Serial.print(' ');
    Serial.print(clearRaw);
    Serial.print("; ");
    switch (colorNumber) {
      case 1: Serial.print("RED"); break;
      case 2: Serial.print("BLUE"); break;
      case 3: Serial.print("GREEN"); break;
      case 4: Serial.print("BELT"); break;
      default: Serial.print("UNKNOWN"); break;
    }
    Serial.write('\n');
  }
  if (colorNumber == 4) {
    applyClassification(currentTime);  // Empty-belt reading: no sorting wait/rejection.
    return;
  }
  changeState(CLASSIFIED_WAIT, currentTime);
}

void applyClassification(unsigned long currentTime) {
  switch (colorNumber) {
    case 1:
      setLeds(true, false, false);
      servo.write(RED_ANGLE);
      break;
    case 2:
      setLeds(false, false, true);
      servo.write(BLUE_ANGLE);
      break;
    case 3:
      setLeds(false, true, false);
      servo.write(CLEAR_ANGLE);
      break;
    case 4:
      // No object to route: preserve flap/LED state and resume immediately.
      changeState(IDLE, currentTime);
      return;
    default:
      setLeds(false, false, false);
      changeState(REJECT_BEEPS, currentTime);
      setBuzzer(500);
      return;
  }
  changeState(FLAP_SETTLE, currentTime);
}

void updateState(unsigned long currentTime) {
  unsigned long passedTime = currentTime - stateStartTime;
  switch (state) {
    case SENSOR_POWER:
      if (passedTime >= SENSOR_POWER_TIME) {
        if (!sensorWrite(ENABLE_REGISTER, POWER_ON | SENSOR_ON)) {
          sensorFault("Sensor enable failed; START retries.", currentTime);
        } else {
          changeState(SENSOR_WARMUP, millis());
        }
      }
      break;
    case SENSOR_WARMUP:
      if (passedTime >= SENSOR_READ_TIME) {
        setLeds(true, true, true);
        changeState(STARTUP_LIGHTS, currentTime);
      }
      break;
    case STARTUP_LIGHTS:
      if (passedTime >= STARTUP_LED_TIME) {
        setLeds(false, false, false);
        setBuzzer(100);
        changeState(STARTUP_BEEP, currentTime);
        logMessage("TCS34725 ready.");
      }
      break;
    case STARTUP_BEEP:
      if (passedTime >= STARTUP_BEEP_TIME) {
        setBuzzer(0);
        changeState(IDLE, currentTime);
        pauseTime = currentTime;
      }
      break;
    case IDLE:
      if (isRunning && readyForObject && digitalRead(IR_PIN) == LOW) {
        readyForObject = false;
        motorBrake();  // Stop before starting a fresh integration.
        if (!sensorWrite(ENABLE_REGISTER, POWER_ON) ||
            !sensorWrite(ENABLE_REGISTER, POWER_ON | SENSOR_ON)) {
          sensorFault("Sensor acquisition failed; START retries.", currentTime);
        } else {
          changeState(SENSING, millis());
          lastReadTime = stateStartTime;
        }
      }
      break;
    case SENSING:
      if (!isRunning || passedTime < SENSOR_READ_TIME) break;
      if (passedTime >= SENSOR_TIMEOUT) {
        sensorFault("Sensor conversion timed out; START retries.", currentTime);
      } else if (currentTime - lastReadTime >= SENSOR_CHECK_TIME) {
        lastReadTime = currentTime;
        byte sensorStatus;
        if (!sensorRead(STATUS_REGISTER, &sensorStatus, 1)) {
          sensorFault("Sensor status failed; START retries.", currentTime);
        } else if (sensorStatus & 0x01) {
          readAndClassify(currentTime);
        }
      }
      break;
    case CLASSIFIED_WAIT:
      if (isRunning && passedTime >= COLOR_WAIT_TIME) {
        applyClassification(currentTime);
      }
      break;
    case REJECT_BEEPS:
      if (!isRunning) break;
      if (passedTime >= 5 * BEEP_TIME) {
        setBuzzer(0);
        servo.write(BLOCK_ANGLE);
        changeState(REJECT_REVERSE, currentTime);
      } else if ((passedTime / BEEP_TIME) % 2 == 0) {
        setBuzzer(500);
      } else {
        setBuzzer(0);
      }
      break;
    case REJECT_REVERSE:
      if (isRunning && passedTime >= REVERSE_TIME) {
        changeState(FLAP_SETTLE, currentTime);
      }
      break;
    case FLAP_SETTLE:
      if (isRunning && passedTime >= SERVO_MOVE_TIME) {
        changeState(IDLE, currentTime);
      }
      break;
    case FAULT:
      break;
  }
}

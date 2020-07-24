#include <ShiftRegister74HC595.h> //Hay que descargar manualmente esta libreria para poder compilar
#include <EEPROM.h>

//Addresses
const int wetStateInitialAddress = 0;
const int dryStateInitialAddress = 12;

//Pin connected to latch pin (ST_CP) of 74HC595
const int latchPin = 2;
//Pin connected to clock pin (SH_CP) of 74HC595
const int clockPin = 4;
////Pin connected to Data in (DS) of 74HC595
const int dataPin = 3;
ShiftRegister74HC595<1> shiftRegister(dataPin, clockPin, latchPin);

//Menu buttons
const int leftButton = 11;
const int rightButton = 9;

int wateringThresholdLevel;
float wateringThreshold;
float sensorsData [6];
float adjustedSensorsData [6];
int calibratedDryState [6];
int calibratedWetState [6];
int frame;
float menuTimeOut;

void writeIntIntoEEPROM(int address, int number)
{
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

int readIntFromEEPROM(int address)
{
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

void read_sensors() {
  sensorsData[0] = analogRead(A0);
  sensorsData[1] = analogRead(A1);
  sensorsData[2] = analogRead(A2);
  sensorsData[3] = analogRead(A3);
  sensorsData[4] = analogRead(A4);
  sensorsData[5] = analogRead(A5);

  for (int sensor = 0; sensor < 6; sensor++) {
    adjustedSensorsData[sensor] = (sensorsData[sensor] - calibratedWetState[sensor]) / (calibratedDryState[sensor] - calibratedWetState[sensor]);
  }
  Serial.println();

  Serial.print(adjustedSensorsData[0]);
  Serial.print('\t');
  Serial.print(adjustedSensorsData[1]);
  Serial.print('\t');
  Serial.print(adjustedSensorsData[2]);
  Serial.print('\t');
  Serial.print(adjustedSensorsData[3]);
  Serial.print('\t');
  Serial.print(adjustedSensorsData[4]);
  Serial.print('\t');
  Serial.println(adjustedSensorsData[5]);
}

int frame_counter_up(int currentFrame) {
  int newFrame;
  if (currentFrame < 60) {
    newFrame = currentFrame + 1;
  }
  else {
    newFrame = 0;
  }
  return newFrame;
}

void light_up_watering_threshold_adjustment() {
  switch (wateringThresholdLevel) {
    case 1 :
      shiftRegister.setAllLow();
      shiftRegister.set(6, HIGH);
      break;
    case 2 :
      shiftRegister.setAllLow();
      shiftRegister.set(5, HIGH);
      shiftRegister.set(6, HIGH);
      break;
    case 3 :
      shiftRegister.setAllLow();
      shiftRegister.set(4, HIGH);
      shiftRegister.set(5, HIGH);
      shiftRegister.set(6, HIGH);
      break;
    case 4 :
      shiftRegister.setAllHigh();
      shiftRegister.set(1, LOW);
      shiftRegister.set(2, LOW);
      break;
    case 5 :
      shiftRegister.setAllHigh();
      shiftRegister.set(1, LOW);
      break;
    case 6 :
      shiftRegister.setAllHigh();
      break;
    default :
      shiftRegister.setAllLow();
      shiftRegister.set(1, HIGH);
      shiftRegister.set(3, HIGH);
      shiftRegister.set(5, HIGH);
  }
}

void increase_watering_threshold() {
  float newWateringThreshold;
  if (wateringThresholdLevel >= 6) {
    newWateringThreshold = 6;
    shiftRegister.setAllLow();
    delay(100);
  }
  else {
    newWateringThreshold = wateringThresholdLevel + 1;
  }
  wateringThresholdLevel = newWateringThreshold;
}

void decrease_watering_threshold() {
  float newWateringThreshold;
  if (wateringThresholdLevel <= 1) {
    newWateringThreshold = 1;
    shiftRegister.setAllLow();
    delay(100);
  }
  else {
    newWateringThreshold = wateringThresholdLevel - 1;
  }
  wateringThresholdLevel = newWateringThreshold;
}

void left_button_function() {
  int pressedButtonTimer = 0;
  while (pressedButtonTimer < 300) {
    if (digitalRead(leftButton) == LOW) {
      decrease_watering_threshold();
      update_watering_threshold();
      return;
    }
    pressedButtonTimer += 1;
    delay(1000 / 60);
  }
  calibrate_wet_state();
  return;
}

void right_button_function() {
  int pressedButtonTimer = 0;
  while (pressedButtonTimer < 300) {
    if (digitalRead(rightButton) == LOW) {
      increase_watering_threshold();
      update_watering_threshold();
      return;
    }
    pressedButtonTimer += 1;
    delay(1000 / 60);
  }
  calibrate_dry_state();
  return;
}

void calibrate_dry_state() {
  for (int sensor = 0; sensor < 6; sensor++) {
    writeIntIntoEEPROM(dryStateInitialAddress + 2 * sensor, sensorsData[sensor]);
  }
  read_dry_calibration_from_memory();
  dry_calibration_blink_leds();
}

void read_dry_calibration_from_memory() {
  for (int sensor = 0; sensor < 6; sensor++) {
    calibratedDryState[sensor] = readIntFromEEPROM(dryStateInitialAddress + 2 * sensor);
    Serial.println(calibratedDryState[sensor]);
  }
}

void dry_calibration_blink_leds() {
  for (int repeats = 0; repeats < 4; repeats ++) {
    shiftRegister.setAllLow();
    shiftRegister.set(1, HIGH);
    shiftRegister.set(2, HIGH);
    shiftRegister.set(3, HIGH);
    delay(1000 / 8);
    shiftRegister.setAllLow();
    delay(1000 / 8);
  }
}

void calibrate_wet_state() {
  for (int sensor = 0; sensor < 6; sensor++) {
    writeIntIntoEEPROM(wetStateInitialAddress + 2 * sensor, sensorsData[sensor]);
  }
  read_wet_calibration_from_memory();
  wet_calibration_blink_leds();
}

void read_wet_calibration_from_memory() {
  for (int sensor = 0; sensor < 6; sensor++) {
    calibratedWetState[sensor] = readIntFromEEPROM(wetStateInitialAddress + 2 * sensor);
    Serial.println(calibratedWetState[sensor]);
  }
}

void wet_calibration_blink_leds() {
  for (int repeats = 0; repeats < 4; repeats ++) {
    shiftRegister.setAllLow();
    shiftRegister.set(4, HIGH);
    shiftRegister.set(5, HIGH);
    shiftRegister.set(6, HIGH);
    delay(1000 / 8);
    shiftRegister.setAllLow();
    delay(1000 / 8);
  }
}

void enter_menu() {
  menuTimeOut = 0;
  frame = 0;
  while (menuTimeOut < 8.0) {
    frame = frame_counter_up(frame);
    // Light up watering level threshold
    if (frame < 30) {
      light_up_watering_threshold_adjustment();
    }
    else {
      shiftRegister.setAllLow();
    }
    // React to buttons
    if (digitalRead(leftButton) == HIGH) {
      left_button_function();
      frame = 0;
      menuTimeOut = 0.0;
    }
    if (digitalRead(rightButton) == HIGH) {
      right_button_function();
      frame = 0;
      menuTimeOut = 0.0;
    }
    delay(1000 / 60); //read states 60 times per second
    menuTimeOut += 0.02;
  }
}

void update_watering_threshold() {
  wateringThreshold = wateringThresholdLevel / 7.0;
  Serial.println(wateringThreshold)
}

void setup() {
  Serial.begin(9600);
  shiftRegister.setAllLow();
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  wateringThresholdLevel = 3;
  update_watering_threshold();
  read_dry_calibration_from_memory();
  read_wet_calibration_from_memory();
}

void loop() {
  read_sensors();
  for (int sensor = 0; sensor < 6; sensor++) {
    if (adjustedSensorsData[sensor] > wateringThreshold) {
      shiftRegister.set(sensor + 1, HIGH);
    }
    else {
      shiftRegister.set(sensor + 1, LOW);
    }
  }
  if ((digitalRead(leftButton) == HIGH) || (digitalRead(rightButton) == HIGH)) {
    enter_menu();
  }
  delay(1000 / 30);
}
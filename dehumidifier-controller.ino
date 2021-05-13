// libraries
#include <Wire.h>
#include <Adafruit_AM2315.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// initialize display
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

// initialize sensor
Adafruit_AM2315 am2315;
float temperature;
float temperatureF;
float humidity;

// initialize relays
const int dehumidifierRelay = 26;
const int fanRelay = 27;

// initialize LED
const int redPin = 12;
const int greenPin = 14;

//initialize buttons
const int dispBtnPin = 25;
int dispBtnState = 0;
int dispBtnPrevState = 0;
unsigned long dispBtnDebTime = 0;// debounce time
const int incBtnPin = 35;
int incBtnState = 0;
int incBtnPrevState = 0;
unsigned long incBtnDebTime = 0;
const int decBtnPin = 34;
int decBtnState = 0;
int decBtnPrevState = 0;
unsigned long decBtnDebTime = 0;
unsigned long debounceDelay = 50;

// other variables
int humiditySetting = 60;
int lowTempSetting = 55; // degrees F
String humiditySettingDisplay = String(humiditySetting);
String humidityCurrentDisplay = String(humidity);
int humidityThreshold = 2;
int lowTempThreshold = 2;
bool dehumidifierActive = false;
bool fanActive = false;
bool errorExists = false;
bool LEDOn = false;
bool displayOn = false;
bool lowTemp = false;
unsigned long onTime = millis();
unsigned long offTime = millis();
unsigned long lastRecording = millis();
unsigned long lastReading = millis();
unsigned long displayTime = millis();
unsigned long LEDOnTime = millis();
unsigned long LEDOffTime = millis();

void setup() {
  Serial.begin(9600);
  
  // setup display I2C address
  alpha4.begin(0x70);

  // setup pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  pinMode(dispBtnPin, INPUT);
  pinMode(incBtnPin, INPUT);
  pinMode(decBtnPin, INPUT);
  pinMode(dehumidifierRelay, OUTPUT);
  pinMode(fanRelay, OUTPUT);
  digitalWrite(dehumidifierRelay, HIGH);
  digitalWrite(fanRelay, HIGH);

  // if sensor not found, print to screen and light LED red
  if (! am2315.begin()) {
    Serial.println("Sensor not found, check wiring & pullups!");
    digitalWrite(redPin, HIGH);
    while (1);
  }

  // display initial humidity setting
  alpha4.setBrightness(8);
  alpha4.writeDigitAscii(2, humiditySettingDisplay[0]);
  alpha4.writeDigitAscii(3, humiditySettingDisplay[1]);
  alpha4.writeDisplay();
  displayTime = millis();
  displayOn = true;

  delay(5000);
}

void loop() {
  // read temp/humidity every 5 seconds. if errors encountered: send message, and flash red LED
  if (millis()-lastReading > 5000 || lastReading == 0) {
    if (! am2315.readTemperatureAndHumidity(&temperature, &humidity)) {
    Serial.println("Failed to read data from AM2315");
    errorExists = true;
    }
    else {
      errorExists = false;
    }
    lastReading = millis();
    humidityCurrentDisplay = String(humidity);
    temperatureF = temperature*1.8+32;
    if (displayOn) {
      alpha4.writeDigitAscii(0, humidityCurrentDisplay[0]);
      alpha4.writeDigitAscii(1, humidityCurrentDisplay[1]);
      alpha4.writeDisplay();
    }
  }

// check for low temp condition
  if (temperatureF < lowTempSetting-lowTempThreshold) {
    lowTemp = true;
  }
  else if (temperatureF > lowTempSetting+lowTempThreshold) {
    lowTemp = false;
  }

  // turn on dehumidifier if humidity is high, dehumidifier is not currently active, and it's been at least 5 minutes since it turned off
  if (humidity > humiditySetting+humidityThreshold && !lowTemp && !dehumidifierActive && millis()-offTime > 300000) {
    dehumidifierOn();
  }

  // turn off dehumidifier if the humidity is low, dehumidifier is currently active, and it's been at least 5 minutes since it turned on
  if (humidity < humiditySetting-humidityThreshold && dehumidifierActive && millis()-onTime > 300000) {
    dehumidifierOff();
  }

  // record readings to server every 5 minutes
  if (!errorExists && (millis()-lastRecording > 10000 || lastRecording == 0)) {
    Serial.print("Temp *F: "); Serial.println(temperatureF);
    Serial.print("Hum %: "); Serial.println(humidity);
    Serial.println(lowTemp);
    lastRecording = millis();
  }

  // turn off dehumidifier if it runs longer than 55 minutes. send message
  if (dehumidifierActive && millis()-onTime > 3300000) {
    dehumidifierOff();
    Serial.println("Dehumidifier has been running continuously for 55 minutes. Turned off for 5 minutes.");
  }

  // if display has been on more than 5 minutes with no change, turn it off
  if (millis()-displayTime > 30000) {
    alpha4.writeDigitRaw(0, 0x0);
    alpha4.writeDigitRaw(1, 0x0);
    alpha4.writeDigitRaw(2, 0x0);
    alpha4.writeDigitRaw(3, 0x0);
    alpha4.writeDisplay();
    displayTime = millis();
    displayOn = false;
  }

  // turn display on if button pressed
  checkDisplayButton();

  // turn display on and increase humidity setting if button pressed
  checkIncreaseButton();

  // turn display on and decrease humidity setting if button pressed
  checkDecreaseButton();

  // LED control.
  blinkLED();
}

void dehumidifierOn() {
  digitalWrite(dehumidifierRelay, LOW);
  dehumidifierActive = true;
  onTime = millis();
  // send message to track usage
}

void dehumidifierOff() {
  digitalWrite(dehumidifierRelay, HIGH);
  dehumidifierActive = false;
  offTime = millis();
  // send message to track usage
}

void fanOn() {
  digitalWrite(fanRelay, LOW);
}

void fanOff() {
  digitalWrite(fanRelay, HIGH);
}

void checkDisplayButton() {
  int reading = digitalRead(dispBtnPin);
  if (reading != dispBtnPrevState) {
    dispBtnDebTime = millis();
  }
  if (millis()-dispBtnDebTime > debounceDelay) {
    if (reading != dispBtnState) {
      dispBtnState = reading;
      if (dispBtnState == HIGH) {
        wakeupDisplay();
      }
    }
  }
  dispBtnPrevState = reading;
}

void checkIncreaseButton() {
  int reading = digitalRead(incBtnPin);
  if (reading != incBtnPrevState) {
    incBtnDebTime = millis();
  }
  if (millis()-incBtnDebTime > debounceDelay) {
    if (reading != incBtnState) {
      incBtnState = reading;
      if (incBtnState == HIGH) {
        humiditySetting++;
        humiditySettingDisplay = String(humiditySetting);
        wakeupDisplay();
      }
    }
  }
  incBtnPrevState = reading;
}

void checkDecreaseButton() {
  int reading = digitalRead(decBtnPin);
  if (reading != decBtnPrevState) {
    decBtnDebTime = millis();
  }
  if (millis()-decBtnDebTime > debounceDelay) {
    if (reading != decBtnState) {
      decBtnState = reading;
      if (decBtnState == HIGH) {
        humiditySetting--;
        humiditySettingDisplay = String(humiditySetting);
        wakeupDisplay();
      }
    }
  }
  decBtnPrevState = reading;
}

void wakeupDisplay() {
  alpha4.writeDigitAscii(0, humidityCurrentDisplay[0]);
  alpha4.writeDigitAscii(1, humidityCurrentDisplay[1]);
  alpha4.writeDigitAscii(2, humiditySettingDisplay[0]);
  alpha4.writeDigitAscii(3, humiditySettingDisplay[1]);
  alpha4.writeDisplay();
  displayTime = millis();
  displayOn = true;
}

void blinkLED() {
  if (errorExists){
    // blink rapid red if error exists
    digitalWrite(greenPin, LOW);
    if (!LEDOn && millis()-LEDOffTime > 1000) {
      digitalWrite(redPin, HIGH);
      LEDOn = true;
      LEDOnTime = millis();
    }
    else if (LEDOn && millis()-LEDOnTime > 1000) {
      digitalWrite(redPin, LOW);
      LEDOn = false;
      LEDOffTime = millis();
    }
  }
  // blink slow red if low temp detected
  else if (lowTemp) {
    digitalWrite(greenPin, LOW);
    if (!LEDOn && millis()-LEDOffTime > 4000) {
      digitalWrite(redPin, HIGH);
      LEDOn = true;
      LEDOnTime = millis();
    }
    else if (LEDOn && millis()-LEDOnTime > 1000) {
      digitalWrite(redPin, LOW);
      LEDOn = false;
      LEDOffTime = millis();
    }
  }
  // blink fast green if in initial 5 minute wait period
  else if ((offTime == 0 && millis()-offTime < 300000) || (humidity < humiditySetting-humidityThreshold && millis()-onTime < 300000)) {
    digitalWrite(redPin, LOW);
    if (!LEDOn && millis()-LEDOffTime > 1000) {
      digitalWrite(greenPin, HIGH);
      LEDOn = true;
      LEDOnTime = millis();
    }
    else if (LEDOn && millis()-LEDOnTime > 1000) {
      digitalWrite(greenPin, LOW);
      LEDOn = false;
      LEDOffTime = millis();
    }
  }
  // blink slow green for normal operation
  else {
    digitalWrite(redPin, LOW);
    if (!LEDOn && millis()-LEDOffTime > 4000) {
      digitalWrite(greenPin, HIGH);
      LEDOn = true;
      LEDOnTime = millis();
    }
    else if (LEDOn && millis()-LEDOnTime > 1000) {
      digitalWrite(greenPin, LOW);
      LEDOn = false;
      LEDOffTime = millis();
    }
  }
}
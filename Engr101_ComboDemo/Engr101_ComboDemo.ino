/***********************************************************
Merged and modified from several uctronics sensor examples
***********************************************************/
#include <LiquidCrystal.h>
#include <dht11.h>
#include <Wire.h>

const int echoPin = 5;  // pin connected to Echo Pin in the ultrasonic distance sensor
const int trigPin = 6;  // pin connected to trig Pin in the ultrasonic distance sensor
int redPin = 11;        // R petal on RGB LED module connected to digital pin 11
int greenPin = 10;      // G petal on RGB LED module connected to digital pin 10
int bluePin = 9;        // B petal on RGB LED module connected to digital pin 9
char cmd = ' ';

int adc_id = A3;
int HistoryValue = 0;
char buffer[128];

const int PIRpin = 4;

dht11 DHT11;
int DHT11PIN = A2;

int thermistorPin = A1;

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

LiquidCrystal lcd(32, 30, 28, 26, 24, 22);

void setup() {
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);    // sets the redPin to be an output
  pinMode(greenPin, OUTPUT);  // sets the greenPin to be an output
  pinMode(bluePin, OUTPUT);   // sets the bluePin to be an output
  color(0, 0, 0);

  pinMode(PIRpin, INPUT);
  pinMode(echoPin, INPUT);   //Set the connection pin output mode Echo pin
  pinMode(trigPin, OUTPUT);  //Set the connection pin output mode trog pin
  lcd.begin(16, 2);          //set up the LCD's number of columns and rows:
  lcd.clear();               //Clears the LCD screen and positions the cursor in the upper-left corner
  accelSetup();
  delay(1000);  //delay 1000ms
}

void loop() {
  bool isNew = false;

  if (Serial.available()) {
    char tst = Serial.read();
    if (!((tst == '\n') | (tst == '\r'))) {
      cmd = tst;
      isNew = true;
      // Serial.print("got ");
      // Serial.println(cmd);
    } else {
      color(0, 0, 0);
    }
    // Serial.print("got ");
    // Serial.println(tst, HEX);
  }
  switch (cmd) {
    case 'p':
      if (isNew) {
        Serial.println("PIR");
        lcd.clear();
        lcd.println("PIR");
      }
      pirLoop();
      break;
    case 'u':
      if (isNew) Serial.println("Ultrasonic");
      ultrasonicLoop();
      break;

    case 'h':
      if (isNew) Serial.println("DHT temp+humidity");
      dhtloop();
      break;

    case 't':
      if (isNew) Serial.println("thermistor");
      readThermistor();
      break;

    case 's':
      if (isNew)
        ;
      Serial.println("Sound sensor");
      soundloop();
      break;
    case 'a':
      if (isNew) {
        Serial.print("MPU6050 Accelerometer");
        lcd.setCursor(0, 0);
        lcd.print("Accelerometer");
      }
      accelLoop();
      break;
    case 'w':
      if (isNew) {
        Serial.println("Water level");
        lcd.clear();
        lcd.print("Water level");
      }
      waterloop();
      break;

      case 'r':
      lcd.clear();
      lcd.print("RAVE TIME!!");
      color(random(), random(), random());
      delay(20);
      break;
    default:
      if (isNew) {
        lcd.clear();
        color(0, 0, 0);
        lcd.print("no comprende...");
        delay(500);
      }
  }
}

void pirLoop(void) {
  bool pirVal = digitalRead(PIRpin);
  lcd.setCursor(0, 1);
  lcd.print(pirVal);

  if (pirVal == LOW) {
    color(0, 0, 0);
  } else {
    // digitalWrite(ledpin, HIGH);
    color(255, 128, 0);
  }
  delay(200);
}

int ping(int echoPin) {
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm;
}

void ultrasonicLoop(void) {
  int cm = ping(echoPin);
  lcd.setCursor(0, 0);      // set the cursor to column 0, line 0
  lcd.print("distance: ");  // Print a message of "Temp: "to the LCD.
  lcd.print(cm);            // Print a centigrade temperature to the LCD.
  lcd.print(" cm    ");     // Print the unit of the centigrade temperature to the LCD.
  Serial.print("distance: ");
  Serial.println(cm);
  delay(500);
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void color(unsigned char red, unsigned char green, unsigned char blue)  // the color generating function
{
  analogWrite(redPin, 255 - red);      // PWM signal output
  analogWrite(greenPin, 255 - green);  // PWM signal output
  analogWrite(bluePin, 255 - blue);    // PWM signal output
}

void waterloop() {
  long temp_max, temp_min, temp_value, temp;
  long value = 0;
  temp_max = analogRead(adc_id);  // get adc value
  temp_min = temp_max;
  temp_value = temp_max;

  for (int i = 0; i < 201; i++) {
    temp = analogRead(adc_id);
    if (temp_max < temp) temp_max = temp;
    if (temp_min > temp) temp_min = temp;
    temp_value += temp;
  }
  value = (temp_value - temp_max - temp_min) / 200;
  if (HistoryValue != value) {
    sprintf(buffer, "ADC%d level is %d\n", adc_id, value);
    Serial.print(buffer);
    HistoryValue = value;
    color(0, 0, (uint8_t)value >> 2);
    delay(500);
  }
}


void dhtloop() {
  int chk = DHT11.read(DHT11PIN);
  lcd.setCursor(0, 0);                  // set the cursor to column 0, line 0
  lcd.print("Humidity:");               // Print a message of "Humidity: "to the LCD.
  lcd.print((float)DHT11.humidity, 2);  // Print a message of "Humidity: "to the LCD.
  lcd.print(" % ");                     // Print the unit of the centigrade temperature to the LCD.

  lcd.setCursor(0, 1);                     // set the cursor to column 0, line 1
  lcd.print("Temp:    ");                  // Print a message of "Temp: "to the LCD.
  lcd.print((float)DHT11.temperature, 2);  // Print a centigrade temperature to the LCD.
  lcd.print(" C ");                        // Print the unit of the centigrade temperature to the LCD.
  delay(1000);
}

void accelSetup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void accelLoop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = ");
  Serial.print(AcX);
  Serial.print(" | AcY = ");
  Serial.print(AcY);
  Serial.print(" | AcZ = ");
  Serial.print(AcZ);
  Serial.print(" | Tmp = ");
  Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = ");
  Serial.print(GyX);
  Serial.print(" | GyY = ");
  Serial.print(GyY);
  Serial.print(" | GyZ = ");
  Serial.println(GyZ);

  color(downsampleG(AcX), downsampleG(AcY), downsampleG(AcZ));
  delay(100);
}

uint8_t downsampleG(int16_t input) {
  unsigned char conv;
  conv = abs(input) >> 8;
  return conv;
}

void soundloop() {
  int sensorValue = analogRead(A0);
  // digitalWrite(ledPin,  HIGH);
  // delay(sensorValue);
  // digitalWrite(ledPin,  LOW);
  // delay(sensorValue);
  uint8_t colorval = (uint8_t)sensorValue;
  color(0, colorval, 0);
  Serial.println(sensorValue, DEC);
}

void readThermistor() {
  int tempReading = analogRead(thermistorPin);
  // This is OK
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK)) * tempK);  //  Temp Kelvin
  float tempC = tempK - 273.15;                                                           // Convert Kelvin to Celcius
  float tempF = (tempC * 9.0) / 5.0 + 32.0;                                               // Convert Celcius to Fahrenheit
  /*  replaced
    float tempVolts = tempReading * 5.0 / 1024.0;
    float tempC = (tempVolts - 0.5) * 10.0;
    float tempF = tempC * 9.0 / 5.0 + 32.0;
  */
  // Display Temperature in C
  lcd.setCursor(0, 0);
  lcd.print("Temp         C  ");
  // Display Temperature in F
  lcd.setCursor(0, 1);
  lcd.print("Temp         F  ");
  lcd.setCursor(6, 0);
  // Display Temperature in C
  lcd.print(tempC);
  lcd.setCursor(6, 1);
  // Display Temperature in F
  lcd.print(tempF);
  Serial.println(tempC);

  delay(500);
}
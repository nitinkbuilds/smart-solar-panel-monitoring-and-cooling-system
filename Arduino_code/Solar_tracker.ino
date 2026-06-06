#include <Servo.h>
#include <DHT.h>

// =====================================================
// SMART SOLAR TRACKER + TEMP CONTROLLED FAN SYSTEM
// =====================================================

// ================= DHT =================
#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// ================= LDR =================
const int ldrLeft  = A1;
const int ldrRight = A0;

// ================= SOLAR SERVO =================
const int solarServoPin = 9;

Servo solarServo;

// CENTER POSITION
const int solarCenter = 90;

// ONLY ±30 DEGREE MOVEMENT
const int solarLeftLimit  = 60;
const int solarRightLimit = 120;

int solarPos = solarCenter;

// sensitivity
const int ldrThreshold = 2;

// movement step
const int solarStep = 2;

// ================= FAN SERVO =================
const int fanServoPin = 10;

Servo fanServo;

int fanPos = 90;
int fanDir = 1;

const int fanMin = 60;
const int fanMax = 120;

// ================= MOTOR DRIVER =================
const int in1 = 7;
const int in2 = 8;
const int ena = 5;

// ================= TEMPERATURE =================
float tempThreshold = 31.0;
float maxTemp = 45.0;

bool fanOn = false;

unsigned long previousMillis = 0;
const long interval = 40;

void setup() {

  Serial.begin(9600);

  Serial.println("SMART SOLAR TRACKER STARTED");

  dht.begin();

  solarServo.attach(solarServoPin);
  fanServo.attach(fanServoPin);

  solarServo.write(solarCenter);
  fanServo.write(90);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);

  Serial.println("System Ready");
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

  
    float temp = dht.readTemperature();

    if (!isnan(temp)) {

      Serial.print("Temp: ");
      Serial.println(temp);

      controlFan(temp);
    }

    // ================= LDR =================
    int leftValue  = analogRead(ldrLeft);
    int rightValue = analogRead(ldrRight);

    Serial.print("L: ");
    Serial.print(leftValue);

    Serial.print(" R: ");
    Serial.println(rightValue);

    trackSun(leftValue, rightValue);

    Serial.println("--------------------");
  }
}


void controlFan(float temp) {

  if (temp > tempThreshold) {

    fanOn = true;

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    int speedVal = map(temp, tempThreshold, maxTemp, 120, 255);
    speedVal = constrain(speedVal, 120, 255);

    analogWrite(ena, speedVal);

  } else {

    fanOn = false;

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);

    analogWrite(ena, 0);
  }


  if (fanOn) {

    fanPos += fanDir * 2;

    if (fanPos >= fanMax || fanPos <= fanMin) {
      fanDir *= -1;
    }

  } else {

    // return to center slowly
    if (fanPos < 90) fanPos++;
    else if (fanPos > 90) fanPos--;
  }

  fanServo.write(fanPos);
}


void trackSun(int leftValue, int rightValue) {

  int difference = leftValue - rightValue;

  Serial.print("Diff: ");
  Serial.println(difference);

 
  if (abs(difference) <= ldrThreshold) {

    // smooth return to center
    if (solarPos < solarCenter) {
      solarPos += solarStep;
    }
    else if (solarPos > solarCenter) {
      solarPos -= solarStep;
    }
  }


  else if (leftValue > rightValue) {

    solarPos += solarStep;

    solarPos = constrain(solarPos,
                         solarCenter,
                         solarRightLimit);
  }

  
  else {

    solarPos -= solarStep;

    solarPos = constrain(solarPos,
                         solarLeftLimit,
                         solarCenter);
  }

  solarServo.write(solarPos);

  Serial.print("Solar Servo: ");
  Serial.println(solarPos);
}

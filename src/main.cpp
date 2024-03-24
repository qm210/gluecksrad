#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define DATA_PIN 14
#define CLOCK_PIN 27

#define N_LEDS_RIGHT 9
#define N_LEDS_LEFT 9
#define N_LEDS (N_LEDS_RIGHT + N_LEDS_LEFT)

#define MAX_BRIGHTNESS 20
#define FRAME_DELAY 50

CRGB leds[N_LEDS];

void runLight(unsigned long elapsedMs);
void fade();

int currentHeight = 0;
bool isCurrentlyRight = false;
CRGB currentColor = CRGB::Magenta;

using Vec3 = std::array<float, 3>;
Vec3 gravity;
Vec3 currentAcceleration = {0.f, 0.f, 0.f};

unsigned long _time;
unsigned long elapsedMs;

constexpr unsigned long increaseLightEveryMs = 500;
constexpr unsigned long switchSideEveryMs = 1000;
constexpr unsigned long readSensorCountEveryMs = 500;

unsigned long increaseCountMs = 0;
unsigned long switchSideCountMs = 0;
unsigned long readSensorCountMs = 0;

int indexLeft(int heightIndex) {
    return heightIndex;
}

int indexRight(int heightIndex) {
    return N_LEDS - 1 - heightIndex;
}

void setLeft(int heightIndex, CRGB target) {
    int i = indexLeft(heightIndex);
    leds[i] = target;
}

void setRight(int heightIndex, CRGB target) {
    int i = indexRight(heightIndex);
    leds[i] = target;
}

void set(int heightIndex, bool isRightSide, CRGB target) {
    int i = (isRightSide ? indexRight : indexLeft)(heightIndex);
    leds[i] = target;
}

bool useAccel = false;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displaySensorDetails(void)
{
  if(!accel.begin())
  {
    Serial.println("Could not intialize the ADXL345 - sorrie!");
    return;
  }

  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");

  useAccel = true;
  delay(500);
}

sensors_event_t event;

void initGravity() {
    if (!useAccel) {
        return;
    }
    accel.getEvent(&event);
    // unit: m/s^2
    gravity = {
        event.acceleration.x,
        event.acceleration.y,
        event.acceleration.z
    };
}

void readSensor(unsigned long elapsedMs);


void setup() {
    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, BGR>(leds, N_LEDS);
    FastLED.setBrightness(MAX_BRIGHTNESS);

    Serial.begin(115200);
    Serial.println("Starting Glücksrad...");

    displaySensorDetails();
    initGravity();

    _time = micros();
}

void loop() {
    elapsedMs = micros() - _time;

    readSensor(elapsedMs);

    runLight(elapsedMs);

    // basic
    set(currentHeight, isCurrentlyRight, currentColor);

    FastLED.show();
    delay(FRAME_DELAY);

    fade();

    _time += elapsedMs;
}

int currentSign = 1;

void runLight(unsigned long elapsedMs) {

    switchSideCountMs += elapsedMs;
    if (switchSideCountMs > switchSideEveryMs) {
        switchSideCountMs -= switchSideEveryMs;

        isCurrentlyRight = !isCurrentlyRight;
    }

    bool movementRegistered = false;
    if (useAccel && !movementRegistered) {
        return;
    }

    increaseCountMs += elapsedMs;
    if (increaseCountMs > increaseLightEveryMs) {
        increaseCountMs -= increaseLightEveryMs;

        currentHeight += currentSign;
        if (currentHeight == N_LEDS_LEFT - 1 || currentHeight == 0) {
            currentSign *= -1;
        }
    }
}

float fadeFactor = 0.45;

void fade() {
    for (int i = 0; i < N_LEDS; i++) {
        CRGB current =  leds[i];
        current.r = static_cast<uint8_t>(current.r * fadeFactor);
        current.g = static_cast<uint8_t>(current.g * fadeFactor);
        current.b = static_cast<uint8_t>(current.b * fadeFactor);
        leds[i] = current;
    }
}

void readSensor(unsigned long elapsedMs) {
    if (!useAccel) {
        return;
    }

    readSensorCountMs += elapsedMs;
    if (readSensorCountMs < readSensorCountEveryMs) {
        return;
    }
    readSensorCountMs -= readSensorCountEveryMs;

    accel.getEvent(&event);

    currentAcceleration = {
        event.acceleration.x - gravity[0],
        event.acceleration.y - gravity[1],
        event.acceleration.z - gravity[2]
    };

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(currentAcceleration[0]); Serial.print("  ");
    Serial.print("Y: "); Serial.print(currentAcceleration[1]); Serial.print("  ");
    Serial.print("Z: "); Serial.print(currentAcceleration[2]); Serial.print("  ");Serial.println("m/s^2 ");
}
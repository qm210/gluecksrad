#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <cmath>

#include <WiFi.h>
#include <WebServer.h>

#define DATA_PIN 14
#define CLOCK_PIN 27

#define N_LEDS_RIGHT 9
#define N_LEDS_LEFT 9
#define N_LEDS (N_LEDS_RIGHT + N_LEDS_LEFT)

#define MAX_BRIGHTNESS 35
#define FRAME_DELAY 50

CRGB leds[N_LEDS];

void runLight(unsigned long elapsedMs);
void mightReset(unsigned long elapsedMs);
void fade();

float currentHeight = 0;
float maxHeight = float(N_LEDS - 1);
float oldHeight = -1.;

using Vec3 = std::array<float, 3>;
Vec3 gravity = {0.f, 0.f, 0.f};
Vec3 currentAcceleration = {0.f, 0.f, 0.f};
float currentAccelerationMagnitude = 0.;
Vec3 lastAcceleration = {};
float jerkMagnitude = 0.;
// "jerk" is the real name of the derivative of the acceleration, i.e. da/dt

// do not react if jerk is below...
float jerkThreshold = 0.08;
// if jerk is above threshold, multiply this for the point speed
float jerkFactor = 23.;

// just some debug / feedback values
float measureMaxJerk = 0.;
float maxAccelerationEver = 0.;

float fadeFactor = 0.66;

uint8_t currentHue;
uint8_t startHue = 210;
float hueFactor = 160;
CRGB currentColor;

bool useAccel = false;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#include "network.h";
WebServer server(80);
#include "html.h";
void MainPage() {
    String _html_page = html_page;
    server.send(200, "text/html", _html_page);
}

void JerkPage() {
    String data = "[\"" + String(X) + "\",\"" + String(Y) + "\",\"" + String(Z) + "\"]";
    server.send(200, "text/plane", data);
}

unsigned long _time;
unsigned long elapsedMs;

constexpr unsigned long increaseLightEveryMs = 50;
constexpr unsigned long readSensorCountEveryMs = 200;
constexpr unsigned long resetAfterConsecutiveNoJerkMs = 500;

unsigned long increaseCountMs = 0;
unsigned long readSensorCountMs = 0;
unsigned long resetCountMs = 0;

CRGB dimmed(CRGB color, float factor) {
    auto r = static_cast<uint8_t>(factor * color.r);
    auto g = static_cast<uint8_t>(factor * color.g);
    auto b = static_cast<uint8_t>(factor * color.b);
    return CRGB(r, g, b);
}

CRGB dimmed(CRGB color, float factor, uint8_t threshold) {
    auto result = dimmed(color, factor);
    if (result.r < threshold) {
        result.r = 0;
    }
    if (result.g < threshold) {
        result.g = 0;
    }
    if (result.b < threshold) {
        result.b = 0;
    }

    return result;
}

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

void setPoint(float height, CRGB target) {
    float integerHeight;
    float fractionalHeight = modf(height, &integerHeight);
    int heightIndex = int(integerHeight);
    bool isRightSide = heightIndex % 2 == 0;
    int i_0, i_1;
    if (isRightSide) {
        i_0 = indexRight(heightIndex / 2);
        i_1 = indexLeft(heightIndex / 2);
    } else {
        i_0 = indexLeft(heightIndex / 2);
        i_1 = indexRight(heightIndex / 2 + 1);
    }

    // some weighting
    fractionalHeight *= fractionalHeight;
    leds[i_0] = dimmed(target, 1. - fractionalHeight);
    leds[i_1] = dimmed(target, fractionalHeight);
}

void setHeight(float height, CRGB target) {
    if (height == oldHeight) {
        return;
    }

    setPoint(height, target);

    // new idea: no gaps for high speeds
    float span = height - oldHeight;
    if (abs(span > 1)) {
        int sign = height > oldHeight ? +1 : -1;
        for (int h = ceil(oldHeight); h <= floor(height); h += sign) {
            setPoint(float(h), target);
        }
    }

    oldHeight = height;
}

void displaySensorDetails(void)
{
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

    if(!accel.begin())
    {
        Serial.println("Could not intialize the ADXL345 - sorrie!");
        return;
    }

    useAccel = true;
    delay(500);
}

void initGravity() {
    if (!useAccel) {
        return;
    }
    sensors_event_t event;
    accel.getEvent(&event);
    // unit: m/s^2
    gravity[0] = event.acceleration.x;
    gravity[1] = event.acceleration.y;
    gravity[2] = event.acceleration.z;

    Serial.print("g: "); Serial.print(gravity[0]); Serial.print("  ");
    Serial.print(gravity[1]); Serial.print("  ");
    Serial.print(gravity[2]); Serial.print("  ");Serial.println("m/s^2 ");
}

void readSensor(unsigned long elapsedMs);

void setupServer() {
    server.on('/', MainPage);
    server.on('/jerk', JerkPage);
    server.begin();
}

bool setupWiFiConnection() {
    // lelel, cf. https://www.electronicwings.com/esp32/adxl345-accelerometer-interfacing-with-esp32
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PW);

    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
    delay(1000);

    int abort_after_attempts = 50;
    while (abort_after_attempts-- > 0) {
        if (WiFi.waitForConnectResult() == WL_CONNECTED) {
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
            return true;
        }
        Serial.println(".");
    }
    Serial.println("Timeout");
    return false;
}

void setup() {
    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, BGR>(leds, N_LEDS);
    FastLED.setBrightness(MAX_BRIGHTNESS);

    Serial.begin(115200);
    Serial.println("Starting QM's Glücksrad...");

    displaySensorDetails();
    initGravity();

    _time = micros();

    currentHue = startHue;

    setupServer();
    setupWiFiConnection();
}

void handleSerialInput() {
    if (!Serial.available()) {
        return;
    }
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "I") {
        Serial.print("Info: ");
        Serial.println(useAccel ? "Uses Acceleration" : "Doesn't use Acceleration");
        Serial.print("height=");
        Serial.println(currentHeight);
    } else {
        Serial.println("-- Unknown Command");
    }
}

void loop() {
    auto now = micros();
    elapsedMs = (now - _time) / 1000;

    server.handleClient();

    handleSerialInput();

    readSensor(elapsedMs);

    runLight(elapsedMs);

    // basic
    currentColor = CHSV(currentHue, 255, 128);
    setHeight(currentHeight, currentColor);

    FastLED.show();
    delay(FRAME_DELAY);

    fade();

    _time = now;
}

int currentSign = 1;

void runLight(unsigned long elapsedMs) {

    float factor = 1.;

    if (useAccel) {
        factor = jerkMagnitude > jerkThreshold
            ? jerkFactor * (jerkMagnitude - jerkThreshold)
            : 0.;
    }

    mightReset(elapsedMs);

    increaseCountMs += elapsedMs;

    if (increaseCountMs > increaseLightEveryMs) {
        increaseCountMs -= increaseLightEveryMs;

        currentHeight += factor * currentSign;
        if (currentHeight >= maxHeight) {
            currentHeight = maxHeight;
            currentSign = -1;
        } else if (currentHeight <= 0) {
            currentHeight = 0.;
            currentSign = +1;
        }

        currentHue = static_cast<uint8_t>(fmod(float(startHue) * exp(hueFactor * jerkMagnitude), 255.));
    }
}

void fade() {
    for (int i = 0; i < N_LEDS; i++) {
        leds[i] = dimmed(leds[i], fadeFactor);
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

    sensors_event_t event;
    accel.getEvent(&event);

    float absoluteAcceleration = sqrt(
        event.acceleration.x * event.acceleration.x +
        event.acceleration.y * event.acceleration.y +
        event.acceleration.z * event.acceleration.z
    );
    if (absoluteAcceleration > maxAccelerationEver) {
        // measured for the whole duration of the uptime, to find out whether we can adjust the range
        maxAccelerationEver = absoluteAcceleration;
    }

    currentAcceleration = {
        event.acceleration.x - gravity[0],
        event.acceleration.y - gravity[1],
        event.acceleration.z - gravity[2]
    };

    float norm = 0.;
    Vec3 jerk;
    float jerkNorm = 0.;
    for (int c = 0; c < 3; c++) {
        norm += currentAcceleration[c] * currentAcceleration[c];
        jerk[c] = (currentAcceleration[c] - lastAcceleration[c]) / elapsedMs;
        jerkNorm += jerk[c] * jerk[c];
    }
    currentAccelerationMagnitude = sqrt(norm);
    jerkMagnitude = sqrt(jerkNorm);

    if (jerkMagnitude > 0.01) {
        Serial.print("X: "); Serial.print(currentAcceleration[0]); Serial.print("  ");
        Serial.print("Y: "); Serial.print(currentAcceleration[1]); Serial.print("  ");
        Serial.print("Z: "); Serial.print(currentAcceleration[2]); Serial.print("  -> ");
        Serial.print(currentAccelerationMagnitude);Serial.print("m/s^2, jerk ");Serial.println(jerkMagnitude);
    }

    if (jerkMagnitude > measureMaxJerk) {
        measureMaxJerk = jerkMagnitude;
    }

    lastAcceleration = currentAcceleration;
}

void mightReset(unsigned long elapsedMs) {
    if (jerkMagnitude > jerkThreshold) {
        resetCountMs = 0;
        return;
    }

    resetCountMs += elapsedMs;
    if (resetCountMs > resetAfterConsecutiveNoJerkMs) {
        currentHeight = 0.;
        currentHue = startHue;

        Serial.print("Jerk Amplitude was [m/s^3]: ");
        Serial.print(measureMaxJerk);
        Serial.print("; this lifetime's max. acceleration [m/s^2]: ");
        Serial.println(maxAccelerationEver);

        measureMaxJerk = 0.;
    }
}

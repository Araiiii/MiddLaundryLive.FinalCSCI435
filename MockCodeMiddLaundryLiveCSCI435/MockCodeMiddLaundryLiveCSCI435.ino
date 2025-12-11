/*Midd.LaundryLive
CSCI 435 Final Project
Sebastian Cruz, Arai Hardy
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

//WiFi & MQTT 
const char* ssid = "Middlebury-IoT";
const char* password = "password"; //Middlebury IoT Password for specific ESP32 board

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_data = "topic/<building>/<machineId>/data";
const char* mqtt_topic_control = "topic/<building>/<machineId>/control";

WiFiClient espClient;
PubSubClient client(espClient);

// Loggin and testing
const int LOG_INTERVAL = 250 ; // every 60000 readings (~every 2min if MS_BETWEEN_READINGS=20) 26_HZ
int logCounter = 0;
// -------------------- WASHER VIBRATION DETECTION ----------------------
// Total time window to decide washer ON/OFF (ms)
const int TOTAL_WINDOW = 30000; //in ms
const int MS_BETWEEN_READINGS = 20; //Coming roughly from the data rate(oversampling) 1/(2frequency) 

// Compute window sizes in terms of readings
const int LONG_WINDOW = TOTAL_WINDOW / MS_BETWEEN_READINGS;
const int SHORT_WINDOW = (TOTAL_WINDOW / 3) / MS_BETWEEN_READINGS;

// Exponential Moving Average alpha
float alpha_long = 1.0 / LONG_WINDOW;
float alpha_short = 1.0 / SHORT_WINDOW;
const float ENERGY_ALPHA = alpha_long; // small alpha = slow smoothing

// Variables for running calculations
float energyEMA = 0;          // Smoothed energy
bool washerOn = false;        // Washer state
bool PrevWasherState = false;        // Washer state

float longBuffer[LONG_WINDOW] = {0};  // Long-term baseline
int longIndex = 0;
float longSum = 0;
float longMean = 0; 

float shortBuffer[SHORT_WINDOW] = {0}; // Short-term RMS of AC
int shortIndex = 0;
float shortSumSquares = 0;

// Thresholds to detect washer ON/OFF
const float ON_THRESHOLD = 0.6;
const float OFF_THRESHOLD = 0.055;

// ------ For polling detection loop ---- 
const float MOVE_THRESHOLD = 0.02;     // RMS threshold
const unsigned long OFF_TIMEOUT = 120000; // 2 minutes in ms
unsigned long lastMovementTime = 0;   // Tracks last time movement was detected

//-----Experiment with Different Alpha values so that the emma rises quickly and decreases slowly
const float ALPHA_UP = 0.3;   // fast increase
const float ALPHA_DOWN = 0.01; // slow decrease



Adafruit_LSM6DSOX sox;

//Enumarate states
typedef enum {
  CALIBRATION, 
  DETECTION, 
  POLLING
} state_e;

state_e state = CALIBRATION; 
state_e next_state = POLLING;

//wifiiii
void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32Laundry-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_control);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void calibration_loop() {
  for (int i = 0; i < TOTAL_WINDOW / MS_BETWEEN_READINGS + 100; i++) {
    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    float mx = accel.acceleration.x;
    float my = accel.acceleration.y;
    float mz = accel.acceleration.z;
    float mag = sqrt(mx * mx + my * my + mz * mz);

    longSum -= longBuffer[longIndex];
    longSum += mag;
    longBuffer[longIndex] = mag;
    longIndex = (longIndex + 1) % (TOTAL_WINDOW / MS_BETWEEN_READINGS);
    longMean = longSum / (TOTAL_WINDOW / MS_BETWEEN_READINGS);

    Serial.print("LongMean: "); 
    Serial.println(longMean);
    delay(MS_BETWEEN_READINGS);
  }
}

void detection_loop() {
  //----- Connect to client -------
  while (1) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();

    // ------ get sensor data ------- 
    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    // Compute magnitude of acceleration
    float mx = accel.acceleration.x;
    float my = accel.acceleration.y;
    float mz = accel.acceleration.z;
    float mag = sqrt(mx * mx + my * my + mz * mz);

    // Remove gravity + orientation
    float acc = mag - longMean;

    // ------------------ Short-window RMS of AC ------------------
    float oldSq = shortBuffer[shortIndex];
    shortSumSquares -= oldSq;
    float newSq = (fabs(acc) > 0.01) ? fabs(acc) : 0;
    shortSumSquares += newSq;
    shortBuffer[shortIndex] = newSq;
    shortIndex = (shortIndex + 1) % ((TOTAL_WINDOW / 3) / MS_BETWEEN_READINGS);

    float rms = shortSumSquares;
    //ORIGINAL:energyEMA = (1.0 / ((TOTAL_WINDOW / MS_BETWEEN_READINGS))) * rms + (1.0 - (1.0 / ((TOTAL_WINDOW / MS_BETWEEN_READINGS)))) * energyEMA;

    //Emma Will increase faster, and decrease slower. 
      if (rms > energyEMA) {
          // Rising → increase fast
         energyEMA = ALPHA_UP * rms + (1 - ALPHA_UP) * energyEMA;
      } else {
          // Falling → decay slowly
          energyEMA = ALPHA_DOWN * rms + (1 - ALPHA_DOWN) * energyEMA;
      }

    // ------------------ Washer state detection ------------------
    if (!washerOn && energyEMA >= ON_THRESHOLD) {
      washerOn = true;
    }
    if (washerOn && energyEMA <= OFF_THRESHOLD) {
      washerOn = false;
    }

    // ------------------ Logging ------------------
    logCounter++;
    if (logCounter >= LOG_INTERVAL) {
      logCounter = 0;
      // Publish JSON to MQTT
      String payload = "{\"washerState\":\"" + String(washerOn ? "on" : "off") + "\"}";
      
      client.publish(mqtt_topic_data, payload.c_str());
      //     String debug = "{\"acc\":" + String(acc, 3) +
      //             ",\"rms\":" + String(rms, 3) +
      //             ",\"ema\":" + String(energyEMA, 3) +
      //             ",\"washerState\":\"" + (washerOn ? "on" : "off") + "\"}";
      // Serial.println(debug);
    }

    // String payload = "{\"acc\":" + String(acc, 3) +
    //               ",\"rms\":" + String(rms, 3) +
    //               ",\"ema\":" + String(energyEMA, 3) +
    //               ",\"washerState\":\"" + (washerOn ? "on" : "off") + "\"}";


    delay(MS_BETWEEN_READINGS);
  }
}

void detection_loop2Polling() {
  //----- Connect to client -------
  while (1) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();

    // ------ get sensor data ------- 
    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    // Compute magnitude of acceleration
    float mx = accel.acceleration.x;
    float my = accel.acceleration.y;
    float mz = accel.acceleration.z;
    float mag = sqrt(mx * mx + my * my + mz * mz);

    // Remove gravity + orientation
    float acc = mag - longMean;

    // ------------------ Short-window RMS of AC ------------------
    float oldSq = shortBuffer[shortIndex];
    shortSumSquares -= oldSq;
    float newSq = (fabs(acc) > 0.01) ? fabs(acc) : 0;
    shortSumSquares += newSq;
    shortBuffer[shortIndex] = newSq;
    shortIndex = (shortIndex + 1) % ((TOTAL_WINDOW / 3) / MS_BETWEEN_READINGS);

    float rms = shortSumSquares;
    //energyEMA = (1.0 / ((TOTAL_WINDOW / MS_BETWEEN_READINGS))) * rms + (1.0 - (1.0 / ((TOTAL_WINDOW / MS_BETWEEN_READINGS)))) * energyEMA;

    // ------------------ Washer state detection (RMS-only) ------------------
    unsigned long now = millis();

    if (rms > MOVE_THRESHOLD) {
        // Movement detected -> washer is ON
        washerOn = true;
        lastMovementTime = now;   // reset timer because movement occurred
    } 
    else {
        // No strong movement. If 2 minutes pass since last movement -> OFF
        if (washerOn && (now - lastMovementTime > OFF_TIMEOUT)) {
            washerOn = false;
        }
    }

    // ------------------ Logging ------------------
    logCounter++;
    if (logCounter >= LOG_INTERVAL) {
      logCounter = 0;
      // Publish JSON to MQTT
      String payload = "{\"washerState\":\"" + String(washerOn ? "on" : "off") + "\"}";
      client.publish(mqtt_topic_data, payload.c_str());
      String debug = "{\"acc\":" + String(acc, 3) +
                  ",\"rms\":" + String(rms, 3) +
                  ",\"ema\":" + String(energyEMA, 3) +
                  ",\"washerState\":\"" + (washerOn ? "on" : "off") + "\"}";
      Serial.println(debug);
    }


    delay(MS_BETWEEN_READINGS);
  }
}






void setup() {
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, LOW);

  // WiFi
  connectToWiFi();

  // MQTT
  client.setServer(mqtt_server, mqtt_port);

    if (!sox.begin_I2C()) {
        // For SPI: sox.begin_SPI(LSM_CS);
        // For software SPI: sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI);
        Serial.println("Failed to find LSM6DSOX chip");
        while (1) { 
            delay(10); }
    }


    //Connecting to accelerometer
    Serial.println("LSM6DSOX Found!");
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    sox.setAccelDataRate(LSM6DS_RATE_26_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);

    Serial.print("Accelerometer data rate set to: ");
    switch (sox.getAccelDataRate()) {
        case LSM6DS_RATE_SHUTDOWN: Serial.println("0 Hz"); break;
        case LSM6DS_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
        case LSM6DS_RATE_26_HZ: Serial.println("26 Hz"); break;
        case LSM6DS_RATE_52_HZ: Serial.println("52 Hz"); break;
        case LSM6DS_RATE_104_HZ: Serial.println("104 Hz"); break;
        case LSM6DS_RATE_208_HZ: Serial.println("208 Hz"); break;
        case LSM6DS_RATE_416_HZ: Serial.println("416 Hz"); break;
        case LSM6DS_RATE_833_HZ: Serial.println("833 Hz"); break;
        case LSM6DS_RATE_1_66K_HZ: Serial.println("1.66 KHz"); break;
        case LSM6DS_RATE_3_33K_HZ: Serial.println("3.33 KHz"); break;
        case LSM6DS_RATE_6_66K_HZ: Serial.println("6.66 KHz"); break;
    }

  // Computing for EMA
  // initialized globally
}

void loop() {
  switch (state) {
    case CALIBRATION:
      calibration_loop();
      state = next_state;
      break;

    case DETECTION:
      detection_loop();
      break;

    case POLLING:
      detection_loop2Polling();
      break;
  }
}


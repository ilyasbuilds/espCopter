#include "Arduino.h"
#include "Drone.h"
#include "Imu.h"
#include "MotorController.h"
#include "Receiver.h"
#include "config.h"
#include "esp_timer.h"
#include <ESP32Servo.h>

#if USE_WEB
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

// Forward declarations
String formatDroneData();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length);

// WebSocket setup
const char *ssid = "Bingus Copter";
const char *password = "12345678";
WebSocketsServer webSocket(9090);
StaticJsonDocument<1024> doc;
int64_t lastUpdate = 0;
const int64_t updateInterval = 33000; // ~30Hz (in microseconds)
#endif

// State Machine
enum State { ArmESC, Idle, StartMission, TESTAREA };

State droneState = ArmESC;
State pastState = ArmESC;

// System components
Receiver *rc;
MotorInterface *flight;
Drone *drone;
int startup = 0;

// Add at the top with other globals
unsigned long loopStartTime;
unsigned long loopEndTime;
unsigned long loopCount = 0;
unsigned long totalTime = 0;

// Add these timing variables at the top with other globals
struct LoopTiming {
  unsigned long webSocketTime;
  unsigned long droneTime;
  unsigned long totalTime;
  unsigned long count;
} timing = {0, 0, 0, 0};

// ---------------------------------------------------------------------------

/**
 * Start serial, attach motors, and display the instructions and format
 */

// Mac Address 67E05EFD-6F57-A683-D854-6B1E472AE099
void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

#if USE_WEB

  // Configure WiFi exactly as in the working test file
  Serial.println("Configuring WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                // Disable power saving
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Max power

  // Start AP
  if (WiFi.softAP(ssid, password, 11, 0,
                  8)) { // channel 1, open network, max 4 connections
    Serial.println("WiFi AP started successfully");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.softAPmacAddress());
  } else {
    Serial.println("WiFi AP failed to start!");
  }

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 9090");
#endif

  rc = new Receiver();
  flight = new PwmMotorController();
  drone = new Drone(flight, rc);

  // Update PID gains
  droneState = ArmESC;
}

#if USE_WEB
// Only compile these functions if web is enabled
String formatDroneData() {
  doc.clear();

  // IMU Array
  JsonArray imu = doc.createNestedArray("IMU");
  imu.add(drone->getRoll());          // degrees
  imu.add(drone->getPitch());         // degrees
  imu.add(drone->getYaw());           // degrees
  imu.add(drone->getRollVelocity());  // degrees/second
  imu.add(drone->getPitchVelocity()); // degrees/second
  imu.add(drone->getYawVelocity());   // degrees/second
  // Add calibration values
  imu.add(drone->getSystemCalibration()); // 0-3
  imu.add(drone->getGyroCalibration());   // 0-3
  imu.add(drone->getAccelCalibration());  // 0-3
  imu.add(drone->getMagCalibration());    // 0-3

  // PID Output Array
  JsonArray pidOutput = doc.createNestedArray("PIDOutput");
  pidOutput.add(drone->getRollPIDOutput());  // -100 to 100
  pidOutput.add(drone->getPitchPIDOutput()); // -100 to 100
  pidOutput.add(drone->getYawPIDOutput());   // -100 to 100

  // Setpoints Array
  JsonArray setpoints = doc.createNestedArray("Setpoints");
  setpoints.add(drone->getRollSetpoint());  // degrees
  setpoints.add(drone->getPitchSetpoint()); // degrees
  setpoints.add(drone->getYawSetpoint());   // degrees

  // PID Gains Array
  JsonArray pidGains = doc.createNestedArray("PIDGains");
  double gains[9];
  drone->getGains(gains); // Assuming this method exists or needs to be added
  for (int i = 0; i < 9; i++) {
    pidGains.add(gains[i]);
  }

  // Receiver Values Array (normalized to 0-100%)
  JsonArray receiverValues = doc.createNestedArray("ReceiverValues");
  Receiver::values values = rc->getData();
  // map values to pulse width for compatability
  auto to_pulse_width = [](double value) {
    return static_cast<unsigned long>(value * 500 + 1500);
  };
  receiverValues.add(to_pulse_width(values.yaw)); // CH1 / yaw
  receiverValues.add(
      to_pulse_width(values.throttle * 2.0 - 1.0));  // CH2 / throttle
  receiverValues.add(to_pulse_width(values.roll));   // CH3 / roll
  receiverValues.add(to_pulse_width(values.pitch));  // CH4 / pitch
  receiverValues.add(to_pulse_width(values.aux[0])); // CH5 / aux 0
  receiverValues.add(to_pulse_width(values.aux[1])); // CH6 / aux 1

  // Motor Values Array (normalized to 0-100%)
  JsonArray motorValues = doc.createNestedArray("MotorValues");
  MotorInterface::MotorCommand command = drone->getMotorValues();
  motorValues.add(command.front_left);
  motorValues.add(command.front_right);
  motorValues.add(command.back_left);
  motorValues.add(command.back_right);

  // Battery Life (you'll need to implement battery monitoring)
  doc["BatteryLife"] =
      100; // Placeholder value, implement actual battery monitoring

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2],
                  ip[3]);

    // Send initial data immediately upon connection
    String jsonString = formatDroneData();
    webSocket.sendTXT(num, jsonString);
  } break;

  case WStype_TEXT:
    Serial.printf("[%u] Received text: %s\n", num, payload);
    // Echo back the received message
    webSocket.sendTXT(num, payload, length);

    {
      // Parse incoming JSON message
      String payloadStr = String((char *)payload);

      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }

      // Check if this is a PID update message
      if (doc.containsKey("gains")) {
        JsonArray pidValues = doc["gains"];
        if (pidValues.size() == 9) {
          double gains[9];
          for (int i = 0; i < 9; i++) {
            gains[i] = pidValues[i];
          }
          drone->updateGain(gains);
          Serial.println("gains");
          drone->printGains();
        }
      }
    }
    break;

  case WStype_BIN:
    Serial.printf("[%u] Received binary length: %u\n", num, length);
    break;

  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
    break;
  }
}
#endif

/**
 * Loop: Read input and execute instruction
 */
void loop() {
  unsigned long loopStart = micros();
  unsigned long stepStart;

#if USE_WEB
  stepStart = micros();
  webSocket.loop();

  // Print WiFi status every 5 seconds
  static int64_t lastWifiStatusUpdate = 0;
  if (esp_timer_get_time() - lastWifiStatusUpdate >= 5000000 &&
      LOG_WIFI) { // 5 seconds in microseconds
    Serial.println("\n=== WiFi Status ===");
    Serial.print("Connected clients: ");
    Serial.println(WiFi.softAPgetStationNum());
    Serial.print("TX Power: ");
    Serial.println(WiFi.getTxPower());
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
    lastWifiStatusUpdate = esp_timer_get_time();
  }

  // Only send data if clients are connected
  if (esp_timer_get_time() - lastUpdate >= updateInterval &&
      WiFi.softAPgetStationNum() > 0) {
    String jsonString = formatDroneData();
    webSocket.broadcastTXT(jsonString);
    lastUpdate = esp_timer_get_time();
  }
  timing.webSocketTime += (micros() - stepStart);
#endif

  switch (droneState) {
  case (ArmESC):
    Serial.println("ARM ESC");
    // Start up ESC's
    if (!flight->armControllers()) {
      Serial.println("Arming failed, retrying");
      delay(2000);
      break;
    } 
    Serial.println("Setup completed");
    droneState = StartMission;
    break;
  case (StartMission):
    stepStart = micros();
    drone->loop();
    timing.droneTime += (micros() - stepStart);

    timing.totalTime += (micros() - loopStart);
    timing.count++;

    // Print timing breakdown every 1000 loops
    if (timing.count >= 1000 && LOG_TIMING) {
      Serial.println("\n=== Loop Timing Breakdown ===");
      Serial.print("WebSocket avg (us): ");
      Serial.println(timing.webSocketTime / timing.count);
      Serial.print("Drone avg (us): ");
      Serial.println(timing.droneTime / timing.count);
      Serial.print("Total avg (us): ");
      Serial.println(timing.totalTime / timing.count);
      Serial.print("Frequency (Hz): ");
      Serial.println(1000000.0 / (timing.totalTime / timing.count));
      Serial.println("===========================\n");

      // Reset counters
      timing = {0, 0, 0, 0};
    }
    break;
  }
}
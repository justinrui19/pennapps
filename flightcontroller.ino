/* MinimalESP32_FC.ino
   Minimal flight-controller template for ESP32 + ICM-20948
   - Attitude estimation: Mahony AHRS
   - Stabilization: PID on roll & pitch (angle PID -> rate PID architecture simplified)
   - Motor outputs via LEDC (servo-style PWM)
   - Arming + failsafe + simple Serial command RC

   WARNING: experimental. Test without props. Tune on bench.
*/

// ====== includes & library note ======
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <Preferences.h>
#include "ICM_20948.h"         // SparkFun / commonly-named ICM-20948 library
// Install via Library Manager or replace with your preferred ICM-20948 driver

// ====== HW config ======
const int SDA_PIN = 21;   // set to your actual SDA pin (ESP32-CAM pin mapping!)
const int SCL_PIN = 22;   // set to your actual SCL pin
ICM_20948_I2C imu;        // object for SparkFun-style lib

// ESC / motor pins (change to pins that work on your board)
const int ESC_PIN[4] = {17, 16, 4, 2}; // motor0..motor3
const int LEDC_CH[4]  = {0,1,2,3};
const int LEDC_TIMER  = 0;
const int PWM_FREQ = 50;   // 50Hz standard servo pulses for ESCs
const int PWM_RES  = 16;   // 16-bit resolution

const int THRUST_MIN_US = 1000;
const int THRUST_MAX_US = 2000;

// ====== control loop & timing ======
const float LOOP_HZ = 200.0;    // control loop frequency (Hz) — try 200
const float dt = 1.0 / LOOP_HZ;
unsigned long lastLoopMicros = 0;

// ====== Mahony AHRS state ======
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
const float twoKp = 2.0f * 1.0f; // proportional gain
const float twoKi = 2.0f * 0.0f; // integral gain
float integralFBx = 0, integralFBy = 0, integralFBz = 0;

// ====== IMU scale (set according to your library / units) ======
float gyroScale = 1.0f; // library returns deg/s ideally; adapt if needed

// ====== simple PID structures ======
struct PID {
  float kp, ki, kd;
  float integrator;
  float lastError;
  float integratorLimit;
  PID(): kp(0), ki(0), kd(0), integrator(0), lastError(0), integratorLimit(1000) {}
  float update(float error, float dt) {
    integrator += error * dt;
    if (integrator > integratorLimit) integrator = integratorLimit;
    if (integrator < -integratorLimit) integrator = -integratorLimit;
    float derivative = (error - lastError) / dt;
    lastError = error;
    return kp*error + ki*integrator + kd*derivative;
  }
};

PID pidRollAngle, pidPitchAngle; // angle -> rate
// Simple rate P for inner loop (we'll approximate with gyro feedback)
float rateP = 0.06f;

// ====== desired setpoints and rc ======
volatile float rcThrottle = 1000; // microseconds
volatile float rcRollDeg = 0.0f;  // desired roll angle (deg)
volatile float rcPitchDeg = 0.0f; // desired pitch angle (deg)
volatile float rcYawRate = 0.0f;  // desired yaw rate (deg/s) - placeholder

// ====== arming & failsafe ======
bool armed = false;
unsigned long lastRCcommand = 0;
const unsigned long RC_TIMEOUT_MS = 500; // disarm if no RC update

// ====== helpers ======
uint32_t usToDuty(uint32_t us) {
  float period_us = 1000000.0f / PWM_FREQ;
  uint32_t maxDuty = (1 << PWM_RES) - 1;
  float duty = ((float)us / period_us) * (float)maxDuty;
  if (duty < 0) duty = 0;
  if (duty > maxDuty) duty = maxDuty;
  return (uint32_t) duty;
}

void escWriteMicroseconds(int channel, uint32_t us) {
  uint32_t duty = usToDuty(us);
  ledcWrite(channel, duty);
}

// ====== Mahony AHRS update (compact) ======
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float sampleFreq) {
  // gx,gy,gz in deg/s from IMU; convert to rad/s inside
  float recipNorm;
  float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
  float gx_rad = gx * (PI/180.0f);
  float gy_rad = gy * (PI/180.0f);
  float gz_rad = gz * (PI/180.0f);

  // Normalise accelerometer measurement
  recipNorm = sqrt(ax*ax + ay*ay + az*az);
  if (recipNorm == 0.0f) return;
  recipNorm = 1.0f / recipNorm;
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  // Estimated direction of gravity
  float vx = 2*(q1*q3 - q0*q2);
  float vy = 2*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Error = cross product between estimated and measured direction of gravity
  float ex = (ay*vz - az*vy);
  float ey = (az*vx - ax*vz);
  float ez = (ax*vy - ay*vx);

  if (twoKi > 0.0f) {
    integralFBx += twoKi * ex * (1.0f/sampleFreq);
    integralFBy += twoKi * ey * (1.0f/sampleFreq);
    integralFBz += twoKi * ez * (1.0f/sampleFreq);
    gx_rad += integralFBx;
    gy_rad += integralFBy;
    gz_rad += integralFBz;
  } else {
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  // Apply proportional feedback
  gx_rad += twoKp * ex;
  gy_rad += twoKp * ey;
  gz_rad += twoKp * ez;

  // Integrate rate of change of quaternion
  float qa = q0;
  float qb = q1;
  float qc = q2;
  float qd = q3;
  float halfDt = 0.5f * (1.0f/sampleFreq);
  q0 += (-qb * gx_rad - qc * gy_rad - qd * gz_rad) * halfDt;
  q1 += ( qa * gx_rad + qc * gz_rad - qd * gy_rad) * halfDt;
  q2 += ( qa * gy_rad - qb * gz_rad + qd * gx_rad) * halfDt;
  q3 += ( qa * gz_rad + qb * gy_rad - qc * gx_rad) * halfDt;

  // Normalize quaternion
  recipNorm = 1.0f / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// Convert quaternion to Euler angles (deg)
void quatToEuler(float &roll, float &pitch, float &yaw) {
  roll  = atan2(2.0f*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180.0f / PI;
  pitch = asin( 2.0f*(q0*q2 - q3*q1)) * 180.0f / PI;
  yaw   = atan2(2.0f*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0f / PI;
}

// ====== WiFi / Web / OTA ======
Preferences prefs;
WebServer web(80);
HTTPUpdateServer httpUpdater;

String wifiSSID = "";
String wifiPASS = "";
bool wifiConnected = false;

void handleRoot() {
  web.send(200, "text/html",
    "<html><body>ESP32 FC Online<br/>"
    "<a href=/wifi>WiFi Config</a> | <a href=/update>OTA Update</a> | <a href=/api/status>API Status</a>"
    "</body></html>");
}

void handleWifiPage() {
  String html = "<html><body><h3>WiFi Config</h3><form method=post action=/wifi>";
  html += "SSID: <input name=ssid value='" + wifiSSID + "'/><br/>";
  html += "PASS: <input name=pass type=password value='" + wifiPASS + "'/><br/>";
  html += "<button type=submit>Save & Reboot</button></form></body></html>";
  web.send(200, "text/html", html);
}

void handleWifiPost() {
  if (web.hasArg("ssid")) wifiSSID = web.arg("ssid");
  if (web.hasArg("pass")) wifiPASS = web.arg("pass");
  prefs.begin("net", false);
  prefs.putString("ssid", wifiSSID);
  prefs.putString("pass", wifiPASS);
  prefs.end();
  web.send(200, "text/plain", "Saved. Rebooting...");
  delay(500);
  ESP.restart();
}

void handleStatus() {
  char buf[256];
  float roll, pitch, yaw; quatToEuler(roll, pitch, yaw);
  snprintf(buf, sizeof(buf), "{\"armed\":%s,\"rc\":{\"T\":%d,\"R\":%.1f,\"P\":%.1f,\"Y\":%.1f},\"att\":{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}}",
           armed?"true":"false", (int)rcThrottle, rcRollDeg, rcPitchDeg, rcYawRate, roll, pitch, yaw);
  web.send(200, "application/json", buf);
}

void handlePing() {
  web.send(200, "text/plain", "ok");
}

void handleArm() { armed = true; web.send(200, "text/plain", "ARMED"); }
void handleDisarm() { armed = false; web.send(200, "text/plain", "DISARMED"); }

void handleRC() {
  if (web.hasArg("T")) rcThrottle = web.arg("T").toInt();
  if (web.hasArg("R")) rcRollDeg = web.arg("R").toFloat();
  if (web.hasArg("P")) rcPitchDeg = web.arg("P").toFloat();
  if (web.hasArg("Y")) rcYawRate = web.arg("Y").toFloat();
  lastRCcommand = millis();
  web.send(200, "text/plain", "RC OK");
}

void setupWiFiAndHTTP() {
  prefs.begin("net", true);
  wifiSSID = prefs.getString("ssid", "");
  wifiPASS = prefs.getString("pass", "");
  prefs.end();

  if (wifiSSID.length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
      delay(200);
    }
    wifiConnected = (WiFi.status() == WL_CONNECTED);
  }

  if (!wifiConnected) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Rescuer-Setup", "rescue123");
  }

  httpUpdater.setup(&web);
  web.on("/", handleRoot);
  web.on("/wifi", HTTP_GET, handleWifiPage);
  web.on("/wifi", HTTP_POST, handleWifiPost);
  web.on("/api/status", handleStatus);
  web.on("/api/ping", handlePing);
  web.on("/api/arm", handleArm);
  web.on("/api/disarm", handleDisarm);
  web.on("/api/rc", handleRC);
  web.begin();
}

// ====== setup ======
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  setupWiFiAndHTTP();

  // Init IMU
  if (imu.begin() != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed!");
    // don't hard fail; continue for debugging but nothing will stabilize
  } else {
    Serial.println("IMU ok");
  }

  // LEDC setup for ESC outputs
  ledcSetup(LEDC_TIMER, PWM_FREQ, PWM_RES);
  for (int i=0;i<4;i++) {
    ledcAttachPin(ESC_PIN[i], LEDC_CH[i]);
    escWriteMicroseconds(LEDC_CH[i], THRUST_MIN_US); // set to min
  }

  // PID initial gains (start small)
  pidRollAngle.kp = 4.0f; pidRollAngle.ki = 0.1f; pidRollAngle.kd = 0.05f;
  pidPitchAngle.kp = 4.0f; pidPitchAngle.ki = 0.1f; pidPitchAngle.kd = 0.05f;
  pidRollAngle.integratorLimit = 50;
  pidPitchAngle.integratorLimit = 50;

  lastLoopMicros = micros();
  lastRCcommand = millis();
  Serial.println("FC setup complete. Send 'ARM' to arm, 'DIS' to disarm. RC packets: T:throttle R:roll P:pitch Y:yawRate (e.g. T1500 R0 P0 Y0)");
}

// ====== simple serial RC parser (non-blocking) ======
void parseSerialCommands() {
  // Example commands:
  // ARM
  // DIS
  // T1500 R10 P-5 Y20  (throttle in us, roll/pitch in degrees, yaw rate deg/s)
  static String sb = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (sb.length() > 0) {
        // process
        if (sb.equalsIgnoreCase("ARM")) {
          armed = true; Serial.println("ARMED");
        } else if (sb.equalsIgnoreCase("DIS")) {
          armed = false; Serial.println("DISARMED");
        } else {
          // parse tokens
          int idx;
          idx = sb.indexOf('T'); if (idx>=0) { int end=sb.indexOf(' ', idx); String token = (end>idx) ? sb.substring(idx+1,end) : sb.substring(idx+1); rcThrottle = token.toInt(); }
          idx = sb.indexOf('R'); if (idx>=0) { int end=sb.indexOf(' ', idx); String token = (end>idx) ? sb.substring(idx+1,end) : sb.substring(idx+1); rcRollDeg = token.toFloat(); }
          idx = sb.indexOf('P'); if (idx>=0) { int end=sb.indexOf(' ', idx); String token = (end>idx) ? sb.substring(idx+1,end) : sb.substring(idx+1); rcPitchDeg = token.toFloat(); }
          idx = sb.indexOf('Y'); if (idx>=0) { int end=sb.indexOf(' ', idx); String token = (end>idx) ? sb.substring(idx+1,end) : sb.substring(idx+1); rcYawRate = token.toFloat(); }
          lastRCcommand = millis();
        }
        sb = "";
      }
    } else {
      sb += c;
      if (sb.length() > 200) sb = sb.substring(sb.length()-200); // bound size
    }
  }
}

// ====== main loop ======
void loop() {
  parseSerialCommands();
  web.handleClient();

  // failsafe: if no RC for timeout, disarm & stop motors
  if (millis() - lastRCcommand > RC_TIMEOUT_MS) {
    if (armed) {
      armed = false;
      Serial.println("RC timeout -> disarm");
    }
  }

  // timing control loop
  unsigned long nowMicros = micros();
  if ((nowMicros - lastLoopMicros) < (unsigned long)(dt*1e6)) {
    // allow other processing
    delay(0);
    return;
  }
  lastLoopMicros = nowMicros;

  // read IMU (replace with actual reading flow for your library)
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (imu.status != ICM_20948_Stat_Ok) {
    // try to recover or skip if IMU not ready
  } else {
    // read sensor; SparkFun lib: imu.getAGMT(); then access imu.accelX(), imu.gyrX() etc.
    imu.getAGMT(); // update cached values
    ax = (float)imu.accX() / 2048.0f; // library-specific scaling; adapt as necessary
    ay = (float)imu.accY() / 2048.0f;
    az = (float)imu.accZ() / 2048.0f;
    gx = (float)imu.gyrX(); // deg/s ideally; check your library
    gy = (float)imu.gyrY();
    gz = (float)imu.gyrZ();
  }

  // update filter
  MahonyAHRSupdate(gx, gy, gz, ax, ay, az, LOOP_HZ);

  // get euler
  float roll, pitch, yaw;
  quatToEuler(roll, pitch, yaw);

  // control law: angle error -> angle PID -> rate setpoint -> P on gyro to get motor corrections
  float rollError  = rcRollDeg  - roll;   // desired angle - measured
  float pitchError = rcPitchDeg - pitch;

  float rollRateCmd  = pidRollAngle.update(rollError, dt);  // deg/s setpoint (approx)
  float pitchRateCmd = pidPitchAngle.update(pitchError, dt);

  // inner loop: use simple P on gyro to convert rate error to torque command
  float rollRateErr  = rollRateCmd  - gx;
  float pitchRateErr = pitchRateCmd - gy;
  float rollControl  = rateP * rollRateErr;
  float pitchControl = rateP * pitchRateErr;
  float yawControl   = 0.0f; // placeholder - implement yaw rate PID as desired

  // motor mixing for X quad (assume motor mapping):
  // motor0: front-left
  // motor1: front-right
  // motor2: rear-right
  // motor3: rear-left
  // Basic mixing: throttle + / - roll/pitch + yaw
  float throttle_us = rcThrottle;
  if (throttle_us < THRUST_MIN_US) throttle_us = THRUST_MIN_US;
  if (throttle_us > THRUST_MAX_US) throttle_us = THRUST_MAX_US;

  // scale control outputs to microsecond range (tweak scaling gains!)
  float ctrlScale = 150.0f; // scale factor: map control units -> microseconds. Tune this small->large.
  float m0 = throttle_us + (-pitchControl * ctrlScale) + (-rollControl * ctrlScale) + (yawControl * ctrlScale);
  float m1 = throttle_us + (-pitchControl * ctrlScale) + ( rollControl * ctrlScale) - (yawControl * ctrlScale);
  float m2 = throttle_us + ( pitchControl * ctrlScale) + ( rollControl * ctrlScale) + (yawControl * ctrlScale);
  float m3 = throttle_us + ( pitchControl * ctrlScale) + (-rollControl * ctrlScale) - (yawControl * ctrlScale);

  // clamp and write to ESC channels
  float motors[4] = {m0,m1,m2,m3};
  for (int i=0;i<4;i++) {
    int us = (int)motors[i];
    if (us < THRUST_MIN_US) us = THRUST_MIN_US;
    if (us > THRUST_MAX_US) us = THRUST_MAX_US;
    if (!armed) us = THRUST_MIN_US; // ensure motors at min when not armed
    escWriteMicroseconds(LEDC_CH[i], us);
  }

  // telemetry (low rate)
  static unsigned long lastTele = 0;
  if (millis() - lastTele > 200) {
    Serial.printf("ARM:%d T:%d Rcmd:%.1f Pcmd:%.1f AT:%.1f PT:%.1f Y:%.1f\n",
                  armed, (int)rcThrottle, rcRollDeg, rcPitchDeg, roll, pitch, yaw);
    lastTele = millis();
  }
}

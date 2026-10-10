#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 2


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET 7

// logの表示の有無
const bool log_accel = false;
const bool log_gyro = false;

volatile bool new_data_flug = false;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


sh2_SensorId_t reportType_accel = SH2_ACCELEROMETER;
long reportIntervalUs_accel = 5000;

sh2_SensorId_t reportType_gyro = SH2_GYROSCOPE_CALIBRATED;
long reportIntervalUs_gyro = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void interruptHandler(void) {
  bno08x.setInterruptTimestamp(micros());
  //Serial.println((unsigned long long)micros());
  new_data_flug = true;
}

void setup(void) {

  Serial.begin(1000000);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("BNO085 demo");

  // Try to initialize!
  //if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType_accel, reportIntervalUs_accel);
  setReports(reportType_gyro, reportIntervalUs_gyro);

  // int pin: active low 
  attachInterrupt(digitalPinToInterrupt(BNO08X_INT), interruptHandler, FALLING);

  Serial.println("Reading events");
  delay(100);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType_accel, reportIntervalUs_accel);
    setReports(reportType_gyro, reportIntervalUs_gyro);
  }

  if (!new_data_flug) {
    return;
  }
  new_data_flug = false;
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)

    if (sensorValue.sensorId == reportType_accel && log_accel) {
      Serial.println((unsigned long long)sensorValue.timestamp);
      /*
      static long last = 0;
      long now = micros();
      Serial.print(now - last);             Serial.print("\t");
      last = now;
      Serial.print("seq:");
      Serial.print(sensorValue.sequence); Serial.print("\t");
      Serial.print("timestamp_us:");
      Serial.print((unsigned long long)sensorValue.timestamp); Serial.print("\t");
      Serial.print("accel:");
      Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
      Serial.print(sensorValue.un.accelerometer.x);                Serial.print("\t");
      Serial.print(sensorValue.un.accelerometer.y);              Serial.print("\t");
      Serial.println(sensorValue.un.accelerometer.z);
      */
    }
    else if (sensorValue.sensorId == reportType_gyro && log_gyro) {
      Serial.println((unsigned long long)sensorValue.timestamp);
      /*
      static long last = 0;
      long now = micros();
      Serial.print(now - last);             Serial.print("\t");
      last = now;
      Serial.print("seq:");
      Serial.print(sensorValue.sequence); Serial.print("\t");
      Serial.print("timestamp_us:");
      Serial.println((unsigned long long)sensorValue.timestamp); Serial.print("\t");
      /*
      Serial.print("gyro:");
      Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
      Serial.print(sensorValue.un.gyroscope.x);                Serial.print("\t");
      Serial.print(sensorValue.un.gyroscope.y);              Serial.print("\t");
      Serial.println(sensorValue.un.gyroscope.z);
      */
    }
  }
}


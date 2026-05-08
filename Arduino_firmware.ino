#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// IMU scale factors
#define ACCEL_SCALE 8192.0f   // LSB per g
#define GYRO_SCALE  16.4f     // LSB per dps

// BLE service and characteristic UUIDs (must match the host script)
#define SERVICE_UUID  "12345678-1234-5678-1234-56789abcdef0"
#define DATA_UUID     "12345678-1234-5678-1234-56789abcdef1"
#define CONTROL_UUID  "12345678-1234-5678-1234-56789abcdef2"

// Packet layout: [timestamp_us : uint32][ax,ay,az,gx,gy,gz : int16] = 16 bytes
#define PACKET_SIZE 16

BLEService runService(SERVICE_UUID);
BLECharacteristic dataChar(DATA_UUID, BLENotify, PACKET_SIZE);
BLECharacteristic controlChar(CONTROL_UUID, BLEWrite, 20);

bool isRecording = false;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000);

  if (!IMU.begin()) {
    Serial.println("IMU init failed");
    while (1);
  }
  Serial.println("IMU ready");

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }

  BLE.setLocalName("RunLogger");
  BLE.setDeviceName("RunLogger");
  BLE.setAdvertisedService(runService);
  runService.addCharacteristic(dataChar);
  runService.addCharacteristic(controlChar);
  BLE.addService(runService);
  BLE.advertise();

  Serial.println("Advertising as RunLogger");
}


void handleControl() {
  if (!controlChar.written()) return;

  String cmd = "";
  const uint8_t* v = controlChar.value();
  int len = controlChar.valueLength();
  for (int i = 0; i < len; i++) cmd += (char)v[i];
  cmd.trim();

  if (cmd == "START") {
    isRecording = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("START");
  }
  if (cmd == "STOP") {
    isRecording = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("STOP");
  }
}


void sendSample() {
  // Wait for a fresh sample from the IMU
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  uint32_t t_us = micros();

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);   // g
  IMU.readGyroscope(gx, gy, gz);      // dps

  int16_t ax_i = (int16_t)(ax * ACCEL_SCALE);
  int16_t ay_i = (int16_t)(ay * ACCEL_SCALE);
  int16_t az_i = (int16_t)(az * ACCEL_SCALE);
  int16_t gx_i = (int16_t)(gx * GYRO_SCALE);
  int16_t gy_i = (int16_t)(gy * GYRO_SCALE);
  int16_t gz_i = (int16_t)(gz * GYRO_SCALE);

  uint8_t packet[PACKET_SIZE];
  memcpy(packet + 0,  &t_us, 4);
  memcpy(packet + 4,  &ax_i, 2);
  memcpy(packet + 6,  &ay_i, 2);
  memcpy(packet + 8,  &az_i, 2);
  memcpy(packet + 10, &gx_i, 2);
  memcpy(packet + 12, &gy_i, 2);
  memcpy(packet + 14, &gz_i, 2);

  dataChar.writeValue(packet, PACKET_SIZE);
}


void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      BLE.poll();
      handleControl();
      if (isRecording) sendSample();
    }

    isRecording = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected");
  }
}

#include <MMA845xQ.h>

MMA845xQ Sensor;

// Time control
uint32_t t;
uint32_t period = 2500;

void setup() {

  // Begin serial
  Serial.begin(115200);

  // Begin MMA845XQ sensor
  Serial.println("Begin MMA845xQ Sensor...");
  if (Sensor.begin(400000)){
    if (Sensor.deviceId() == MMA8451Q_ID) Serial.println("MMA8451Q detected");
    if (Sensor.deviceId() == MMA8452Q_ID) Serial.println("MMA8452Q detected");
    if (Sensor.deviceId() == MMA8453Q_ID) Serial.println("MMA8453Q detected");
  }
  else {
    Serial.println("No sensor detected");
    while (1);
  }

  // Set acceleration range to +/- 4 g
  Serial.println("Set acceleration range +/- 4 g");
  Sensor.setRange(MMA845XQ_RANGE_4);

  // Set data rate to 400 Hz in wake phase
  Serial.println("Set data rate to 400 Hz");
  Sensor.setDataRate(MMA845XQ_DATA_RATE_400, MMA845XQ_DATA_RATE_6_25);

  t = micros();

}

void loop() {

  // Get sampling time and measure acceleration
  uint32_t ts = micros();
  int16_t accelX, accelY, accelZ;
  Sensor.readInst(&accelX, &accelY, &accelZ);

  // Store time and acceleration data as hexadecimal number in buffer
  uint8_t buf[21];
  buf[0]  = (ts >> 28) & 0x0F;
  buf[1]  = (ts >> 24) & 0x0F;
  buf[2]  = (ts >> 20) & 0x0F;
  buf[3]  = (ts >> 16) & 0x0F;
  buf[4]  = (ts >> 12) & 0x0F;
  buf[5]  = (ts >> 8) & 0x0F;
  buf[6]  = (ts >> 4) & 0x0F;
  buf[7]  = ts & 0x0F;
  buf[8]  = (accelX >> 12) & 0x0F;
  buf[9]  = (accelX >> 8) & 0x0F;
  buf[10] = (accelX >> 4) & 0x0F;
  buf[11] = accelX & 0x0F;
  buf[12] = (accelY >> 12) & 0x0F;
  buf[13] = (accelY >> 8) & 0x0F;
  buf[14] = (accelY >> 4) & 0x0F;
  buf[15] = accelY & 0x0F;
  buf[16] = (accelZ >> 12) & 0x0F;
  buf[17] = (accelZ >> 8) & 0x0F;
  buf[18] = (accelZ >> 4) & 0x0F;
  buf[19] = accelZ & 0x0F;
  buf[20] = '\n';

  // Convert hexadecimal number in buffer to ASCII character
  for (uint8_t i=0; i<20; i++) {
    if (buf[i] < 10) buf[i] += 48;
    else buf[i] += 55;
  }
  // Print data using serial communication to host
  Serial.flush();
  Serial.write(buf, 21);

  // Wait until period end
  t += period;
  while (micros() < t) {
    delayMicroseconds(1);
  }

}

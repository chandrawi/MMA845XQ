#include <MMA845xQ.h>

MMA845xQ Sensor;

// Uncomment below to select to read raw acceleration data
//#define READING_ACCELERATION_RAW
// Uncomment below to select to read acceleration in mm/s^2
#define READING_ACCELERATION_MM_S2

void setup() {

  // Begin serial
  Serial.begin(38400);

  // Begin MMA845XQ sensor
  Serial.println("Begin MMA845xQ Sensor...");
  if (Sensor.begin()){
    if (Sensor.deviceId() == MMA8451Q_ID) Serial.println("MMA8451Q detected");
    if (Sensor.deviceId() == MMA8452Q_ID) Serial.println("MMA8452Q detected");
    if (Sensor.deviceId() == MMA8453Q_ID) Serial.println("MMA8453Q detected");
  }
  else {
    Serial.println("No sensor detected");
    while (1);
  }
  
  // Activate FIFO for reading sensor data
  Sensor.setFifoEnable();

  // Set acceleration range to +/- 4 g and unit to mm/s^2
  Serial.println("Set acceleration range +/- 4 g and unit to 9,81 mm/s^2");
  Sensor.setRange(MMA845XQ_RANGE_4);
  Sensor.setUnit(MMA845XQ_UNIT_M_S2);

  // Set data rate to 100 Hz in wake phase
  Serial.println("Set data rate to 100 Hz");
  Sensor.setDataRate(MMA845XQ_DATA_RATE_100, MMA845XQ_DATA_RATE_6_25);

  Serial.println("\nAccel-X\tAccel-Y\tAccel-Z");

}

void loop() {
  
#ifdef READING_ACCELERATION_RAW
  Serial.print(Sensor.readX()); Serial.print("\t");
  Serial.print(Sensor.readY()); Serial.print("\t");
  Serial.print(Sensor.readZ()); Serial.print("\n");
#endif
  
#ifdef READING_ACCELERATION_MM_S2
  Serial.print(Sensor.accelerationX()); Serial.print("\t");
  Serial.print(Sensor.accelerationY()); Serial.print("\t");
  Serial.print(Sensor.accelerationZ()); Serial.print("\n");
#endif

}
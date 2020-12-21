#include <MMA845xQ.h>
#include <avr/sleep.h>

MMA845xQ Accel;

// Interrupt pin for waking cpu and interrupt flag
int8_t wakeUpPin = 2;
volatile bool flag = false;

// Duration and period data sampling setting
uint32_t tDuration = 1000, tStart;                       // sampling duration in millisecond
uint32_t tPeriod = 10000, tStartPeriod;                  // sampling period in microsecond
uint32_t nSample = (tDuration * 1000) / tPeriod;
uint16_t i;

// Data object for acceleration readings
struct dataObject {
  uint32_t t;
  int16_t x;
  int16_t y;
  int16_t z;
};
dataObject dataLog;

void setup() {

  // Set interrupt pin as input
  pinMode(wakeUpPin, INPUT);

  // Begin serial communication
  Serial.begin(38400);

  // Begin MMA845XQ sensor
  Accel.begin();

  // Set acceleration range to +/- 4 g
  Accel.setRange(MMA845XQ_RANGE_4);

  // Setting for data rate, oversampling mode in both wake and sleep phase
  Accel.setDataRate(
    MMA845XQ_DATA_RATE_100,                        // 100 Hz data rate in wake up phase
    MMA845XQ_DATA_RATE_12_5                        // 12.5 Hz data rate in sleep phase
  );
  Accel.setOverSamplingMode(
    MMA845XQ_OVER_SAMPLING_HIGH_RESOLUTION,        // High resolution mode in wake up phase
    MMA845XQ_OVER_SAMPLING_LOW_POWER               // Low power mode in sleep phase
  );

  // Setting minimun wake time and enable auto sleep
  Accel.setWakeTimeMin(10);                        // (10 x 320 ms) Minimum wake up time 3.2 s
  Accel.setSleepEnable();
  
  // Enabling and setting for motion detection
  Accel.setMotion(
    MMA845XQ_EVENT_MOTION,                         // Motion event detection mode without latch
    MMA845XQ_AXIS_XY,                              // Motion detection on axis X and Z
    8,                                             // (8 x 0.0625 g) 0.5 g acceleration threshold
    6                                              // 6 times debounce counter
  );
  Accel.setMotionEnable();

  // Enabling and setting for transient detection
  Accel.setTransient(
    MMA845XQ_EVENT_TRANSIENT_HPF,                  // Transient event detection using high pass filter (HPF) mode without latch
    MMA845XQ_AXIS_XYZ,                             // Transient detection on axis X, Y and Z
    8,                                             // (8 x 0.0625 g) 0.5 g acceleration threshold
    6                                              // 6 times debounce counter
  );
  Accel.setTransientEnable();

  // Request interrupt for motion and transient detection on interrupt pi 1
  Accel.requestInterrupt(MMA845XQ_INT_PIN_1, MMA845XQ_INT_EVENT_MOTION_WAKE, MMA845XQ_INT_MODE_FALLING);
  Accel.requestInterrupt(MMA845XQ_INT_PIN_1, MMA845XQ_INT_EVENT_TRANSIENT_WAKE, MMA845XQ_INT_MODE_FALLING);

}

void loop() {

  // Put mcu to sleep
  Sleep();

  // Check for motion or transient threshold reached
  if(flag){
    uint8_t intCode = Accel.checkInterrupt();

    // Check if motion event detected
    // Show which axis triggered motion detection interrupt and its polarity
    if (Accel.checkMotion()){
      Serial.print("Motion interrupt triggered (code: ");
      Serial.print(intCode);
      Serial.print(")\nAxis-X: ");
      Serial.print(Accel.motionAxisX);
      Serial.print("\tAxis-Y: ");
      Serial.print(Accel.motionAxisY);
      Serial.print("\tAxis-Z: ");
      Serial.print(Accel.motionAxisZ);
      Serial.print("\n\n");
    }

    // Check if transient event detected
    // Show which axis triggered transient detection interrupt and its polarity
    if (Accel.checkTransient()){
      Serial.print("Transient interrupt triggered (code: ");
      Serial.print(intCode);
      Serial.print(")\nAxis-X: ");
      Serial.print(Accel.transientAxisX);
      Serial.print("\tAxis-Y: ");
      Serial.print(Accel.transientAxisY);
      Serial.print("\tAxis-Z: ");
      Serial.print(Accel.transientAxisZ);
      Serial.print("\n\n");
    }

    // Show message if interrupt didn't caused by motion or transient detection
    if (!Accel.interruptMotion && !Accel.interruptTransient){
      Serial.print("Unknown interrupt (code: ");
      Serial.print(intCode);
      Serial.print(")\n\n");
      return;
    }

    Serial.print("time\taccel-X\taccel-Y\taccel-Z\n");

    dataLog.t = 0;
    tStart = millis();
    tStartPeriod = micros();
    
    // Continuously read and show acceleration readings and sampling time at given sampling period and duration
    for (i=0; i<nSample; i++){
      
      dataLog.t = millis() - tStart;
      Accel.read(&dataLog.x, &dataLog.y, &dataLog.z);
  
      Serial.print(dataLog.t); Serial.print("\t");
      Serial.print(dataLog.x); Serial.print("\t");
      Serial.print(dataLog.y); Serial.print("\t");
      Serial.print(dataLog.z); Serial.print("\n");

      while (micros() < (tStartPeriod + tPeriod)) delayMicroseconds(1);
      tStartPeriod += tPeriod;
    }
    Serial.print("\n");

    // Clear interrupt flag
    flag = false;
  }

}

void Sleep() {

  // Clear serial buffer
  Serial.flush();
  
  // Allow wake up pin to trigger interrupt on falling.
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);

#if defined (__AVR__)
  
  // Enter power down state with ADC and BOD module disabled.
  // Wake up when wake up pin is falling.
  noInterrupts();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  ADCSRA = 0;
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);
  interrupts();
  sleep_cpu();

  // Disable external pin interrupt on wake up pin.
  detachInterrupt(digitalPinToInterrupt(wakeUpPin));
  
#endif

}

void wakeUp() {

  // Set flag in case interrupt happen
  flag = true;
  
}

#include <MMA845xQ.h>

MMA845xQ Sensor;

// Interrupt pin on arduino
int8_t intPin = 2;

// Inteerupt flag
volatile bool flag = false;

void setup() {

  // Begin serial and MMA845XQ sensor
  Serial.begin(38400);
  Sensor.begin();

  // Set data rate 50 Hz in wake phase
  Sensor.setDataRate(MMA845XQ_DATA_RATE_50, MMA845XQ_DATA_RATE_6_25);

  // Setting and enabling orientation detection
  Sensor.setOrientation(
    MMA845XQ_ORIENTATION_DEF_THRESHOLD,     // default portrait to landscape angle threshold setting (45°)
    MMA845XQ_ORIENTATION_DEF_HYSTERISIS,    // default hysterisis angle setting (+/- 14°)
    MMA845XQ_ORIENTATION_DEF_BACKFRONT,     // default back to front angle setting (Z < 75° or Z > 285°)
    MMA845XQ_ORIENTATION_DEF_ZLOCK,         // default z-lock angle threshold setting (29°)
    8                                       // 8 times debounce counter
  );
  Sensor.setOrientationEnable();

  // Request interrupt for orientation detection on interrupt pin 1
  Sensor.requestInterrupt(MMA845XQ_INT_PIN_1, MMA845XQ_INT_EVENT_ORIENTATION, MMA845XQ_INT_MODE_FALLING);

  // Attach interrupt pin 2 on arduino to interrupt pin 1
  pinMode(intPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin), trigger, FALLING);

}

void loop() {
  
  // Checking for orientation change
  if (flag){
    if (Sensor.checkOrientation()){

      // Show current orientaion
      switch (Sensor.orientationStatus){
        case MMA845XQ_PORTRAIT_UP_FRONT:
          Serial.println("PORTRAIT_UP_FRONT");
          break;
        case MMA845XQ_PORTRAIT_UP_BACK:
          Serial.println("PORTRAIT_UP_BACK");
          break;
        case MMA845XQ_PORTRAIT_DOWN_FRONT:
          Serial.println("PORTRAIT_DOWN_FRONT");
          break;
        case MMA845XQ_PORTRAIT_DOWN_BACK:
          Serial.println("PORTRAIT_DOWN_BACK");
          break;
        case MMA845XQ_LANDSCAPE_RIGHT_FRONT:
          Serial.println("LANDSCAPE_RIGHT_FRONT");
          break;
        case MMA845XQ_LANDSCAPE_RIGHT_BACK:
          Serial.println("LANDSCAPE_RIGHT_BACK");
          break;
        case MMA845XQ_LANDSCAPE_LEFT_FRONT:
          Serial.println("LANDSCAPE_LEFT_FRONT");
          break;
        case MMA845XQ_LANDSCAPE_LEFT_BACK:
          Serial.println("LANDSCAPE_LEFT_BACK");
          break;
      }

      // Check if z-lock detected
      if (Sensor.orientationZlock){
        Serial.println("Z-lock detected");
      }
      
      Serial.println();
    }
	
	// Clear interrupt flag
    flag = false;
  }

  delay(1000);

}

void trigger() {
  
  // Set flag in case interrupt happen
  flag = true;
  
}

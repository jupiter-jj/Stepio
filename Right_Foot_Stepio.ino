/**
 * This example turns the ESP32 into a Bluetooth LE mouse that continuously moves the mouse.
 */
#include <BleMouse.h>
#include <PMW3360.h>
#include <SPI.h>

//SPIClass SPI1(VSPI);

// PMW Mouse Sensor & Mouse Sensor BLE 
#define SCK 18    // GPIO for Serial Clock
#define MOSI 23   // GPIO for Master Out Slave In
#define MISO 19   // GPIO for Master In Slave Out
#define SS 5     // GPIO for Slave Select
#define MOT 22   // motion interrupt pin, connect this pin to MT on the module.

BleMouse bleMouse("Right ESP32");
PMW3360 sensor;
volatile bool motion = false;

// FSR Analog
#define fsrAnalogPin 36 // FSR is connected to GPIO36
#define fsrThreshold
int fsrReading;      // the analog reading from the FSR resistor divider

// debouncer variables
#define debounceDelay 50
#define changeThreshold 150
#define onThreshold 50
bool mouseFlag = false;
int lastFSRreading;
long lastDebounceTime;

#define cli 250
#define exponent 1.03
#define horiShift -110
#define vertiShift -20


void setup() {
  //SERIAL SETUP ----------------------------------
  Serial.begin(115200);
  //while(!Serial);
  //Serial.println("Serial Initalized");

  //Serial.println("Initializing SPI...");

  // Initialize SPI1
  SPI.begin(SCK, MISO, MOSI, SS); // Configure pins for HSPI
  //Serial.println("SPI Initialized");
  

  //SENSOR SETUP ----------------------------------
  sensor.begin(SS, 250); // to set CPI (Count per Inch), pass it as the second parameter
  /*if(sensor.begin(SS)){  // 10 is the pin connected to SS of the module.
    Serial.println("Sensor initialization successed");
  } else {
    Serial.println("Sensor initialization failed");
  }*/

  //sensor.setCPI(1600);    // or, you can set CPI later by calling setCPI();
  
  pinMode(MOT, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(MOT), motionDetected, FALLING);

  // if motion interrupt pin is already low, read it from the first loop.
  /*if(digitalRead(MOT) == LOW){
    motion = true;
  }*/

  //BLE SETUP ----------------------------------
  //Serial.println("Starting BLE work!");
  bleMouse.begin();
  
  //FSR SETUP ----------------------------------
  void analogSetAttenuation(adc_attenuation_t ADC_ATTEN_DB_11);
}

void loop() {
  fsrReading = analogRead(fsrAnalogPin); // read FSR sensor data
  //Serial.println(fsrReading);

  // DEBOUNCER -----------------------------------------------------------------------
  // If the switch changed, due to noise or pressing:
  if (abs(lastFSRreading - fsrReading) >= changeThreshold) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (mouseFlag != (fsrReading > onThreshold)) {
      mouseFlag = !mouseFlag;
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastFSRreading = fsrReading;

  // --------------------------------------------------------------------------------

  PMW3360_DATA data = sensor.readBurst(); // read PMW mouse sensor data

  if(data.isOnSurface && data.isMotion && bleMouse.isConnected() && mouseFlag)
  {
    //cli();      
    // disable interrupt during motion data processing.
    
    /*Serial.print(data.dx);
    Serial.print("\t");
    Serial.print(data.dy);
    Serial.println();*/

    signed char adjustedDX;
    signed char adjustedDY;
  
    if (data.dx%256 > 128){
      adjustedDX = -((data.dx%256)-256);
      adjustedDX = pow(exponent, adjustedDX-horiShift) + vertiShift;
      adjustedDX = -adjustedDX;
    } else {
      adjustedDX = ((data.dx%256)-256);
      adjustedDX = pow(exponent, adjustedDX-horiShift) + vertiShift;
    }

    if (data.dy%256 > 128){
      adjustedDY = -((data.dy%256)-256);
      adjustedDY = pow(exponent, adjustedDY-horiShift) + vertiShift;
      adjustedDY = -adjustedDY;
    } else{
      adjustedDY = ((data.dy%256)-256);
      adjustedDY = pow(exponent, adjustedDY-horiShift) + vertiShift;
    }

    /*Serial.print(data.dx%256-256);
    Serial.print(", ");
    Serial.println(data.dy%256-256);*/

    bleMouse.move(adjustedDX,adjustedDY,0,0);
    delay(10);

    //motion = false;
    //sei();      // enable interrupt again.
  }
}

/*void motionDetected() // interrupt service routine should be minimized.
{
  motion = true;    // flag setting.
}*/
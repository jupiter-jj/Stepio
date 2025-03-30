/* FSR testing sketch. 
 
Connect one end of FSR to 3.3V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground
Connect LED from pin 11 through a resistor to ground 
 
For more information see www.ladyada.net/learn/sensors/fsr.html */

#include <Arduino.h>
#include <BleMouse.h>
BleMouse bleMouse("Left ESP32");

#define EN 12

//Mux 0 Pins
#define MUX0_S0 4
#define MUX0_S1 0
#define MUX0_S2 2
#define MUX0_S3 15
#define MUX0_AnalogPin 36  // FSR is connected to GPIO36

//Mux 1 Pins
#define MUX1_S0 14
#define MUX1_S1 27
#define MUX1_S2 26
#define MUX1_S3 25
#define MUX1_AnalogPin 39  // FSR is connected to GPIO36


//FSR Variables
int fsrReading;  // the analog reading from the FSR resistor divider
#define N_ROWS 6
#define N_COLS 4
#define scaleFactor 3
#define maxRange -3000
#define minRange 3000
#define debounceThreshold 600
int fsrDataArray[N_ROWS][N_COLS] = {0};
int maxRow;
int maxCol;
int centerRow;
int centerCol;
int changeRow;
int changeCol;
//int prevCenterRow;
//int prevCenterCol;

static unsigned long lastZoneRelease = 0;
static unsigned long lastScroll = 0;
int lastZone = 0;
int currentZone = 0;
bool clickFlag = false;
bool scrollFlag = false;
int scrollRef;
#define scrollUpdate 100
#define scrollTimer 1500

#define scrollThreshold 20
#define scrollExponent 3

int fsrRead(int row, int column){
  row -= 1;
  column -= 1;

  const uint8_t fsrBinaryMap[N_ROWS][N_COLS] = {
    {0b00000, 0b00001, 0b00010, 0b00011},
    {0b00110, 0b01001, 0b01100, 0b01111},
    {0b00101, 0b01000, 0b01011, 0b01110},
    {0b00100, 0b00111, 0b01010, 0b01101},
    {0b10111, 0b10101, 0b10011, 0b10001},
    {0b10110, 0b10100, 0b10010, 0b10000}
  };

  uint8_t fsrBinaryCode = fsrBinaryMap[row][column];
  //Serial.println(fsrBinaryCode, BIN);
  const int muxPins0[4] = {MUX0_S0, MUX0_S1, MUX0_S2, MUX0_S3}; //array for binary pins
  const int muxPins1[4] = {MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3};  // Example GPIO pins for the 4 binary digits

  if (fsrBinaryCode >> 4){ //falsey if 0xxxx & true is 1xxxx
    //MUX 1:
    for (int n = 0; n < 4; n++) {
       /*Serial.print("muxPins1 S");
       Serial.print(n);
       Serial.print(": ");*/
      if (fsrBinaryCode & (1 << n)) {  // Check if the nth bit is 1
        digitalWrite(muxPins1[n], HIGH);
        //Serial.println("HIGH");
      } else {
        digitalWrite(muxPins1[n], LOW);
        //Serial.println("LOW");
      }
    }
    return analogRead(MUX1_AnalogPin); //return fsr analog value

  } else {
    //MUX 0:
    for (int n = 0; n < 4; n++) {
      if (fsrBinaryCode & (1 << n)) {  // Check if the ith bit is 1
        digitalWrite(muxPins0[n], HIGH);
      } else {
        digitalWrite(muxPins0[n], LOW);
      }
    }
    return analogRead(MUX0_AnalogPin); //return fsr analog value
  }

}

void maxCoordinates(int data[N_ROWS][N_COLS], int &maxRow, int &maxCol) {
    maxRow = -1;
    maxCol = -1;
    int maxVal = debounceThreshold; // Initialize with first element

    
    for (int i = 1; i <= N_ROWS; i++) {
        for (int j = 1; j <= N_COLS; j++) {
            if (data[i-1][j-1] > maxVal) {
                maxVal = data[i-1][j-1];
                maxRow = i;
                maxCol = j;
            }
        }
    }
}

void dataRefineRes(int data[N_ROWS][N_COLS], int &maxRow, int &maxCol, int &centerRow, int &centerCol){
    int analyzeFSR;
    int row_points = 0;
    int col_points = 0;

    // Directions: {row offset, col offset}
    int directions[8][2] = {
        {-1,  0}, // Above
        {1,   0}, // Below
        {0,  -1}, // Left
        {0,   1}, // Right
        {-1, -1}, // Top-left diagonal
        {-1,  1}, // Top-right diagonal
        {1,  -1}, // Bottom-left diagonal
        {1,   1}  // Bottom-right diagonal
    };


    if ((maxCol == -1) && (maxRow == -1)){
      centerRow = -1;
      centerCol = -1;
    } else {
      for (int i = 0; i < 8; i++) {
          int analyzeX = (maxRow-1) + directions[i][0];
          int analyzeY = (maxCol-1) + directions[i][1];

          if (analyzeX >= 0 && analyzeX < N_ROWS && analyzeY >= 0 && analyzeY < N_COLS) {
            analyzeFSR = data[analyzeX][analyzeY];
            row_points += analyzeFSR * directions[i][0];
            col_points += analyzeFSR * directions[i][1];
          }
      }

      // vertical alignment
      if (row_points < -1500){ // above
        centerRow = (maxRow * 3) - 2;
      } else if (row_points > 1500){ // below
        centerRow = (maxRow * 3);
      } else { // middle
        centerRow = (maxRow * 3) - 1;
      }

      // horizontal alignment
      if (col_points < -1500){ // left
        centerCol = (maxCol * 3) - 2;
      } else if (col_points > 1500){ // right
        centerCol = (maxCol * 3);
      } else { // middle
        centerCol = (maxCol * 3) - 1;
      }
    }
}

//0 = inactive, 1 = left active, 2 = right active, 3 = both active
int activationZone(int data[N_ROWS][N_COLS]){
  bool activateFlag = false;

  for(int row=1; row<=2; row++){
    for(int col=0; col<=3; col++){
      if (data[row][col] > debounceThreshold){
        activateFlag = true;
      }
    }
  }

  if (!activateFlag){
    return 0;
  } else { // if (leftBool && !rightBool)
    return 1;
  } /*else if (!leftBool && rightBool){
    return 2;
  } else if (leftBool && !rightBool){
    return 3;
  }*/
}

signed char scrollAdjust(int scrollRef, int centerRow) {
  signed char adjustedScroll = scrollRef-centerRow;
  if (adjustedScroll < 0) {
    adjustedScroll = -adjustedScroll;
  }

  if (adjustedScroll > scrollThreshold){
    adjustedScroll = (pow(adjustedScroll, scrollExponent));
    if (adjustedScroll > 128){
      adjustedScroll = 128;
    }
  }

  if (scrollRef-centerRow < 0){
    adjustedScroll = -adjustedScroll;
  }

  Serial.println(adjustedScroll);
  return adjustedScroll;
}

// MAIN PROGRAM ---------------------------------------------

void setup(void) {
  Serial.begin(115200); 
  pinMode(EN, OUTPUT);

  //init Mux 0 binary pins
  pinMode(MUX0_S0, OUTPUT);
  pinMode(MUX0_S1, OUTPUT);
  pinMode(MUX0_S2, OUTPUT);
  pinMode(MUX0_S3, OUTPUT);

  //init Mux 1 binary pins
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  pinMode(MUX1_S3, OUTPUT);

  //BLE Setup
  bleMouse.begin();
  delay(100);

  //set Mux 1 & Mux 2 analog pins
  void analogSetAttenuation(adc_attenuation_t ADC_ATTEN_DB_11);
}

void loop(void) {
  digitalWrite(EN, LOW);

  for (int row=1; row<=N_ROWS; row++) {
    for (int column=1; column<=N_COLS; column++){
      fsrDataArray[row-1][column-1] = fsrRead(row, column);
    }
  }

  maxCoordinates(fsrDataArray, maxRow, maxCol);
  
  /*Serial.print(maxRow);
  Serial.print(" x ");
  Serial.println(maxCol);*/

  dataRefineRes(fsrDataArray, maxRow, maxCol, centerRow, centerCol);

  /*Serial.print(centerRow);
  Serial.print(" x ");
  Serial.println(centerCol);*/

  currentZone = activationZone(fsrDataArray);

  //update all "PREV" variables
  if (currentZone == 0) {
    lastZoneRelease = millis();
  }
  
  if (maxRow == -1 && maxCol == -1){
    scrollFlag = false;
  }

  if ((millis() - lastZoneRelease >= scrollTimer) && (!scrollFlag)){
    scrollFlag = true;
    scrollRef = centerRow;
    lastScroll = millis();
    bleMouse.move(0,0, -5, 0);
    delay(500);
    bleMouse.move(0,0, 5, 0);
  }

  if (scrollFlag && (millis()-lastScroll >= scrollUpdate)) {
    lastScroll = millis();
    signed char adjustedScroll = scrollAdjust(scrollRef, centerRow);
    bleMouse.move(0,0, adjustedScroll, 0);
  } else if ((currentZone != lastZone) && (currentZone == 0)){
    //Serial.println("MOUSE MODE");
    if (lastZone != 0){
      //LEFT CLICK
      bleMouse.click(MOUSE_LEFT);
    }/* else if (lastZone == 2){
      //RIGHT CLICK
      bleMouse.click(MOUSE_RIGHT);
    } else if (lastZone == 3){
      //MIDDLE CLICK
      bleMouse.click(MOUSE_MIDDLE);
    }*/
  }

  lastZone = activationZone(fsrDataArray);
}

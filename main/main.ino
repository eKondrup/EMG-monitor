#include <compat/deprecated.h>
#include <TimerOne.h>

#define NUMCHANNELS 3
#define HEADERLEN 4
#define PACKETLEN (NUMCHANNELS * 2 + HEADERLEN + 1)
#define SAMPFREQ 256 // Increased sampling frequency
#define TIMER_INTERVAL_MICROSECONDS (1000000UL / SAMPFREQ)
#define LED1 13
#define CAL_SIG 9
#define WINDOWSIZE 100

#define OFFSET 800
#define NOISE_LVL 20
#define AMP_X 1.5


volatile signed int maxValue = 0;
volatile signed int minValue = 0;
volatile long sumValues = 0;
volatile int numValues = 0;
volatile signed int movingAvgValue = 0;

#define MOVING_AVG_WINDOW 100
volatile signed int movingAvgArr[MOVING_AVG_WINDOW];
volatile long movingAvgSum = 0;
volatile int movingAvgIndex = 0;
volatile unsigned int printCounter = 0;
volatile bool clearCommandReceived = false;

volatile unsigned char Filt_Arr[WINDOWSIZE];
volatile unsigned char FiltIndex = 0;
volatile unsigned int SumVal = 0;
volatile signed int ADC_Value = 0;
volatile signed int Filt_Value = 0;

volatile bool processDataFlag = false;

void setup() {
  noInterrupts();

  for (int i = 0; i < WINDOWSIZE; i++) {
    Filt_Arr[i] = 0;
  }
  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    movingAvgArr[i] = 0;
  }

  Timer1.initialize(TIMER_INTERVAL_MICROSECONDS);
  Timer1.attachInterrupt(Timer2_Overflow_ISR);

  Serial.begin(57600);


  interrupts();
}

void Timer2_Overflow_ISR() {
  // Read the ADC input
  ADC_Value = analogRead(0);

  // Removing offset before rectifying
  Filt_Value = ADC_Value - OFFSET;
  if (Filt_Value < 0) Filt_Value = -Filt_Value;

  // Smoothing filter values update
  SumVal -= Filt_Arr[WINDOWSIZE - 1];
  for (FiltIndex = WINDOWSIZE - 1; FiltIndex > 0; FiltIndex--) {
    Filt_Arr[FiltIndex] = Filt_Arr[FiltIndex - 1];
  }
  Filt_Arr[0] = Filt_Value;
  SumVal += Filt_Arr[0];

  // Smoothing filter processing
  long smoothValue = SumVal / WINDOWSIZE;
  if (smoothValue < NOISE_LVL) {
    Filt_Value = 0;
  } else {
    long tempValue = smoothValue - NOISE_LVL;
    Filt_Value = tempValue * tempValue; // Square to avoid floating-point pow
    if (Filt_Value < 0) Filt_Value = 0; // Ensure no negative values due to overflow
  }

  // Update statistics
  numValues++;
  sumValues += Filt_Value;
  long avgValue = sumValues / numValues;
  if (Filt_Value > maxValue) maxValue = Filt_Value;
  if (Filt_Value < minValue) minValue = Filt_Value;

  // Update moving average
  movingAvgSum -= movingAvgArr[movingAvgIndex];
  movingAvgArr[movingAvgIndex] = Filt_Value;
  movingAvgSum += Filt_Value;
  movingAvgIndex = (movingAvgIndex + 1) % MOVING_AVG_WINDOW;
  movingAvgValue = movingAvgSum / MOVING_AVG_WINDOW;

  // Control serial output frequency
  printCounter++;
  if (printCounter >= (SAMPFREQ / 10)) { // Adjust this value to control print frequency
    printCounter = 0;
    Serial.print("EMG_Værdi:"); Serial.print(Filt_Value); Serial.print(", ");
    Serial.print("Moving_Avg:"); Serial.print(movingAvgValue); Serial.print(", ");
    Serial.print("Avg:"); Serial.print(avgValue); Serial.print(", ");
    Serial.print("Min:"); Serial.print(minValue); Serial.print(", ");
    Serial.print("Max:"); Serial.print(maxValue); Serial.println();
  }

  // Check for serial commands
  if (Serial.available() > 0) {
    char c = Serial.read();
    static String command = "";
    if (c == '\n') {
      command.trim();
      if (command.equals("clear")) {
        clearCommandReceived = true;
      }
      command = ""; // Clear the command string
    } else {
      command += c;
    }
  }
}
void loop() {
  if (clearCommandReceived) {
    clearCommandReceived = false;
    resetValues();
  }
 __asm__ __volatile__ ("sleep");
 
}
void resetValues() {
  noInterrupts();

  for (int i = 0; i < WINDOWSIZE; i++) {
    Filt_Arr[i] = 0;
  }
  maxValue = 0;
  minValue = 0;
  sumValues = 0;
  numValues = 0;

  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    movingAvgArr[i] = 0;
  }

  movingAvgSum = 0;
  movingAvgIndex = 0;
  movingAvgValue = 0;

  interrupts();
}

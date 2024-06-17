
/**********************************************************/
//Dette script er en modificering af kode skrevet af 
//Penko Todorov Bozhkov
//Programmet er skrevet til SHIELD-EKG/EMG + Olimexino328
//Lavet i Arduino IDE 2.3.2 og testet på Arduino Uno REV3
//Det oprindelige script kørte på flexi2timer, men blev
//ændret til TimerOne grundet problemer med pwm på pin 3
//Programmet virker sammen med Arduino Motor Shield REV3
//med OLIMEX tilkoblet
/**********************************************************/
#include <compat/deprecated.h>
#include <TimerOne.h>

// All definitions
#define NUMCHANNELS 3
#define HEADERLEN 4
#define PACKETLEN (NUMCHANNELS * 2 + HEADERLEN + 1)
#define SAMPFREQ 256
#define TIMER_INTERVAL_MICROSECONDS (1000000UL / SAMPFREQ)
#define LED1 13
#define CAL_SIG 9
#define WINDOWSIZE 100

//Kalibrerings variabler
#define OFFSET 800
#define NOISE_LVL 20
#define AMP_X 1.5

//Styringsvariabler
#define THRESHOLD_HIGH 200
#define THRESHOLD_LOW 100
#define pwmPin 3
volatile int pwmValue = 120;      // Starting PWM value
static int isrCounter = 0;        // ISR invocation counter


// Statistics variables
volatile signed int maxValue = 0;
volatile signed int minValue = 0;
volatile float avgValue = 0;
volatile long sumValues = 0;
volatile int numValues = 0;

// Moving average variables
#define MOVING_AVG_WINDOW 100
volatile signed int movingAvgArr[MOVING_AVG_WINDOW];
volatile long movingAvgSum = 0;
volatile int movingAvgIndex = 0;
volatile signed int movingAvgValue = 0;

// Global constants and variables
volatile unsigned char TXBuf[PACKETLEN];      //The transmission packet
volatile unsigned char TXIndex;               //Next byte to write in the transmission packet.
volatile unsigned char counter = 0;	          //Additional divider used to generate CAL_SIG

volatile unsigned char Filt_Arr[WINDOWSIZE];  //Values for Smoothing filter input
volatile unsigned char FiltIndex;             //Index for updating Filt_Arr.
volatile unsigned int SumVal;                 //Sum of values from Filt_Arr
volatile signed int ADC_Value = 0;	          //ADC current value
volatile signed int Filt_Value = 0;           //Filtered current value


//~~~~~~~~~~
// Functions
//~~~~~~~~~~


void setup() {
 noInterrupts();
  pinMode(pwmPin, OUTPUT);
 // LED1
 pinMode(LED1, OUTPUT);  //Setup LED1 direction
 digitalWrite(LED1,LOW); //Setup LED1 state
 pinMode(CAL_SIG, OUTPUT);
 
 pinMode(LED1, OUTPUT);  //Setup LED1 direction
 digitalWrite(LED1,LOW); //Setup LED1 state
 pinMode(CAL_SIG, OUTPUT);
 
 //Write packet header and footer
TXBuf[0] = 0xA5;  // Sync 0
TXBuf[1] = 0x5A;  // Sync 1
 TXBuf[2] = 2;       //Protocol version
 TXBuf[3] = 0;       //Packet counter
 TXBuf[4] = 0x02;    //CH1 High Byte
 TXBuf[5] = 0x00;    //CH1 Low Byte
 TXBuf[6] = 0x02;    //CH2 High Byte
 TXBuf[7] = 0x00;    //CH2 Low Byte
 TXBuf[8] = 0x02;    //CH3 High Byte
 TXBuf[9] = 0x00;    //CH3 Low Byte
 TXBuf[10] = 0x02;   //CH4 High Byte
 TXBuf[11] = 0x00;   //CH4 Low Byte
 TXBuf[12] = 0x02;   //CH5 High Byte
 TXBuf[13] = 0x00;   //CH5 Low Byte
 TXBuf[14] = 0x02;   //CH6 High Byte
 TXBuf[15] = 0x00;   //CH6 Low Byte 
 TXBuf[2 * NUMCHANNELS + HEADERLEN] =  0x01;  // Switches state

//Initialize Filt_Arr to 0
   for (int i = 0; i < WINDOWSIZE; i++) {
     Filt_Arr[i] = 0;
   }
//Initialize mav to all zeros
for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
  movingAvgArr[i] = 0;
}


 // Timer setup with TimerOne
  Timer1.initialize(TIMER_INTERVAL_MICROSECONDS);  // Initialize TimerOne to the desired interval in microseconds
  Timer1.attachInterrupt(Timer2_Overflow_ISR);  // Attach the ISR function
    
 
 // Serial Port
 Serial.begin(57600);
 //Set speed to 57600 bps

 // MCU sleep mode = idle.
 //outb(MCUCR,(inp(MCUCR) | (1<<SE)) & (~(1<<SM0) | ~(1<<SM1) | ~(1<<SM2)));
 
 interrupts();  // Enable all interrupts after initialization has been completed
}

/****************************************************/
/*  Function name: Timer2_Overflow_ISR              */
/*  Parameters                                      */
/*    Input   :  No                             */
/*    Output  :  No                                 */
/*    Action: Determines ADC sampling frequency.    */
/****************************************************/
void Timer2_Overflow_ISR() {
  // initialize digital pin(s) as output. NB: Pin 9 and 13 are in use
  pinMode(4, OUTPUT); //Relay4 COM4 - J1
  //analogWrite(pwmPin, 160);
  //Read the ADC input

    ADC_Value = analogRead(0);
    
    
    // Removing offset before rectifying
    Filt_Value = ADC_Value-OFFSET;

    //Rectifier
    if(Filt_Value<0)Filt_Value = Filt_Value*-1;


    //Serial.println(Filt_Value, DEC);
    //Smoothing filter values update
    SumVal-=Filt_Arr[WINDOWSIZE-1];
    for(FiltIndex=WINDOWSIZE-1;FiltIndex>0;FiltIndex--){
      Filt_Arr[FiltIndex]=Filt_Arr[FiltIndex-1];
    }
    Filt_Arr[0]=Filt_Value;
    SumVal+= Filt_Arr[0];
    //Filt_Value = pow((SumVal / WINDOWSIZE)-NOISE_LVL, 2);
    
    if((SumVal / WINDOWSIZE)<NOISE_LVL){
      Filt_Value = 0;
    }else{
    Filt_Value = pow((SumVal / WINDOWSIZE)-NOISE_LVL, AMP_X);
    }

    //statistik
  numValues++;
  sumValues += Filt_Value;
  avgValue = (float)sumValues / numValues;
  if (Filt_Value > maxValue) maxValue = Filt_Value;
  if (Filt_Value < minValue) minValue = Filt_Value;

  // Update moving average
  movingAvgSum -= movingAvgArr[movingAvgIndex];
  movingAvgArr[movingAvgIndex] = Filt_Value;
  movingAvgSum += Filt_Value;
  movingAvgIndex = (movingAvgIndex + 1) % MOVING_AVG_WINDOW;
  movingAvgValue = movingAvgSum / MOVING_AVG_WINDOW;

  // Send Filt_Value and statistics to Serial Plotter
  Serial.print(Filt_Value);
  Serial.print(",");
  Serial.print(maxValue);
  Serial.print(",");
  Serial.print(minValue);
  Serial.print(",");
  Serial.println(avgValue);
  Serial.print(",");
  Serial.println(movingAvgValue);
  
}


/****************************************************/
/*  Function name: loop                             */
/*  Parameters                                      */
/*    Input   :  No                             */
/*    Output  :  No                                 */
/*    Action: Puts MCU into sleep mode.             */
/****************************************************/
void loop() {
  
 __asm__ __volatile__ ("sleep");
 
}
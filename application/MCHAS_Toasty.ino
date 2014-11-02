// This #include statement was automatically added by the Spark IDE.
#include "pid/pid.h"

// This #include statement was automatically added by the Spark IDE.
#include "encoder.h"

// This #include statement was automatically added by the Spark IDE.
#include "SparkIntervalTimer/SparkIntervalTimer.h"

// This #include statement was automatically added by the Spark IDE.
#include "adafruit-max31855/adafruit-max31855.h"

//TFT stuff
#include "Adafruit_mfGFX.h"    // Core graphics library
#include "Adafruit_ST7735.h" // Hardware-specific library
#include "fonts.h"


// Pin Definitions **************************************************************
#define LCD_CS      D5
#define LCD_DC      D2
#define LCD_RST     D3

#define THERMO1_CS  D7
#define THERMO2_CS  D6

#define SSR1        D1
#define SSR2        D0

#define ENC_A       A1
#define ENC_B       A7
#define ENC_BUTTON  A6

//Constants *********************************************************************
#define ENCODER_SERVICE_DELAY 4       //4 half milliseconds = 2 ms

//instance the hardware Libraries ***********************************************
ClickEncoder encoder(ENC_A, ENC_B, ENC_BUTTON);
Adafruit_ST7735 tft = Adafruit_ST7735(LCD_CS, LCD_DC, LCD_RST); // hardware spi
AdafruitMAX31855 thermocouple1(THERMO1_CS);
AdafruitMAX31855 thermocouple2(THERMO2_CS);


//Timer Instance for reading the encoder ****************************************
IntervalTimer myTimer;

//Global Variables **************************************************************
double externalTemp = 0;
double internalTemp = 0;
int16_t encoderVal = 0;

//pid variables
double Input = 0;
double Output = 0;
double Setpoint = 80;

unsigned long windowStartTime;

double aggKp=2000, aggKi=0.1, aggKd=100;

#define WINDOW_SIZE 5000

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, PID::DIRECT);

//REFLOW STATE State Enumerate **************************************************
typedef enum {
    STATE_IDLE          = 0,
    STATE_RAMP_TO_SOAK  = 10,
    STATE_SOAK          = 11,
    STATE_RAMP_TO_PEAK  = 12,
    STATE_RAMP_DOWN     = 13,
    STATE_COOL_DOWN     = 14,
    STATE_COMPLETE      = 15,
    STATE_TUNE          = 30
} reflowState;

reflowState reflowStateCurrent = STATE_IDLE;
reflowState reflowStateOld = STATE_IDLE;  
uint32_t    reflowStateTime = 0;
double      currentStateTime = 0;
double      currentRunTime = 0;

//encoderHandler ****************************************************************
void encoderService() { 
    encoder.service();
}




//Setup Routine *****************************************************************
void setup() {
    Serial.begin(9600);
    
    tft.initR(INITR_BLACKTAB);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.setCursor(0,0);
    
    //Initialize Thermocouples
    thermocouple1.init();
    
    pinMode(SSR1, OUTPUT);
    pinMode(SSR2, OUTPUT);
    
    digitalWrite(SSR1, LOW);
    
    myTimer.begin(encoderService, ENCODER_SERVICE_DELAY, hmSec);
    encoder.setAccelerationEnabled(1);
    
    Spark.variable("intTemp", &internalTemp, DOUBLE);
    Spark.variable("extTemp", &externalTemp, DOUBLE);
    Spark.variable("refloState", &reflowStateCurrent, INT);
    Spark.variable("stateTime", &currentStateTime, DOUBLE);
    Spark.variable("runTime", &currentRunTime, DOUBLE);    
    
  windowStartTime = millis();
  
  //initialize the variables we're linked to
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WINDOW_SIZE);

  //turn the PID on
  myPID.SetMode(PID::AUTOMATIC);    
}

void loop() {
    externalTemp = thermocouple1.readCelsius();
    internalTemp = thermocouple1.readInternal();
    encoderVal += encoder.getValue();
    
    tft.setCursor(0,0);    
    tft.print("Ext: ");
    tft.print(externalTemp);
    tft.println("   ");
    tft.print("Int: ");
    tft.print(internalTemp);
    tft.println("   ");
    tft.print("encoder: ");
    tft.print(encoderVal);
    tft.println("   ");
    
    tft.print("button: ");
    tft.print(encoder.getButton());
    tft.println("   ");
    tft.print("PID Output: ");
    tft.print(Output);
    tft.println("   ");    
    
    tft.print("M-WST: ");
    tft.print(millis() - windowStartTime);
    tft.println("   "); 
    
    
    
    Input = externalTemp;
    myPID.Compute();
    
    /************************************************
    * turn the output pin on/off based on pid output
    ************************************************/
    if(millis() - windowStartTime>WINDOW_SIZE)
    { //time to shift the Relay Window
    windowStartTime += WINDOW_SIZE;
    }
    
    if(Output > millis() - windowStartTime) digitalWrite(SSR1,HIGH);
    else digitalWrite(SSR1,LOW);
} 
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

//Macros ************************************************************************
#define heaterOn() 		digitalWrite(SSR1,HIGH);
#define heaterOff() 	digitalWrite(SSR1,LOW);
#define fanOn() 		digitalWrite(SSR2,HIGH);
#define fanOff() 		digitalWrite(SSR2,LOW);

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
double pidOutput = 0;
double Setpoint = 80;

unsigned long relayPeriodStartTime;

double aggKp=2000, aggKi=0.1, aggKd=100;

#define RELAY_PERIOD    5000
#define SAFE_TEMP       50.0

//Specify the links and initial tuning parameters
PID myPID(&externalTemp, &pidOutput, &Setpoint, aggKp, aggKi, aggKd, PID::DIRECT);

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

reflowState reflowStateCurrent = STATE_RAMP_TO_SOAK;
reflowState reflowStateOld = STATE_IDLE;  
uint32_t    reflowStateTime = 0;
double      currentStateTime = 0;
double      currentRunTime = 0;

//profile stuff *****************************************************************
typedef struct profileValues_s {
	char	name[31];
	int16_t soakTemp;
	int16_t soakDuration;
	int16_t peakTemp;
	int16_t peakDuration;
	double  rampUpRate;
	double  rampDownRate;
	uint8_t checksum;
} Profile_t;

Profile_t activeProfile; // the one and only instance
uint16_t activeProfileId = 0;
uint8_t  profileZoomFactor = 4;

//profile record *****************************************************************
#define NUM_OF_RECORD_POINTS 512
#define NUM_OF_POST_END_POINTS 64
typedef struct profileRecord_s {
	double secondsPerPoint;
	int16_t  setPoint[NUM_OF_RECORD_POINTS];
	int16_t  actual[NUM_OF_RECORD_POINTS];
} ProfileRecord_t;

ProfileRecord_t profileRecord;
uint16_t    activeProfilePoint = 0;

//encoderHandler ****************************************************************
void encoderService() { 
    encoder.service();
}

//Setup Routine *****************************************************************
void setup() {
    Serial.begin(9600);
    
    tft.initR(INITR_BLACKTAB);
    tft.fillScreen(ST7735_MAGENTA);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.setCursor(0,0);
    
    //Initialize Thermocouples
    thermocouple1.init();
    
    pinMode(SSR1, OUTPUT);
    pinMode(SSR2, OUTPUT);
    heaterOff();
	fanOff();
    
    myTimer.begin(encoderService, ENCODER_SERVICE_DELAY, hmSec);
    encoder.setAccelerationEnabled(1);
    
    Spark.variable("intTemp", &internalTemp, DOUBLE);
    Spark.variable("extTemp", &externalTemp, DOUBLE);
    Spark.variable("refloState", &reflowStateCurrent, INT);
    Spark.variable("stateTime", &currentStateTime, DOUBLE);
    Spark.variable("runTime", &currentRunTime, DOUBLE);    
    
	relayPeriodStartTime = millis();

	//initialize the variables we're linked to
	Setpoint = 100;

	//tell the PID to range between 0 and the full window size
	myPID.SetOutputLimits(0, RELAY_PERIOD);

	//turn the PID on
	myPID.SetMode(PID::AUTOMATIC);  
	
	makeDefaultProfile(); 
}

void loop() {
    externalTemp = thermocouple1.readCelsius();
    internalTemp = thermocouple1.readInternal();
    encoderVal += encoder.getValue();
/*    
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
    tft.print(pidOutput);
    tft.println("   ");    
    
    tft.print("M-WST: ");
    tft.print(millis() - relayPeriodStartTime);
    tft.println("   "); 
    
    tft.print("rflowstate: ");
    tft.print(reflowStateCurrent);
    tft.println("   "); 
    
    tft.print("Setpoint: ");
    tft.print(Setpoint);
    tft.println("   "); 
*/    
	reflowStateMachine();
	tft.setFont(ARIAL_8);
	tft.setTextSize(2);
	tft.setCursor(0,0); 
	tft.print(externalTemp);
	tft.setCursor(64,0); 
	tft.print(Setpoint);
	
} 

//PID Statemachine *****************************************************************
void reflowStateMachine() {
	static uint32_t reflowStateStartTime = millis();
	static uint32_t reflowStateEndTime = millis();
	static double startTemp = 0;
	uint32_t currentTime = millis();

	if(reflowStateCurrent != reflowStateOld) {
		reflowStateStartTime = millis();
		reflowStateOld = reflowStateCurrent;
		
		switch(reflowStateCurrent) {
			case STATE_IDLE:
				//do nothing
				break;
				
			case STATE_RAMP_TO_SOAK:
				startTemp = externalTemp;
				profileRecord.secondsPerPoint = calcProfileTime(startTemp)/(NUM_OF_RECORD_POINTS - NUM_OF_POST_END_POINTS);
				tft.println(profileRecord.secondsPerPoint); 
				resetProfileDisplay();
				reflowStateTime = currentTime;
				activeProfilePoint = 0;
				delay(5000);
				break;
    			
    		case STATE_SOAK:
    			reflowStateEndTime = reflowStateStartTime + activeProfile.soakDuration * 1000;
    			break;
    			
    		case STATE_RAMP_TO_PEAK: 
    			reflowStateEndTime = reflowStateStartTime + activeProfile.peakDuration * 1000;
    			break;
    			
    		case STATE_RAMP_DOWN: 
    			
    			break;
    			
    		case STATE_COOL_DOWN: 
    			
    			break;
    			
    		case STATE_COMPLETE:
    			
    			break;
    			
    		case STATE_TUNE:
    			
    			break;
    			
    		default:
    			
    			break;
        }		
	}
	
	switch(reflowStateCurrent) {
		case STATE_IDLE:
			//do nothing
			break;
			
		case STATE_RAMP_TO_SOAK:
		    trackProfilePoint();
			Setpoint = calculateRampTemp(activeProfile.rampUpRate, currentTime - reflowStateStartTime, startTemp);
			if(Setpoint >= activeProfile.soakTemp) {
				reflowStateCurrent = STATE_SOAK;
			}
			break;
			
		case STATE_SOAK:
		    trackProfilePoint();
			Setpoint = activeProfile.soakTemp;
			if( (int32_t) (currentTime - reflowStateEndTime) >= 0) {
				reflowStateCurrent = STATE_RAMP_TO_PEAK;
			}
			break;
			
		case STATE_RAMP_TO_PEAK:
		    trackProfilePoint();
			if(Setpoint < activeProfile.peakTemp) {
				Setpoint = calculateRampTemp(activeProfile.rampUpRate, currentTime - reflowStateStartTime, activeProfile.soakTemp);
			}
			else {
				Setpoint = activeProfile.peakTemp;
				if( (int32_t) (currentTime - reflowStateEndTime) >= 0) {
					reflowStateCurrent = STATE_RAMP_DOWN;
				}
			}			
			break;
			
		case STATE_RAMP_DOWN:
            trackProfilePoint();		
			Setpoint = calculateRampTemp(activeProfile.rampDownRate, currentTime - reflowStateStartTime, activeProfile.peakTemp);
			if(Setpoint <= activeProfile.soakTemp) {
				reflowStateCurrent = STATE_COOL_DOWN;
			}			
			break;
			
		case STATE_COOL_DOWN: 
		    trackProfilePoint();
		    if(externalTemp <= SAFE_TEMP)
			break;
			
		case STATE_COMPLETE:
			
			break;
			
		case STATE_TUNE:
			
			break;
			
		default:
			
			break;
    }
    
    heaterControl();
}

//Compute PID *****************************************************************
void heaterControl() {
	if(reflowStateCurrent == STATE_IDLE) {
		// don't calculate if in idle
		heaterOff();
	}
	else {
		myPID.Compute();
		
		/************************************************
		* turn the pidOutput pin on/off based on pid pidOutput
		************************************************/
		if(millis() - relayPeriodStartTime>RELAY_PERIOD)
		{ //time to shift the Relay Window
			relayPeriodStartTime += RELAY_PERIOD;
		}
		
		if(pidOutput > millis() - relayPeriodStartTime) {
			heaterOn();
		}
		else {
			heaterOff();
		}
	}
}


double calculateRampTemp(double ramp, int32_t time, int16_t start) {
	return ramp * ( ((double) time) / 1000) + start;
}

double calcProfileTime(double initialTemp) {
	double locReturnValue = (activeProfile.soakTemp - initialTemp) / activeProfile.rampUpRate;
	double temp = (activeProfile.soakTemp - initialTemp) / activeProfile.rampUpRate;
	
	locReturnValue += activeProfile.soakDuration;
	
	if( temp > activeProfile.peakDuration) {
		locReturnValue += temp;
	}
	else {
		locReturnValue += activeProfile.peakDuration;
	}
	
	locReturnValue += (activeProfile.soakTemp - activeProfile.peakTemp) / activeProfile.rampDownRate;
	tft.println(locReturnValue); 
	
	return locReturnValue;
}



//Default Reflow,###,###,###,###,#.##,#.##
void makeDefaultProfile() {
	strcpy(activeProfile.name,"Default Reflow");
	activeProfile.soakTemp     = 130;
	activeProfile.soakDuration =  80;
	activeProfile.peakTemp     = 220;
	activeProfile.peakDuration =  40;
	activeProfile.rampUpRate   = 0.80;
	activeProfile.rampDownRate = -2.0;
}

//************* display functions
#define MAX_TEMP        250
#define MIN_TEMP        30

#define C_PER_PIXEL     2

#define PROFILE_LINES   50

#define TFT_WIDTH       128
#define TFT_HEIGHT      160
#define PROFILE_HEIGHT  (MAX_TEMP - MIN_TEMP) / C_PER_PIXEL
#define PROFILE_BKGD_COLOR ST7735_BLACK
#define PROFILE_LINE_COLOR ST7735_CYAN
#define PROFILE_PROF_COLOR ST7735_BLUE
#define PROFILE_ACTU_COLOR ST7735_RED

void resetProfileDisplay() {
    uint16_t temp = PROFILE_LINES-MIN_TEMP;
    tft.fillRect(0, TFT_HEIGHT-PROFILE_HEIGHT, TFT_WIDTH, PROFILE_HEIGHT, PROFILE_BKGD_COLOR);
    
    while(temp <= MAX_TEMP) {
        tft.fillRect(0, TFT_HEIGHT - (temp / C_PER_PIXEL), TFT_WIDTH, 1, PROFILE_LINE_COLOR);
        
        temp += PROFILE_LINES; 
    }
    
}

void drawProfilePoint() {
    int16_t xPoint = (activeProfilePoint / profileZoomFactor) % TFT_WIDTH;
    int16_t yProfilePoint = TFT_HEIGHT - ((Setpoint - MIN_TEMP) / C_PER_PIXEL);
    int16_t yActualPoint = TFT_HEIGHT - ((externalTemp - MIN_TEMP) / C_PER_PIXEL);
    
    tft.drawPixel(xPoint, yProfilePoint, PROFILE_PROF_COLOR);
    tft.drawPixel(xPoint, yActualPoint, PROFILE_ACTU_COLOR);
}

void trackProfilePoint() {
    uint32_t desiredTimeToTrack = activeProfilePoint * (uint32_t) (profileRecord.secondsPerPoint * 1000);
    
    if(millis() - reflowStateTime >= desiredTimeToTrack) {
        profileRecord.setPoint[activeProfilePoint] = Setpoint * 100;
        profileRecord.actual[activeProfilePoint] = externalTemp * 100;
        if(activeProfilePoint % profileZoomFactor == 0) {
            drawProfilePoint();
        }
        activeProfilePoint++;
    }
}
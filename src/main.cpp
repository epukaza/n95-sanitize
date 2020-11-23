#include <Arduino.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <MAX6675.h>
#include <EasyButton.h>

#define DISPLAY_RESET_PIN 10
#define DISPLAY_DC_PIN 9
#define DISPLAY_CS_PIN 8
#define THERMOCOUPLE_CS_PIN 7
#define DONE_LED_PIN 5
#define SSR_PIN 4
#define BUTTON_PIN 3

const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Heating",
  "Holding temp",
  "Cool",
  "Complete",
  "Wait,hot",
  "Error"
};

typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;


struct reflowProfile{
  const char* profileName;
  int soakTemp;
  unsigned long soakPeriodMS;
};

#define NUM_REFLOW_PROFILES 3
reflowProfile profiles[NUM_REFLOW_PROFILES] = {
  {"Sanitize Masks", 70, 1800000}, // 30*60*1000 30 minutes
  {"Dry PLA", 45, 1800000*8},
  {"Dry PETG", 70, 1800000*4}
};

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 45
#define TEMPERATURE_COOL_MIN 50
#define SENSOR_SAMPLING_TIME 1000
#define DEBOUNCE_PERIOD_MIN 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 50    // default 100
#define PID_KI_PREHEAT 0.025   // default 0.025
#define PID_KD_PREHEAT 50     // default 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300     // default 300
#define PID_KI_SOAK 0.05     // default 0.05
#define PID_KD_SOAK 250     // default 250

#define PID_SAMPLE_TIME 1000  //default 1000

// ***** PID CONTROL VARIABLES *****
double setpoint;
double inputTemp;
double temporaryInputVar;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
unsigned int windowSize;
unsigned long now;
unsigned long windowStartTime;
unsigned long nextRead;
unsigned long soakStartTime;
unsigned long timerSoak;
unsigned long completePeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState = REFLOW_STATE_IDLE;
// Reflow oven controller status
reflowStatus_t reflowStatus = REFLOW_STATUS_OFF;
// did encounter a thermocouple error?
int tcErrorCount = 0;
bool TCError = false;
bool startReflow = false;
int activeReflowProfile = 0;

MAX6675 tcouple(THERMOCOUPLE_CS_PIN);
U8G2_SSD1305_128X64_ADAFRUIT_F_4W_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RESET_PIN);
PID reflowOvenPID(&inputTemp, &output, &setpoint, kp, ki, kd, DIRECT);
EasyButton button(BUTTON_PIN);

// Called once in setup, sets global display attributes.
inline void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_8x13_tr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

// Handle reading temperature from max6675. 
// Also contains logic to protect against single/double read errors.
// Requires three read errors in a row to go into error state.
void readTemp(void) {
  temporaryInputVar = tcouple.readTempC();
  // inputTemp = tcouple.readTempC();
  if(temporaryInputVar == 0.00 || temporaryInputVar == -1.00 ) {
    if(tcErrorCount >= 3) {
      TCError = true;
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    } else {
      tcErrorCount++;
    }
  } else {
    inputTemp = temporaryInputVar;
    tcErrorCount = 0;
    TCError = false;
  }
}

void buttonPressed(void) {
  // If currently reflow process is on going
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    // Button press is for cancelling
    // Turn off reflow process
    reflowStatus = REFLOW_STATUS_OFF;
    // Reinitialize state machine
    reflowState = REFLOW_STATE_IDLE;
    activeReflowProfile = 0;
  } else {
    startReflow = true;
  }
}

void buttonHeld(void) {
  // If currently reflow process is on going
  if (reflowStatus == REFLOW_STATUS_ON)
  {// Button press is for cancelling
    // Turn off reflow process
    reflowStatus = REFLOW_STATUS_OFF;
    // Reinitialize state machine
    reflowState = REFLOW_STATE_IDLE;
    activeReflowProfile = 0;
  } else {
    activeReflowProfile++;
    if (activeReflowProfile >= NUM_REFLOW_PROFILES) {
      activeReflowProfile = 0;
    }
  }
}

// Reflow oven controller state machine
void handleReflowState(void) {
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
    // If oven temperature is still above room temperature
    if (inputTemp >= TEMPERATURE_ROOM)
    {
      reflowState = REFLOW_STATE_TOO_HOT;
    }
    // If switch is pressed, start reflow process
    else if (startReflow)
    {
      // Turn off done LED if it was on from a previous cycle.
      digitalWrite(DONE_LED_PIN, LOW);
      // Reset switch state to prevent triggering later code erroneously.
      startReflow = false;
      // Initialize PID control window starting time
      windowStartTime = millis();
      // Ramp up to minimum soaking temperature
      setpoint = profiles[activeReflowProfile].soakTemp;
      // Tell the PID to range between 0 and the full window size
      reflowOvenPID.SetOutputLimits(0, windowSize);
      reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
      // Turn the PID on
      reflowOvenPID.SetMode(AUTOMATIC);
      // Proceed to preheat stage
      reflowState = REFLOW_STATE_PREHEAT;
    }
    break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    // If minimum soak temperature is achieved.
    if (inputTemp >= profiles[activeReflowProfile].soakTemp)
    {
      soakStartTime = millis();
      // Chop soaking period into smaller sub-period
      timerSoak = soakStartTime + profiles[activeReflowProfile].soakPeriodMS;
      // Set less agressive PID parameters for soaking ramp
      reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Proceed to soaking state
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + profiles[activeReflowProfile].soakPeriodMS;
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_COOL; 
    }
    break;

  case REFLOW_STATE_COOL:
    // If minimum cool temperature is achieve       
    if (inputTemp <= TEMPERATURE_COOL_MIN)
    {
      digitalWrite(DONE_LED_PIN, HIGH);
      completePeriod = millis() + 5000;
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;   
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_COMPLETE; 
    }         
    break;    

  case REFLOW_STATE_COMPLETE:
    if (millis() > completePeriod)
    {
      // Reflow process ended
      reflowState = REFLOW_STATE_IDLE; 
      activeReflowProfile = 0;
    }
    break;
  
  case REFLOW_STATE_TOO_HOT:
    // If oven temperature drops below room temperature
    if (inputTemp < TEMPERATURE_ROOM)
    {
      // Ready to reflow
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
    
  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
    if (isnan(inputTemp))
    {
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
  }    
}

// PID computation and SSR control
void handleSSR(void) {
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    { 
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime)) { 
      digitalWrite(SSR_PIN, HIGH);
    } else {
      digitalWrite(SSR_PIN, LOW);
    }
  } else {
    // Reflow oven process is off, ensure oven is off
    digitalWrite(SSR_PIN, LOW);
  }

}

void drawScreen(void) {
  u8g2.clearBuffer();
  short rowOffset = 0;
  const short rowSize = 12;
  
  // Temperature Row
  char temperatureStr[8];
  dtostrf(inputTemp, 4, 2, temperatureStr);
  u8g2.drawStr( 0, rowOffset, "Temp: ");
  u8g2.drawStr( 60, rowOffset, temperatureStr);
  u8g2.drawStr( 100, rowOffset, "C");
  rowOffset += rowSize;

  // Current Profile
  u8g2.drawStr( 0, rowOffset, profiles[activeReflowProfile].profileName);
  rowOffset += rowSize;
  
  // General status row
  u8g2.drawStr( 0, rowOffset, lcdMessagesReflowStatus[reflowState]);
  rowOffset += rowSize;

  // Timer for Heat cycle row
  if(reflowState == REFLOW_STATE_SOAK) {
    char soakSecondsStr[8];
    itoa((millis() - soakStartTime)/1000, soakSecondsStr, 10);
    u8g2.drawStr( 0, rowOffset, "Time: ");
    u8g2.drawStr( 40, rowOffset, soakSecondsStr);
    u8g2.drawStr( 80, rowOffset, "/1800");
  }
  rowOffset += rowSize;

  u8g2.sendBuffer();
}

void setup(void) {
  // Turn off SSR.
  digitalWrite(SSR_PIN, LOW);
  pinMode(SSR_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.begin();
  button.onPressed(buttonPressed);
  button.onPressedFor(1000, buttonHeld);

  digitalWrite(DONE_LED_PIN, LOW);
  pinMode(DONE_LED_PIN, OUTPUT);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2_prepare();

   // Set window size
  windowSize = 2000;
  // Initialize thermocouple reading variable
  nextRead = millis();
}

void loop(void) {
  if (millis() > nextRead) {
    readTemp();
  }

  handleReflowState();

  button.read();

  handleSSR();

  drawScreen();
}
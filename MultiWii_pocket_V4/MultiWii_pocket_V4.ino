/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
March  2013     V2.2
*/

#include <avr/io.h>

#include "config.h"
#include "def.h"


#include <avr/pgmspace.h>
#define  VERSION  222

/*********** RC alias *****************/
enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

enum box {
  BOXARM,
  #if ACC
    BOXANGLE,
    BOXHORIZON,
  #endif
  #ifdef VARIOMETER
    BOXVARIO,
  #endif
  #ifdef HEADFREE
    BOXHEADFREE,
  #endif
  #ifdef HEADHOLD
    BOXHEADHOLD,
  #endif
  #ifdef ACROTRAINER_MODE
    BOXACROTRN,
  #endif
  CHECKBOXITEMS
};

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #ifdef VARIOMETER
    "VARIO;"
  #endif
  #ifdef HEADFREE
    "HEADFREE;"
  #endif
  #ifdef HEADHOLD
    "HEADHOLD;"
  #endif
  #ifdef ACROTRAINER_MODE
    "ACROTRN;"
  #endif
;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  #if ACC
    1, //"ANGLE;"
    2, //"HORIZON;"
  #endif
  #ifdef VARIOMETER
    3, //"VARIO;"
  #endif
  4, //"HEADFREE;"
  5, //"HEADHOLD;"
  #ifdef ACROTRAINER_MODE
    7, //"ACROTRAINER;"
  #endif
};


static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint16_t acc_1G;            // this is the 1G measured acceleration
static uint16_t acc_25deg;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t  act_heading,magHold,headModeHold; // [-180;+180]
static uint8_t  vbat;                   // battery voltage in 0.1V steps
static uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  EstAlt,AltHold; // in cm
static int16_t  errorAltitudeI = 0;
static int16_t  vario = 0;              // variometer in cm/s

#if defined(ARMEDTIMEWARNING)
  static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];
static int16_t  sonarAlt; //to think about the unit

struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t HEADHOLD_MODE :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t VARIO_MODE :1 ;
  uint8_t ACROTRN_MODE :1 ;
} f;


static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[RC_CHANS];    // interval [1000;2000]
static int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
static uint16_t rssi;               // range: [0;1023]

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];

static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE ! 
} global_conf;


static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
  #if defined (FAILSAFE)
    int16_t failsafe_throttle;
  #endif
  #ifdef VBAT
    uint8_t vbatscale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;
    uint8_t no_vbat;
  #endif
  int16_t minthrottle;
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} conf;
 
static uint8_t alarmArray[16];           // array
 
void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) 
  {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  rcCommand[THROTTLE] = constrain(rcData[THROTTLE],MINCHECK,2000); //2000 ??


  #ifdef HEADFREE
  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (act_heading - headModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
    rcCommand[PITCH] = rcCommand_PITCH;
  }
  #endif

  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
    serialCom();
  #endif

  if (f.ARMED)  {
    #if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
      armedTime += (uint32_t)cycleTime;
    #endif
    #if defined(VBAT)
      if ( (vbat > conf.no_vbat) && (vbat < vbatMin) ) vbatMin = vbat;
    #endif
  }
}

static uint32_t PIDTime = 0;
static uint32_t rcTime  = 0;
static byte noRcDataSince  = 0;
static bool NewRC = false;

void setup() 
{
  SerialOpen(0,SERIAL0_COM_SPEED);
  #if defined(PROMICRO)
    SerialOpen(1,SERIAL1_COM_SPEED);
  #endif
  
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  
  initOutput();
  global_conf.currentSet=0;
  readEEPROM();
  readGlobalSet();
  readEEPROM();                                    // load current setting data
  blinkLED(2,40,global_conf.currentSet+1);          
  configureReceiver();
  initSensors();
  previousTime = micros();
  calibratingG = 768; // 513;
  
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  f.SMALL_ANGLES_25=1; // important for gyro only conf

  currentTime = micros();
  PIDTime = currentTime;
  rcTime  = currentTime;
  
  debugmsg_append_str("initialization completed\n");
}

void go_arm() 
{
  #if defined(FAILSAFE)
  if(calibratingG == 0 && f.ACC_CALIBRATED && failsafeCnt < 2)
  #else 
  if(calibratingG == 0 && f.ACC_CALIBRATED)
  #endif
  {
    if(!f.ARMED)
    { // arm now!
      f.ARMED = 1;
      headModeHold = act_heading;
      #if defined(VBAT)
        if (vbat > conf.no_vbat) vbatMin = vbat;
      #endif
    }
  } else if(!f.ARMED) { 
    blinkLED(2,255,1);
    alarmArray[8] = 1;
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
  }
}

// ******** Main Loop *********
void loop () 
{
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error;
  int16_t errorAngle;
  int16_t PTermACC = 0 , ITermACC = 0 , PTermGYRO = 0 , ITermGYRO = 0;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  static int16_t delta1[3],delta2[3];
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;

  currentTime = micros();

  if (currentTime >= rcTime) 
  { // 50Hz
    rcTime += CHECK_RCTIME; // check for new data every 2.3ms
    NewRC = computeRC();
    noRcDataSince++;
  }
  if (NewRC || (noRcDataSince > NORCDATA)) // 10 * 2300 = 23ms
  {  
    NewRC = false;
    noRcDataSince = 0;
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
        rcData[THROTTLE] = FAILSAFE_THROTTLE;
        if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
          go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
        }
        failsafeEvents++;
      }
      if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
          go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeCnt++;
    #endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for(i=0;i<4;i++) {
      stTmp >>= 2;
      if(rcData[i] > MINCHECK) stTmp |= 0x80;      // check for MIN
      if(rcData[i] < MAXCHECK) stTmp |= 0x40;      // check for MAX
    }
    if(stTmp == rcSticks) {
      if(rcDelayCommand<250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;
    
    // perform actions    
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if(rcDelayCommand == 20) {
      if(f.ARMED) {                   // actions during armed
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
        #endif
      } else {                        // actions during not armed
        i=0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG=512;
        }
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
          else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
        #endif
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA=512;     // throttle=max, yaw=left, pitch=min
        i=0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {conf.angleTrim[PITCH]+=8; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {conf.angleTrim[PITCH]-=8; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {conf.angleTrim[ROLL] +=8; i=1;}
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {conf.angleTrim[ROLL] -=8; i=1;}
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
        }
      }
    }
    
    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
    for(i=0;i<CHECKBOXITEMS;i++)
      rcOptions[i] = (auxState & conf.activate[i])>0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
    #if ACC
      if ( rcOptions[BOXANGLE] || (failsafeCnt > 5*FAILSAFE_DELAY) ) { 
        // bumpless transfer to Level mode
        if (!f.ANGLE_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.ANGLE_MODE = 1;
        }  
      } else {
        // failsafe support
        f.ANGLE_MODE = 0;
      }
      if ( rcOptions[BOXHORIZON] ) {
        f.ANGLE_MODE = 0;
        if (!f.HORIZON_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.HORIZON_MODE = 1;
        }
      } else {
        f.HORIZON_MODE = 0;
      }
    #endif

    #if defined(ACROTRAINER_MODE)
      if (rcOptions[BOXACROTRN] == 1) f.ACROTRN_MODE = 1; else f.ACROTRN_MODE = 0;
    #endif
    #ifdef HEADFREE
      if (rcOptions[BOXHEADFREE] == 1) f.HEADFREE_MODE = 1; else f.HEADFREE_MODE = 0;
    #endif
    #ifdef HEADHOLD
      if (rcOptions[BOXHEADHOLD] == 1) f.HEADHOLD_MODE = 1; else f.HEADHOLD_MODE = 0;
    #endif
    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
    if (f.ANGLE_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}

  } 
  else if (currentTime >= PIDTime)
  {
    PIDTime += PIDCALCTIME;
  
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;
    
    computeIMU();

    //***********************************
    //**** Experimental FlightModes *****
    //***********************************
    #if defined(ACROTRAINER_MODE)
    if(f.ACROTRN_MODE)
    {
      if(f.ANGLE_MODE){
        if (abs(rcCommand[ROLL]) + abs(rcCommand[PITCH]) >= ACROTRAINER_MODE ) 
        {
          f.ANGLE_MODE=0;
          f.HORIZON_MODE=0;
        }
      }
    }
    #endif

    #ifdef HEADHOLD
    if(f.HEADHOLD_MODE) 
    { //to optimize
      errorAngle = headModeHold - act_heading;
      if (errorAngle >  180) errorAngle -= 360;
      if (errorAngle < -180) errorAngle += 360;
      errorAngle = constrain(errorAngle,-100,100);
      rcCommand[YAW] += (errorAngle*conf.P8[PIDMAG]) >> 4;
    }
    #endif
  
    //**** PITCH & ROLL & YAW PID ****
    //----------PID controller----------
    
    int16_t prop;
    prop = min(max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])),500); // range [0;500]
    
   
    {
      // angle is deg * 10
      int16_t tiltfact;
      tiltfact = (ACCRESO - accSmooth[2]) >> 2; // scale accreso 1024
      tiltfact = constrain(tiltfact, -200, +200);
      tiltfact = (tiltfact *  conf.P8[PIDALT]) >> 7;
      rcCommand[THROTTLE] += tiltfact; 
    }
    
    //----------PID controller----------
    for(axis=0;axis<3;axis++) 
    {
      //-----Get the desired angle rate depending on flight mode
      if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2 ) 
      { // MODE relying on ACC
        // calculate error and limit the angle to 50 degrees max inclination
        errorAngle = constrain((rcCommand[axis]<<1),-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
        PTermACC = ((int32_t)errorAngle*conf.P8[PIDLEVEL])>>7;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
        PTermACC = constrain(PTermACC,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);
        
        errorAngleI[axis]     = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
        ITermACC  = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
      }
      if ( !f.ANGLE_MODE || f.HORIZON_MODE || axis == 2 ) 
      { // MODE relying on GYRO or YAW axis
        error = ((int32_t)rcCommand[axis]<<6)/conf.P8[axis] ; // 32 bits is needed for calculation
        error -= gyroData[axis]/4;  // error = rcCommand - gyro
        
        PTermGYRO = rcCommand[axis];
      
        errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);         // WindUp   16 bits is ok here
        if (abs(gyroData[axis]>>2)>640) errorGyroI[axis] = 0;
        ITermGYRO = ((errorGyroI[axis]>>7)*conf.I8[axis])>>6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
      }
      
      if (axis<2)
      {
        if ( f.HORIZON_MODE ) 
        {
          PTerm = ((int32_t)PTermACC*(512-prop) + (int32_t)PTermGYRO*prop)>>9;         // the real factor should be 500, but 512 is ok
          ITerm = ((int32_t)ITermACC*(512-prop) + (int32_t)ITermGYRO*prop)>>9;
        } 
        else if ( f.ANGLE_MODE ) 
        {
          PTerm = PTermACC;
          ITerm = ITermACC;
        } 
        else // Acro
        {
          PTerm = PTermGYRO;
          ITerm = ITermGYRO;
        }
      }
      else // yaw axis
      {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }

      PTerm -= ((int32_t)gyroData[axis]*dynP8[axis])>>8; // 32 bits is needed for calculation   

      delta          = gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
      lastGyro[axis] = gyroData[axis];
      deltaSum       = delta1[axis]+delta2[axis]+delta;
      delta2[axis]   = delta1[axis];
      delta1[axis]   = delta;
 
      DTerm = ((int32_t)deltaSum*dynD8[axis])>>7;        // 32 bits is needed for calculation
                      
      axisPID[axis] =  PTerm + ITerm - DTerm;
    }
    
    mixTable();
    writeMotors();
  }
    
}

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) 
{
  uint8_t i,r;
  for (r=0;r<repeat;r++) 
  {
    for(i=0;i<num;i++) 
    {
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
    }
    delay(60); //wait 60 ms
  }
}

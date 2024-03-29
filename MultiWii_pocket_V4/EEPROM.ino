#include <avr/eeprom.h>


uint8_t calculate_sum(uint8_t *cb , uint8_t siz) {
  uint8_t sum=0x55;  // checksum init
  while(--siz) sum += *cb++;  // calculate checksum (without checksum byte)
  return sum;
}

void readGlobalSet() {
  eeprom_read_block((void*)&global_conf, (void*)0, sizeof(global_conf));
  if(calculate_sum((uint8_t*)&global_conf, sizeof(global_conf)) != global_conf.checksum) {
    global_conf.currentSet = 0;
    global_conf.accZero[ROLL] = 5000;    // for config error signalization
  }
}
 
void readEEPROM() {
  uint8_t i;
  #ifdef MULTIPLE_CONFIGURATION_PROFILES
    if(global_conf.currentSet>2) global_conf.currentSet=0;
  #else
    global_conf.currentSet=0;
  #endif
  eeprom_read_block((void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  if(calculate_sum((uint8_t*)&conf, sizeof(conf)) != conf.checksum) {
    blinkLED(6,100,3);    
    #if defined(BUZZER)
      alarmArray[7] = 3;
    #endif
    LoadDefaults();                 // force load defaults 
  }
  for(i=0;i<6;i++) {
    lookupPitchRollRC[i] = (2500+conf.rcExpo8*(i*i-25))*i*(int32_t)conf.rcRate8/2500;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = conf.minthrottle + (int32_t)(MAXTHROTTLE-conf.minthrottle)* lookupThrottleRC[i]/1000;            // [0;1000] -> [conf.minthrottle;MAXTHROTTLE]
  }

  #if defined(POWERMETER)
    pAlarm = (uint32_t) conf.powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) conf.pleveldiv; // need to cast before multiplying
  #endif
  #ifdef FLYING_WING
    #ifdef LCD_CONF
      conf.wing_left_mid  = constrain(conf.wing_left_mid, WING_LEFT_MIN,  WING_LEFT_MAX); //LEFT
      conf.wing_right_mid = constrain(conf.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    #else // w.o LCD support user may not find this value stored in eeprom, so always use the define value
      conf.wing_left_mid  = WING_LEFT_MID;
      conf.wing_right_mid = WING_RIGHT_MID;
    #endif
  #endif
  #ifdef TRI
    #ifdef LCD_CONF
      conf.tri_yaw_middle = constrain(conf.tri_yaw_middle, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
    #else // w.o LCD support user may not find this value stored in eeprom, so always use the define value
      conf.tri_yaw_middle = TRI_YAW_MIDDLE;
    #endif
  #endif
}

void writeGlobalSet(uint8_t b) {
  global_conf.checksum = calculate_sum((uint8_t*)&global_conf, sizeof(global_conf));
  eeprom_write_block((const void*)&global_conf, (void*)0, sizeof(global_conf));
  if (b == 1) blinkLED(15,20,1);
  #if defined(BUZZER)
    alarmArray[7] = 1; 
  #endif

}
 
void writeParams(uint8_t b) {
  global_conf.currentSet=0;
  conf.checksum = calculate_sum((uint8_t*)&conf, sizeof(conf));
  eeprom_write_block((const void*)&conf, (void*)(global_conf.currentSet * sizeof(conf) + sizeof(global_conf)), sizeof(conf));
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
  #if defined(BUZZER)
    alarmArray[7] = 1; //beep if loaded from gui or android
  #endif
}

void LoadDefaults() 
{
  conf.P8[ROLL]     = 26; conf.I8[ROLL]     = 30; conf.D8[ROLL]     = 23;
  conf.P8[PITCH]    = 26; conf.I8[PITCH]    = 30; conf.D8[PITCH]    = 23;

  conf.P8[PIDLEVEL] = 60; conf.I8[PIDLEVEL] = 10; conf.D8[PIDLEVEL] = 100;

  conf.P8[YAW]      = 50;  conf.I8[YAW]     = 77; conf.D8[YAW]      = 0;
  conf.P8[PIDALT]   = 90;  conf.I8[PIDALT]  = 10; conf.D8[PIDALT]   = 100;
  
  conf.P8[PIDPOS]  = 0;    conf.I8[PIDPOS]   = 0; conf.D8[PIDPOS]    = 0;
  conf.P8[PIDPOSR] = 0;    conf.I8[PIDPOSR]  = 0; conf.D8[PIDPOSR]   = 0;
  conf.P8[PIDNAVR] = 0;    conf.I8[PIDNAVR]  = 0; conf.D8[PIDNAVR]   = 0;

  conf.P8[PIDMAG]   = 40;
  
  conf.P8[PIDVEL] = 0;      conf.I8[PIDVEL] = 0;    conf.D8[PIDVEL] = 0;
  
  conf.rcRate8 = 100; conf.rcExpo8 = 40;
  conf.rollPitchRate = 65;
  conf.yawRate = 55;
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; conf.thrExpo8 = 0;
  for(uint8_t i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
  conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
  conf.powerTrigger1 = 0;
  #ifdef FLYING_WING
    conf.wing_left_mid  = WING_LEFT_MID; 
    conf.wing_right_mid = WING_RIGHT_MID; 
  #endif
  #ifdef FIXEDWING
    conf.dynThrPID = 50;
    conf.rcExpo8   =  0;
  #endif
  #ifdef TRI
    conf.tri_yaw_middle = TRI_YAW_MIDDLE;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
    {
      int16_t s[8] = SERVO_OFFSET;
      for(uint8_t i=0;i<8;i++) conf.servoTrim[i] = s[i];
    }
  #endif
  #if defined (FAILSAFE)
    conf.failsafe_throttle = FAILSAFE_THROTTLE;
  #endif
  #ifdef VBAT
    conf.vbatscale = VBATSCALE;
    conf.vbatlevel_warn1 = VBATLEVEL_WARN1;
    conf.vbatlevel_warn2 = VBATLEVEL_WARN2;
    conf.vbatlevel_crit = VBATLEVEL_CRIT;
    conf.no_vbat = NO_VBAT;
  #endif
  conf.minthrottle = MINTHROTTLE;
  #ifdef GOVERNOR_P
    conf.governorP = GOVERNOR_P;
    conf.governorD = GOVERNOR_D;
    conf.governorR = GOVERNOR_R;
  #endif
  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
}



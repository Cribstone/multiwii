/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be tweaked via serial monitor or LCD.
 *    Changing those values in config.h and upload will require a 'Reset' from the GUI to take effect
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    The type of multicopter    ****************************/
    //#define QUADP
    #define QUADX

  /***************************    Combined IMU Boards    ********************************/

    #define HK_POCKETQUAD   // HobbyKing pocketquad
    
    //#define HEADFREE
    #define HEADHOLD
    
  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
    #define MINTHROTTLE 1050 // (*)

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 2020

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    #define MINCOMMAND  1000

  /**********************************    I2C speed   ************************************/
    //#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
    #define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

  /***************************    Internal i2c Pullups   ********************************/
    /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
    //#define INTERNAL_I2C_PULLUPS
    
  /***************************    Timing of the actions  ********************************/
    #define CHECK_RCTIME 2300 // in us, interval to check for new RC data
    #define NORCDATA     10   // Assume missed RC signal after NORCDATASINCE * CHECK_RCTIME
    #define PIDCALCTIME 3000  // in us, Time interval for PID calculation
    #define SCALECORR 715.0f  // PIDCALCTIME / 4
    #define ACCRESO 4096  //acceleration resolution

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

    #define YAW_DIRECTION 1
    //#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction

   /********************************    ARM/DISARM    *********************************/
   /* optionally disable stick combinations to arm/disarm the motors.
     * In most cases one of the two options to arm/disarm via TX stick is sufficient */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* note: no need to uncomment something in this section if you use a standard receiver */

  /**************************************************************************************/
  /********                       special receiver types             ********************/
  /**************************************************************************************/

    /**********************    Spektrum Satellite Reciver    *******************************/
      /* The following lines apply only for Spektrum Satellite Receiver
         Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
         For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
         For PROMINI, attach sat grey to RX0.  Attach sat black to ground. */
      #define SPEKTRUM 1024
      //#define SPEKTRUM 2048
      #define SPEK_SERIAL_PORT 1    // Forced to 0 on Pro Mini and single serial boards; Set to your choice of 0, 1, or 2 on any Mega based board (defaults to 1 on Mega).
      
    /*******************************    SBUS RECIVER    ************************************/
      /* The following line apply only for Futaba S-Bus Receiver on MEGA boards at RX1 only (Serial 1).
         You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
      //#define SBUS
      #define SBUS_SERIAL_PORT 1
      #define SBUS_OFFSET 988


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  4 - ALTERNATE CPUs & BOARDS                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********   Settings for ProMicro, Leonardo and other Atmega32u4 Boards     ***********/
  /**************************************************************************************/

    /*********************************    pin Layout     **********************************/
      /* activate this for a better pinlayout if all pins can be used => not possible on ProMicro */
      #define A32U4ALLPINS

  /**************************************************************************************/
  /********                      override default pin assignments    ********************/
  /**************************************************************************************/

  /* only enable any of this if you must change the default pin assignment, e.g. your board does not have a specific pin */
  /* you may need to change PINx and PORTx plus #shift according to the desired pin! */
  //#define OVERRIDE_V_BATPIN                   A0 // instead of A3    // Analog PIN 3

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200

    /* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
       if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
       it is relevent only for a conf with NK */
    #define INTERLEAVING_DELAY 3000

    /* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
       it is relevent only for a conf with at least a WMP */
    #define NEUTRALIZE_DELAY 100000


  /**************************************************************************************/
  /********                              Gyro filters                ********************/
  /**************************************************************************************/

        #define GYRO_I_MAX 256
        #define ACC_I_MAX 256

   /*********************    Lowpass filter for some gyros    ****************************/
      /* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
         to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
         It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
         balancing options ran out. Uncomment only one option!
         IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
      //#define MPU6050_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
      //#define MPU6050_LPF_188HZ
      //#define MPU6050_LPF_98HZ
      #define MPU6050_LPF_42HZ
      //#define MPU6050_LPF_20HZ
      //#define MPU6050_LPF_10HZ
      //#define MPU6050_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************   Assisted AcroTrainer    ************************************/
    /* Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
       Remember to activate ANGLE_MODE first!...
       A Value on 200 will give a very distinct transfer */
    //#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC or nunchuk is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THR0TTLE value. You must set this value to descending about 1m/s or so 
       for best results. This value is depended from your configuration, AUW and some other params.  Next, afrer FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
    #define FAILSAFE                                // uncomment  to activate the failsafe function
    #define FAILSAFE_DELAY     5                    // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
    #define FAILSAFE_OFF_DELAY 20                   // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)  // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case


  /**************************************************************************************/
  /***********************                  TX-related         **************************/
  /**************************************************************************************/

  /********************************************************************/
  /****           battery voltage monitoring                       ****/
  /********************************************************************/
    /* for V BAT monitoring
       after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
       with R1=33k and R2=51k
       vbat = [0;1023]*16/VBATSCALE
       must be associated with #define BUZZER ! */
    //#define VBAT              // uncomment this line to activate the vbat code
    #define VBATSCALE       131 // (*) change this value if readed Battery voltage is different than real voltage
    #define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
    #define VBATLEVEL_WARN1 107 // (*) 10,7V
    #define VBATLEVEL_WARN2  99 // (*) 9.9V
    #define VBATLEVEL_CRIT   93 // (*) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
    #define NO_VBAT          16  // (*) Avoid beeping without any battery


  /********************************************************************/
  /****           baord naming                                     ****/
  /********************************************************************/

    /*
     * this name is displayed together with the MultiWii version number
     * upon powerup on the LCD.
     * If you are without a DISPLAYD then You may enable LCD_TTY and
     * use arduino IDE's serial monitor to view the info.
     *
     * You must preserve the format of this string!
     * It must be 16 characters total,
     * The last 4 characters will be overwritten with the version number.
     */
    #define BOARD_NAME "MultiWii   V-.--"
    //                  123456789.123456

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/
    
  /**************************************************************************************/
  /********   special ESC with extended range [0-2000] microseconds  ********************/
  /**************************************************************************************/
    // the only options for 328 processors are 1, 4, 32 khz
    
    //#define EXT_MOTOR_64KHZ // only 255 steps + very risky. Really want it?
    //#define EXT_MOTOR_32KHZ // only 511 steps + on your own risk!
    //#define EXT_MOTOR_16KHZ 
    #define EXT_MOTOR_8KHZ  // i recommend this one for a smooth flight
    //#define EXT_MOTOR_4KHZ  // only 328 device

  /**************************************************************************************/
  /***********************     motor, servo and other presets     ***********************/
  /**************************************************************************************/
    /* motors will not spin when the throttle command is in low position
       this is an alternative method to stop immediately the motors */
    //#define MOTOR_STOP

    /* some radios have not a neutral point centered on 1500. can be changed here */
    #define MIDRC 1500

  /********************************************************************/
  /****           Serial command handling - MSP and other          ****/
  /********************************************************************/

    /* to reduce memory footprint, it is possible to suppress handling of serial commands.
     * This does _not_ affect handling of RXserial, Spektrum or GPS. Those will not be affected and still work the same.
     * Enable either one or both of the following options  */

    /* Remove handling of all commands of the New MultiWii Serial Protocol.
     * This will disable use of the GUI, winGUI, android apps and any other program that makes use of the MSP.
     * You must find another way (like LCD_CONF) to tune the parameters or live with the defaults.
     * If you run a LCD/OLED via i2c or serial/Bluetooth, this is safe to use */
    //#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes

    /* Remove handling of other serial commands.
     * This includes navigating via serial the lcd.configuration menu, lcd.telemetry and permanent.log .
     * Navigating via stick inputs on tx is not affected and will work the same.  */
    #define SUPPRESS_OTHER_SERIAL_COMMANDS // saves  approx 0 to 100 bytes, depending on features enabled


/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/

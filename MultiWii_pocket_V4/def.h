

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif
#if defined(__AVR_ATmega32U4__)
  #define PROMICRO
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif

#if (FAILSAFE_DELAY == 0) 
  #error "FAILSAFE_DELAY cant be 0"
#endif

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/

#if defined(HK_POCKETQUAD)
  #define HWPWM6
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -X; accADC[PITCH]  = -Y; accADC[YAW]  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = Y; gyroADC[PITCH] =  -X; gyroADC[YAW] = -Z;}
#endif

#if defined(CRIUS_SE)
  #define ITG3200
  #define BMA180
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#endif

/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/

#if defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
  #define NUMBER_MOTOR     4
#endif

/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);
  #if !defined(RCAUXPIN12) && !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
    #define POWERPIN_ON                PORTB |= 1<<4;
    #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #define SERVO_8_PINMODE            pinMode(9,OUTPUT); // new
  #if defined(RCAUXPIN12)
    #define RCAUXPIN
  #endif
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #if !defined(MONGOOSE1_0)
    #define PINMODE_LCD                pinMode(0, OUTPUT);
    #define LCDPIN_OFF                 PORTD &= ~1; //switch OFF digital PIN 0
    #define LCDPIN_ON                  PORTD |= 1;
    #define STABLEPIN_PINMODE          ;
    #define STABLEPIN_ON               ;
    #define STABLEPIN_OFF              ;
  #endif 
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define SPEK_SERIAL_PORT           0
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
    
  #define PCINT_PIN_COUNT            5
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define PCINT_RX_PORT              PORTD
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PIND
  #define V_BATPIN                   A3    // Analog PIN 3
  #define PSENSORPIN                 A2    // Analog PIN 2
#endif

/**************************  atmega32u4 (Promicro)  ***********************************/
#if defined(PROMICRO)
  #if defined(MICROWII)
    #define A32U4ALLPINS 
  #endif
  #define LEDPIN_PINMODE             //
  #define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
  #define LEDPIN_OFF                 PORTD |= (1<<5);
  #define LEDPIN_ON                  PORTD &= ~(1<<5);  
  #if defined(A32U4ALLPINS)
    #define BUZZERPIN_PINMODE          DDRD |= (1<<4);
    #define BUZZERPIN_ON               PORTD |= 1<<4;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<4);  
  #else
    #define BUZZERPIN_PINMODE          DDRD |= (1<<3);
    #define BUZZERPIN_ON               PORTD |= 1<<3;
    #define BUZZERPIN_OFF              PORTD &= ~(1<<3); 
  #endif
  #define POWERPIN_PINMODE           //
  #define POWERPIN_ON                //
  #define POWERPIN_OFF               //
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                DDRD |= (1<<2);
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6);EIMSK |= (1 << INT6);EICRB |= (1 << ISC61)|(1 << ISC60);
  #if !defined(SPEK_SERIAL_PORT)
    #define SPEK_SERIAL_PORT         1
  #endif
  #define USB_CDC_TX                 3
  #define USB_CDC_RX                 2
      
  //Standart RX
  #define THROTTLEPIN                  3
  #if defined(A32U4ALLPINS)
    #define ROLLPIN                    6
    #define PITCHPIN                   2
    #define YAWPIN                     4
    #define AUX1PIN                    5
  #else
    #define ROLLPIN                    4
    #define PITCHPIN                   5
    #define YAWPIN                     2
    #define AUX1PIN                    6
  #endif
  #define AUX3PIN                      1 // unused 
  #define AUX4PIN                      0 // unused 
  #if !defined(RCAUX2PIND17)
    #define PCINT_PIN_COUNT          4
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
  #else
    #define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
  #endif
  #define PCINT_RX_PORT                PORTB
  #define PCINT_RX_MASK                PCMSK0
  #define PCIR_PORT_BIT                (1<<0)
  #define RX_PC_INTERRUPT              PCINT0_vect
  #define RX_PCINT_PIN_PORT            PINB

  #if !defined(A32U4ALLPINS)
    #define V_BATPIN                  A3    // Analog PIN 3
  #elif defined(A32U4ALLPINS)
    #define V_BATPIN                  A4    // Analog PIN 4
  #endif
  #define PSENSORPIN                A2    // Analog PIN 2 
#endif

/**************************  all the Mega types  ***********************************/
#if defined(MEGA)
  #define LEDPIN_PINMODE             ;
  #define LEDPIN_TOGGLE              ;
  #define LEDPIN_ON                  ;
  #define LEDPIN_OFF                 ;
  #define POWERPIN_PINMODE           ;
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define I2C_PULLUPS_ENABLE         ;     
  #define I2C_PULLUPS_DISABLE        ;
  #define PINMODE_LCD                ;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
#endif

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/


#if defined(NANOWII)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  =  X; accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -X; gyroADC[PITCH] = -Y; gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
  // move motor 7 & 8 to pin 4 & A2
  #define SOFT_PWM_3_PIN_HIGH        PORTD |= 1<<4;
  #define SOFT_PWM_3_PIN_LOW         PORTD &= ~(1<<4);
  #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<5;
  #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<5);
  #define SW_PWM_P3                  4        
  #define SW_PWM_P4                  A2
  #define HWPWM6
  // move servo 3 & 4 to pin 13 & 11
  #define SERVO_3_PINMODE   DDRC |= (1<<7); // 13
  #define SERVO_3_PIN_HIGH  PORTC |= 1<<7;
  #define SERVO_3_PIN_LOW   PORTC &= ~(1<<7);
  #define SERVO_4_PINMODE   DDRB |= (1<<7); // 11
  #define SERVO_4_PIN_HIGH  PORTB |= 1<<7;
  #define SERVO_4_PIN_LOW   PORTB &= ~(1<<7);
  // use pin 4 as status LED output if we have no octo
  #if !defined(OCTOX8) && !defined(OCTOFLATP) && !defined(OCTOFLATX)
    #define LEDPIN_PINMODE             DDRD |= (1<<4);            //D4 to output
    #define LEDPIN_TOGGLE              PIND |= (1<<5)|(1<<4);     //switch LEDPIN state (Port D5) & pin D4
    #define LEDPIN_OFF                 PORTD |= (1<<5); PORTD &= ~(1<<4);
    #define LEDPIN_ON                  PORTD &= ~(1<<5); PORTD |= (1<<4);  
  #endif
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(MMA8451Q) || defined(NUNCHUCK)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(MPU3050) || defined(WMP)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_PROMINI_SERIAL) && defined(PROMINI)
  #define GPS_SERIAL 0
  #define GPS_PROMINI
  #define GPS_BAUD   GPS_PROMINI_SERIAL
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD) || defined(TINY_GPS)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(TINY_GPS_SONAR)
  #define SONAR 1
#else
  #define SONAR 0
#endif


/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#if defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(VTAIL4)
 #define MULTITYPE 17
#endif

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/

//all new Special RX's must be added here
//this is to avoid confusion :)
#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(RCSERIAL)
  #error "no STANDARD_RX in this version"
#endif


// Spektrum Satellite
#if defined(SPEKTRUM)
  #define SPEK_FRAME_SIZE 16
  #if (SPEKTRUM == 1024)
    #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_DATA_SHIFT          // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_BIND_PULSES 3
  #endif
  #if (SPEKTRUM == 2048)
    #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_DATA_SHIFT >> 1     // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_BIND_PULSES 5
  #endif
#endif

#if defined(SBUS)
  #define RC_CHANS 12
#elif defined(SPEKTRUM)  
  #define RC_CHANS 12
#else
  #define RC_CHANS 8
#endif

#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
  #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40
#endif 

/**************************************************************************************/
/********* enforce your sensors orientation - possibly overriding board defaults  *****/
/**************************************************************************************/
#ifdef FORCE_GYRO_ORIENTATION
  #define GYRO_ORIENTATION FORCE_GYRO_ORIENTATION
#endif
#ifdef FORCE_ACC_ORIENTATION
  #define ACC_ORIENTATION FORCE_ACC_ORIENTATION
#endif
#ifdef FORCE_MAG_ORIENTATION
  #define MAG_ORIENTATION FORCE_MAG_ORIENTATION
#endif

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#ifndef NUMBER_MOTOR
        #error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if defined(POWERMETER) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif



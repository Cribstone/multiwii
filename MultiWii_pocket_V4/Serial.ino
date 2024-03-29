#if defined(MEGA)
  #define UART_NUMBER 4
#elif defined(PROMICRO)
  #define UART_NUMBER 2
#else
  #define UART_NUMBER 1
#endif
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

// Capability is bit flags; next defines should be 2, 4, 8...

const uint32_t PROGMEM capability = 0;

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

#if defined(PROMINI)
  #define CURRENTPORT 0
#else
  static uint8_t CURRENTPORT=0;
#endif

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]);UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() 
{
  uint8_t c,n;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for(n=0;n<UART_NUMBER;n++) {
    {
      uint8_t num_avail = SerialAvailable(CURRENTPORT);
      while (num_avail--) 
      {
        uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[CURRENTPORT]-serialTailTX[CURRENTPORT]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
        if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
        c = SerialRead(CURRENTPORT);
        #ifdef SUPPRESS_ALL_SERIAL_MSP
          // no MSP handling, so go directly
        #else
          // regular data handling to detect and handle MSP and other data
          if (c_state[CURRENTPORT] == IDLE) {
            c_state[CURRENTPORT] = (c=='$') ? HEADER_START : IDLE;
          } else if (c_state[CURRENTPORT] == HEADER_START) {
            c_state[CURRENTPORT] = (c=='M') ? HEADER_M : IDLE;
          } else if (c_state[CURRENTPORT] == HEADER_M) {
            c_state[CURRENTPORT] = (c=='<') ? HEADER_ARROW : IDLE;
          } else if (c_state[CURRENTPORT] == HEADER_ARROW) {
            if (c > INBUF_SIZE) {  // now we are expecting the payload size
              c_state[CURRENTPORT] = IDLE;
              continue;
            }
            dataSize[CURRENTPORT] = c;
            offset[CURRENTPORT] = 0;
            checksum[CURRENTPORT] = 0;
            indRX[CURRENTPORT] = 0;
            checksum[CURRENTPORT] ^= c;
            c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
          } else if (c_state[CURRENTPORT] == HEADER_SIZE) {
            cmdMSP[CURRENTPORT] = c;
            checksum[CURRENTPORT] ^= c;
            c_state[CURRENTPORT] = HEADER_CMD;
          } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
            checksum[CURRENTPORT] ^= c;
            inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
          } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
            if (checksum[CURRENTPORT] == c) {  // compare calculated and transferred checksum
              evaluateCommand();  // we got a valid packet, evaluate it
            }
            c_state[CURRENTPORT] = IDLE;
            num_avail = 0;
          }
        #endif // SUPPRESS_ALL_SERIAL_MSP
      }
    }
  }
}
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand() {
  switch(cmdMSP[CURRENTPORT]) {
   case MSP_SET_RAW_RC:
     for(uint8_t i=0;i<8;i++) {
       rcData[i] = read16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_PID:
     for(uint8_t i=0;i<PIDITEMS;i++) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(0);
     break;
   case MSP_SET_BOX:
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       conf.activate[i]=read16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(0);
     break;
   case MSP_SET_MISC:
     #if defined(POWERMETER)
       conf.powerTrigger1 = read16() / PLEVELSCALE;
     #endif
     headSerialReply(0);
     break;
   case MSP_SET_HEAD:
     magHold = read16();
     headSerialReply(0);
     break;
   case MSP_IDENT:
     headSerialReply(7);
     serialize8(VERSION);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32(pgm_read_dword(&(capability)));        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(11);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize32(
                 #if ACC
                   f.ANGLE_MODE<<BOXANGLE|
                   f.HORIZON_MODE<<BOXHORIZON|
                 #endif
                 #if defined(HEADFREE)
                   f.HEADFREE_MODE<<BOXHEADFREE|
                 #endif
                 #if defined(HEADHOLD)
                   f.HEADHOLD_MODE<<BOXHEADHOLD|
                 #endif
                 #if defined(VARIOMETER)
                   rcOptions[BOXVARIO]<<BOXVARIO |
                 #endif
                 #if defined(ACROTRAINER_MODE)
                   rcOptions[BOXACROTRN]<<BOXACROTRN |
                 #endif
                 f.ARMED<<BOXARM);
       serialize8(global_conf.currentSet);   // current setting
     break;
   case MSP_RAW_IMU:
     headSerialReply(18);
     for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]>>3);
     for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]>>2);
     for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
     break;
   case MSP_SERVO:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++)
       #if defined(SERVO)
       serialize16(servo[i]);
       #else
       serialize16(0);
       #endif
     break;
   case MSP_MOTOR:
     headSerialReply(16);
     for(uint8_t i=0;i<8;i++) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(RC_CHANS * 2);
     for(uint8_t i=0;i<RC_CHANS;i++) serialize16(rcData[i]);
     break;
   case MSP_ATTITUDE:
     headSerialReply(8);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16(act_heading);
     serialize16(headModeHold);
     break;
   case MSP_ALTITUDE:
     headSerialReply(6);
     serialize32(EstAlt);
     serialize16(vario);                  // added since r1172
     break;
   case MSP_ANALOG:
     headSerialReply(5);
     serialize8(vbat);
     serialize16(0);
     serialize16(rssi);
     break;
   case MSP_RC_TUNING:
     headSerialReply(7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for(uint8_t i=0;i<PIDITEMS;i++) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
   case MSP_PIDNAMES:
     headSerialReply(strlen_P(pidnames));
     serializeNames(pidnames);
     break;
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
     headSerialReply(strlen_P(boxnames));
     serializeNames(boxnames);
     break;
   case MSP_BOXIDS:
     headSerialReply(CHECKBOXITEMS);
     for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
       serialize8(pgm_read_byte(&(boxids[i])));
     }
     break;
   case MSP_MISC:
     headSerialReply(2);
     serialize16(1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for(uint8_t i=0;i<8;i++) {
       serialize8(PWM_PIN[i]);
     }
     break;
   case MSP_RESET_CONF:
     if(!f.ARMED) LoadDefaults();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     if(!f.ARMED) calibratingA=512;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     if(!f.ARMED) f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     headSerialReply(8);
     for(uint8_t i=0;i<4;i++) {
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   #ifdef DEBUGMSG
   case MSP_DEBUGMSG:
     {
       uint8_t size = debugmsg_available();
       if (size > 16) size = 16;
       headSerialReply(size);
       debugmsg_serialize(size);
     }
     break;
   #endif
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}
#endif // SUPPRESS_ALL_SERIAL_MSP

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX[CURRENTPORT];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][CURRENTPORT] = a;
  checksum[CURRENTPORT] ^= a;
  serialHeadTX[CURRENTPORT] = t;
}

#if defined(PROMINI) || defined(MEGA)
  #if defined(PROMINI)
  ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
  #endif
  #if defined(MEGA)
  ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA
  #endif
    uint8_t t = serialTailTX[0];
    if (serialHeadTX[0] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
      serialTailTX[0] = t;
    }
    if (t == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif
#if defined(MEGA) || defined(PROMICRO)
  ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
    uint8_t t = serialTailTX[1];
    if (serialHeadTX[1] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
      serialTailTX[1] = t;
    }
    if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
  }
#endif
#if defined(MEGA)
  ISR(USART2_UDRE_vect) { // Serial 2 on a MEGA
    uint8_t t = serialTailTX[2];
    if (serialHeadTX[2] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR2 = serialBufferTX[t][2];
      serialTailTX[2] = t;
    }
    if (t == serialHeadTX[2]) UCSR2B &= ~(1<<UDRIE2);
  }
  ISR(USART3_UDRE_vect) { // Serial 3 on a MEGA
    uint8_t t = serialTailTX[3];
    if (serialHeadTX[3] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR3 = serialBufferTX[t][3];
      serialTailTX[3] = t;
    }
    if (t == serialHeadTX[3]) UCSR3B &= ~(1<<UDRIE3);
  }
#endif

void UartSendData() {
  #if defined(PROMINI)
    UCSR0B |= (1<<UDRIE0);
  #endif
  #if defined(PROMICRO)
    switch (CURRENTPORT) {
      case 0:
        while(serialHeadTX[0] != serialTailTX[0]) {
           if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
           #if !defined(TEENSY20)
             USB_Send(USB_CDC_TX,serialBufferTX[serialTailTX[0]],1);
           #else
             Serial.write(serialBufferTX[serialTailTX[0]],1);
           #endif
         }
        break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
    }
  #endif
  #if defined(MEGA)
    switch (CURRENTPORT) {
      case 0: UCSR0B |= (1<<UDRIE0); break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
      case 2: UCSR2B |= (1<<UDRIE2); break;
      case 3: UCSR3B |= (1<<UDRIE3); break;
    }
  #endif
}

static void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    #endif
    #if defined(PROMICRO)
      #if (ARDUINO >= 100) && !defined(TEENSY20)
        case 0: UDIEN &= ~(1<<SOFE); break;// disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
      #endif
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
      case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
      case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
    #endif
  }
}

static void inline SerialEnd(uint8_t port) {
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    #endif
    #if defined(PROMICRO)
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
      case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2)); break;
      case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3)); break;
    #endif
  }
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) 
{
  #if defined(SPEKTRUM)
    if (portnum == SPEK_SERIAL_PORT) 
    {
      spektrumDataReceive(data);
      return;
    }
  #elif defined(SBUS)
    if (portnum == SBUS_SERIAL_PORT) 
    {
      sbusDataReceive(data);
      return;
    }
  #endif
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;  
  serialHeadRX[portnum] = h;
}

#if defined(PROMINI)
  ISR(USART_RX_vect)  { store_uart_in_buf(UDR0, 0); }
#endif
#if defined(PROMICRO)
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
#endif
#if defined(MEGA)
  ISR(USART0_RX_vect)  { store_uart_in_buf(UDR0, 0); }
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
  ISR(USART2_RX_vect)  { store_uart_in_buf(UDR2, 2); }
  ISR(USART3_RX_vect)  { store_uart_in_buf(UDR3, 3); }
#endif

uint8_t SerialRead(uint8_t port) {
  #if defined(PROMICRO)
    #if defined(TEENSY20)
      if(port == 0) return Serial.read();
    #else
      #if (ARDUINO >= 100)
        if(port == 0) USB_Flush(USB_CDC_TX);
      #endif
      if(port == 0) return USB_Recv(USB_CDC_RX);      
    #endif
  #endif
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

#if defined(SPEKTRUM)
  uint8_t SerialPeek(uint8_t port) {
    uint8_t c = serialBufferRX[serialTailRX[port]][port];
    if ((serialHeadRX[port] != serialTailRX[port])) return c; else return 0;
  }
#endif

uint8_t SerialAvailable(uint8_t port) {
  #if defined(PROMICRO)
    #if !defined(TEENSY20)
      if(port == 0) return USB_Available(USB_CDC_RX);
    #else
      if(port == 0) return T_USB_Available();
    #endif
  #endif
  return (serialHeadRX[port] - serialTailRX[port])%RX_BUFFER_SIZE;
}

void SerialWrite(uint8_t port,uint8_t c){
  #if !defined(PROMINI)
    CURRENTPORT=port;
  #endif
  serialize8(c);UartSendData();
}

#ifdef DEBUGMSG
void debugmsg_append_str(const char *str) {
  while(*str) {
    debug_buf[head_debug++] = *str++;
    if (head_debug == DEBUG_MSG_BUFFER_SIZE) {
      head_debug = 0;
    }
  }
}

static uint8_t debugmsg_available() {
  if (head_debug >= tail_debug) {
    return head_debug-tail_debug;
  } else {
    return head_debug + (DEBUG_MSG_BUFFER_SIZE-tail_debug);
  }
}

static void debugmsg_serialize(uint8_t l) {
  for (uint8_t i=0; i<l; i++) {
    if (head_debug != tail_debug) {
      serialize8(debug_buf[tail_debug++]);
      if (tail_debug == DEBUG_MSG_BUFFER_SIZE) {
        tail_debug = 0;
      }
    } else {
      serialize8('\0');
    }
  }
}
#else
void debugmsg_append_str(const char *str) {};
#endif

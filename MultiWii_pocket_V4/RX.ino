/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

#if defined(SPEKTRUM)
  #include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner. 
#endif

//RAW RC values will be store here
#if defined(SBUS)
  volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
#elif defined(SPEKTRUM) 
  volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
#endif

#if defined(SBUS) //Channel order for SBUS RX Configs
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
  static uint16_t sbusIndex=0;
#elif defined(SPEKTRUM)
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
#endif

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() 
{
  int chan;
  /******************    Configure each rc pin for PCINT    ***************************/
  // Init PPM SUM RX
  // Init Sektrum Satellite RX
  #if defined (SPEKTRUM)
    SerialOpen(SPEK_SERIAL_PORT,115200);
  #endif
  // Init SBUS RX
  #if defined(SBUS)
    SerialOpen(SBUS_SERIAL_PORT,100000);
  #endif
  for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
}

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#if defined(SBUS)

#define SBUS_SYNCBYTE 0x0F 
#define SBUS_ENDBYTE 0x00
#define SBUS_FRAME_SIZE 25

struct dat 
{
  unsigned int chan0  : 11;		
  unsigned int chan1  : 11;		
  unsigned int chan2  : 11;		
  unsigned int chan3  : 11;		
  unsigned int chan4  : 11;		
  unsigned int chan5  : 11;		
  unsigned int chan6  : 11;		
  unsigned int chan7  : 11;		
} __attribute__ ((__packed__));

typedef union 
{		
  uint8_t in[SBUS_FRAME_SIZE];		
  struct dat msg;		
} sbus_msg;

static sbus_msg sbus;
static bool sbusFrameComplete = false;

// Receive ISR callback
static void sbusDataReceive(uint8_t c)
{
    uint32_t sbusTime;
    static uint32_t sbusTimeLast;
    static uint8_t  sbusFramePosition;

    sbusTime = micros();
    if ((sbusTime - sbusTimeLast) > 4000) sbusFramePosition = 0;
    sbusTimeLast = sbusTime;

    if(sbusFramePosition==0 && c != SBUS_SYNCBYTE) return;
    sbusFrameComplete = false; // lazy main loop didnt fetch the stuff
    if (sbusFramePosition != 0) sbus.in[sbusFramePosition-1] = c;
    if (sbusFramePosition == SBUS_FRAME_SIZE - 1)
    {
    	if (sbus.in[sbusFramePosition-1] == SBUS_ENDBYTE) sbusFrameComplete = true;
        sbusFramePosition = 0;
    } else {
        sbusFramePosition++;
    }
}

bool  readSbus()
{
  if (sbusFrameComplete)
  {
    if (!((sbus.in[22] >> 3) & 0x0001)) // failsave flag
    {
	failsafeCnt = 0; // clear FailSafe counter
	rcValue[0]  = sbus.msg.chan0/2 + SBUS_OFFSET;
	rcValue[1]  = sbus.msg.chan1/2 + SBUS_OFFSET;
	rcValue[2]  = sbus.msg.chan2/2 + SBUS_OFFSET;
	rcValue[3]  = sbus.msg.chan3/2 + SBUS_OFFSET;
	rcValue[4]  = sbus.msg.chan4/2 + SBUS_OFFSET;
	rcValue[5]  = sbus.msg.chan5/2 + SBUS_OFFSET;
	rcValue[6]  = sbus.msg.chan6/2 + SBUS_OFFSET;
	rcValue[7]  = sbus.msg.chan7/2 + SBUS_OFFSET;
    }
    sbusFrameComplete = false;
    return true;
  }
  return false;
}
#endif


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
#if defined(SPEKTRUM)
static uint8_t spekFrame[SPEK_FRAME_SIZE];
static bool spekFrameComplete = false;

// Receive ISR callback
static void spektrumDataReceive(uint8_t c)
{
    uint32_t spekTime;
    static uint32_t spekTimeLast, spekTimeInterval;
    static uint8_t  spekFramePosition;

    spekTime = micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;
    if (spekTimeInterval > 5000)
        spekFramePosition = 0;
    spekFrame[spekFramePosition] = (uint8_t)c;
    if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
        spekFrameComplete = true;
        failsafeCnt = 0;   // clear FailSafe counter
    } else {
        spekFramePosition++;
    }
}

bool readSpektrum() 
{
    if (spekFrameComplete)
    {
      for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2)
      {
        uint8_t bh = spekFrame[b];
        uint8_t bl = spekFrame[b+1];
        uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
        if (spekChannel < RC_CHANS) 
          rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
      }
      spekFrameComplete = false;
      return true;
    }
    return false;
}
#endif


/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/

bool computeRC() 
{
  uint8_t chan;
  #if defined(SBUS)
    if (readSbus())
    {
      for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
      return true;
    }
  #elif defined(SPEKTRUM)
    if (readSpektrum())
    {
      for (chan = 0; chan < RC_CHANS; chan++) rcData[chan] = rcValue[rcChannel[chan]];
      return true;
    }
  #endif
  return false;
}






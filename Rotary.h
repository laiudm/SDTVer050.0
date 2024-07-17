// stubbed rotary encoder to allow injecting rotation events from the serial port

#define DIR_NONE 0x0
#define DIR_CW 0x10
#define DIR_CCW 0x20

class Rotary {
  public:
    unsigned char injectedEvent = DIR_NONE;
    
    Rotary(char _pin1, char _pin2) {}

    void begin(bool internalPullup=true, bool flipLogicForPulldown=false) {}

    unsigned char process() {
      unsigned char retValue = injectedEvent;
      injectedEvent = DIR_NONE;
      return retValue;
    }

    void injectEvent(unsigned char e) {
      injectedEvent = e;
    }
  
};

#ifndef LEDBLINK_H
#define LEDBLINK_H

class ledBlink {
  private:
  
  public:
    ledBlink() = default;
    void init();
    void ledOff();
    void ledOn();
    void ledToggle();
    void close();
  };
  
#endif  

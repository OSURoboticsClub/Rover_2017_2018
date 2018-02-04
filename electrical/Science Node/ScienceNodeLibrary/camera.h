//camera firmware can be found at chdk.wikia.com
#include <Arduino.h>

class camera{
  private:
    int pin;
    float zm;
    void kill();
  public:
    camera(int);
    void pulse();
    void shoot();
    void slowZoomIn();
    void slowZoomOut();
    void fullZoomIn();
    void fullZoomOut();
    void focus();
};
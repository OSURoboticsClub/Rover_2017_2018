#include <camera.h>

int cameraPin = 7;
camera science(cameraPin);
  
void setup() {
}

void loop() {
  science.fullZoomIn();
  delay (2000);
  science.fullZoomOut();
  delay(2000);
}

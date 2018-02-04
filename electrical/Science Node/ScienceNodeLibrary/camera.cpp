#include<camera.h>

camera::camera(int pinn){
  pinMode(pinn, OUTPUT);
  pin = pinn;  
}

void camera::pulse(){
  digitalWrite(pin,HIGH);
  delay(105);
  digitalWrite(pin,LOW);
  delay(55);  
}

void camera::kill(){
  digitalWrite(pin,LOW);
  delay(455);
}

void camera::shoot(){
  for(int i=0;i<3;i++)
    pulse();
  kill();  
}

void camera::slowZoomIn(){
  pulse();
  kill();  
}

void camera::slowZoomOut(){
  pulse();
  pulse();
  kill();  
}

void camera::fullZoomIn(){
  for(int i=0;i<4;i++)
    pulse();
  kill();  
}

void camera::fullZoomOut(){
  for(int i=0;i<5;i++)
    pulse();
  kill();  
}
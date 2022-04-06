#include "XYPlotter.hpp"

void XYPlotter::setXDirection(Direction direction){
  currentXDirection = direction;
  if(direction != XYPlotter::Direction::standStill){
    digitalWrite(xDirectionPin, direction);
  }
}
void XYPlotter::setYDirection(Direction direction){
  currentYDirection = direction;
  if(direction != XYPlotter::Direction::standStill){
    digitalWrite(yDirectionPin, direction);
  }
}
void XYPlotter::setXYDirection(Direction xDirection, Direction yDirection){
  setXDirection(xDirection);
  setYDirection(yDirection);
}

XYPlotter::XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin): //, uint8_t servoPin):
	xDirectionPin(xDirectionPin),
	xStepPin(xStepPin),
	yDirectionPin(yDirectionPin),
	yStepPin(yStepPin),
	servoPin(10)
{
	pinMode(xDirectionPin, OUTPUT);
	pinMode(xStepPin, OUTPUT);
	pinMode(yDirectionPin, OUTPUT);
  pinMode(yStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(xStepPin, LOW);
	digitalWrite(yStepPin, LOW);
	digitalWrite(enablePin, LOW);
}

void XYPlotter::step() {
  bool delay = false;
  if(currentXDirection != XYPlotter::Direction::standStill){
		digitalWrite(xStepPin, HIGH);
    delay = true;
    if(currentXDirection == XYPlotter::Direction::counterClockwise){
      currentPos[0]++;
    }
    else{
      currentPos[0]--;
    }
  }
  if(currentYDirection != XYPlotter::Direction::standStill){
    digitalWrite(yStepPin, HIGH);
    delay = true;
    if(currentYDirection == XYPlotter::Direction::counterClockwise){
      currentPos[1]++;
    }
    else{
      currentPos[1]--;
    }
  }
  if (delay){
	  delayMicroseconds(delayUs);
    digitalWrite(xStepPin, LOW);
    digitalWrite(yStepPin, LOW);
    delayMicroseconds(delayUs);
  }
}
struct Vergroting{
  unsigned int stappen;
  unsigned int verhoging;
};
Vergroting findStappen(double ratio){
  if(ratio == 0){
    return {0, 1};
  }
  if(ratio == 1){
    return {1, 1};
  }
  int verhoging=1;
  while(((1/ratio)*verhoging) != int((1/ratio)*verhoging)){
    verhoging++;
  }
  Vergroting returnValue = {(unsigned int)(1/ratio)*verhoging, (unsigned int)verhoging};
  return returnValue;
}

void XYPlotter::draw(int hoek, unsigned int aantalStappen){
  XYPlotter::Angle vorigeHoek = angles[(int)((hoek%360)/ANGLE_INTERVAL)];
  XYPlotter::Angle volgendeHoek = angles[((int)((hoek%360)/ANGLE_INTERVAL)+1)%8];
  double ratio = (double)(hoek%ANGLE_INTERVAL)/(ANGLE_INTERVAL);
  Vergroting returnValue = findStappen(ratio);
  long unsigned int stappen = returnValue.stappen;
  long unsigned int verhoging = returnValue.verhoging;
  Serial.print("Found ratio,    ");
  for(long unsigned int i = 0; i<aantalStappen; i++){
    setXYDirection(vorigeHoek.motorX, vorigeHoek.motorY);
      if((i*verhoging)%stappen==0){
//        Serial.print("in if");
        setXYDirection(volgendeHoek.motorX, volgendeHoek.motorY);

      }
      step();
  }
  Serial.print(", ");
  Serial.print("Ratio = ");
  Serial.print(ratio);
  Serial.print(", ");
  Serial.print("VorigeHoekD = ");
  Serial.print(vorigeHoek.hoek);
  Serial.print(", ");
  Serial.print("VorigeHoekX = ");
  Serial.print(vorigeHoek.motorX);
  Serial.print(", ");
  Serial.print("VorigeHoekY = ");
  Serial.print(vorigeHoek.motorY);
  Serial.print(", ");
  Serial.print("VolgendeHoekD = ");
  Serial.print(volgendeHoek.hoek);
  Serial.print(", ");
  Serial.print("VolgendeHoekX = ");
  Serial.print(volgendeHoek.motorX);
  Serial.print(", ");
  Serial.print("VolgendeHoekY = ");
  Serial.print(volgendeHoek.motorY);
  Serial.print(", ");
  Serial.print("Ratio = ");
  Serial.print(ratio);
  Serial.print(", ");
  Serial.print("Stappen = ");
  Serial.print(stappen);
  Serial.print(", ");
  Serial.print("Verhoging = ");
  Serial.print(verhoging);
  Serial.print(", ");
  Serial.print("Coordinate = ");
  Serial.print(currentPos[0]);
  Serial.print(", ");
  Serial.println(currentPos[1]);
}

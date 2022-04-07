String inString = "";
bool state = 0;

struct coor {
  float x;
  float y;
};

void draw(coor xy, bool pen) {
  Serial.println("je moeder op een driewieler ");
  Serial.println(xy.x);
  Serial.println(' ');
  Serial.println(xy.y);
  Serial.println("bool: ");
  Serial.println(pen);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {

    int g_code = Serial.parseInt();
    float x_coor = 0;
    float y_coor = 0;

    switch (g_code) {
      //G00 Z-axis up and move to location (x, y)
      case 0:
        x_coor = Serial.parseFloat();
        y_coor = Serial.parseFloat();
        draw({x_coor, y_coor}, false);
        break;

      //G01 Z-axis down and move to location (draw line) (x, y)
      case 1:
        x_coor = Serial.parseFloat();
        y_coor = Serial.parseFloat();
        draw({x_coor, y_coor}, true);
        break;

      //G02 Arc movement clockwise arc (x, y, i, j, e)
      case 2:
        Serial.println("case 2");
        break;

      //G03 Arc movement clockwise arc (x, y, i, j, e)
      case 3:
        Serial.println("case 3");
        break;
    }
  }
}

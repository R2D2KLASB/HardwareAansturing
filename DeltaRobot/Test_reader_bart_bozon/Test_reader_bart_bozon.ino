#include <Servo.h>
#include <math.h>
#include <stdio.h>

#define DEBUG


Servo servo1;
Servo servo2;
Servo servo3;

void xyz_naar_hoeken(double x, double y, double z);

void setup() {
  // put your setup code here, to run once:

  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  delay(200);
  servo1.write(150);
  servo2.write(150);
  servo3.write(150);
  Serial.begin(9600);

}


void loop() {
  // put your main code here, to run repeatedly:
  xyz_naar_hoeken(20, 0, -200);
  Serial.println("1");
  delay(2000);
  //
  xyz_naar_hoeken(0, 0, -200);
  Serial.println("2");
  delay(2000);
  xyz_naar_hoeken(60, 0, -190);
  Serial.println("3");
  delay(2000);

}


void xyz_naar_hoeken(double x, double y, double z) {
  const double pi = 3.141592653589793;// PI
  const double rad = pi / 180;
  const double rad60 =  rad * 60 ;

  double L_arm_1 = 50.0;
  double L_arm_2 = 218.0;

  //  double a1 = -10;

  double L_top = 35.0;
  double L_base = 105.0;

  double afstand = 0.0;

  double P_elleboog[3][3] = {0, 0, 0};

  double servo_hoeken[3] = { -10 * rad, -10 * rad, -10 * rad};

  double P_doel[3] = {x, y, z};
  double P_pols_1[3] = {x + L_top, y, z};
  double P_pols_2[3] = {x - cos(rad60) * L_top, y + sin(rad60) * L_top, z};
  double P_pols_3[3] = {x - cos(rad60) * L_top, y - sin(rad60) * L_top, z};
  double P_pols[3][3] = {
    {x + L_top, y, z},
    {x - cos(rad60) * L_top, y + sin(rad60) * L_top, z},
    {x - cos(rad60) * L_top, y - sin(rad60) * L_top, z}
  };

  double P_servo_1[3] = {0 + L_base, 0, 0};
  double P_servo_2[3] = {0 - cos(rad60) * L_base, 0 + sin(rad60) * L_base, 0};
  double P_servo_3[3] = {0 - cos(rad60) * L_base, 0 - sin(rad60) * L_base, 0};

#ifdef DEBUG
  Serial.println(P_pols_1[0]);
  Serial.println(P_pols_1[1]);
  Serial.println(P_pols_1[2] );
  Serial.println();
  Serial.println(P_pols_2[0]);
  Serial.println(P_pols_2[1]);
  Serial.println(P_pols_2[2] );
  Serial.println();
  Serial.println(P_pols_3[0]);
  Serial.println(P_pols_3[1]);
  Serial.println(P_pols_3[2]);
#endif
  for (int i = 2 ; i >= 0 ; i--) {
    afstand = 0.0;
    while ((L_arm_2 - 10 > afstand) || (afstand > L_arm_2 + 10)) {
      switch (i) {
        case 0:
          P_elleboog[i][0] = P_servo_1[0] + L_arm_1 * cos(servo_hoeken[i]);
          P_elleboog[i][1] = P_servo_1[1];
          P_elleboog[i][2] = P_servo_1[2] + L_arm_1 * sin(servo_hoeken[i]);
          break;

        case 1:
          P_elleboog[i][0] = P_servo_2[0] - cos(rad60) * L_arm_1 * cos(servo_hoeken[i]);
          P_elleboog[i][1] = P_servo_2[1] + sin(rad60) * L_arm_1 * cos(servo_hoeken[i]);
          P_elleboog[i][2] = P_servo_2[2] + L_arm_1 * sin(servo_hoeken[i]);
          break;

        case 2:
          P_elleboog[i][0] = P_servo_3[0] - cos(rad60) * L_arm_1 * cos(servo_hoeken[i]);
          P_elleboog[i][1] = P_servo_3[1] - sin(rad60) * L_arm_1 * cos(servo_hoeken[i]);
          P_elleboog[i][2] = P_servo_3[2] + L_arm_1 * sin(servo_hoeken[i]);
          break;
      }

      afstand = sqrt(pow((float)(P_elleboog[i][0] - P_pols[i][0]), 2) + pow((float)(P_elleboog[i][1] - P_pols[i][1]), 2) + pow((float)(P_elleboog[i][2] - P_pols[i][2]), 2));
      //      afstand = sqrt(pow((float)(P_elleboog[i][0] - P_doel[0]), 2) + pow((float)(P_elleboog[i][1] - P_doel[1]), 2) + pow((float)(P_elleboog[i][2] - P_doel[2]), 2));

      servo_hoeken[i] += rad;


#ifdef DEBUG
      Serial.print("afstand ");
      Serial.print(afstand);
      Serial.print("    Servo Hoek: ");
      Serial.print(servo_hoeken[i] * 180 / pi);
      Serial.print("    Nummer: ");
      Serial.println(i);
      //      Serial.print("    a1: ");
      //      Serial.println(a1);
#endif

    }
  }
  servo1.write(servo_hoeken[0] * 180 / pi );
  servo2.write(servo_hoeken[1] * 180 / pi);
  servo3.write(servo_hoeken[2] * 180 / pi);

  Serial.println(servo_hoeken[0] * 180 / pi);
  Serial.println(servo_hoeken[1] * 180 / pi);
  Serial.println(servo_hoeken[3] * 180 / pi);

  Serial.println();

}

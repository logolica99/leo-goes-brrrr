#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

#include <SoftwareSerial.h>

SoftwareSerial BT(10, 11);

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 0 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 535 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

float c[4], d[4], x[4], z[4], A[4], B[4], C[4], D[4], thetaX[4], phi[4], y[4], n, ellipse_a = 4, ellipse_b = 2; // s=radius, A,B,C triangle angles, a,b,c triangle sides, z is height from base
int servoposition[16] = {
  200,
  200,
  70,
  0,
  110,
  90,
  70,
  0,
  100,
  140,
  140,
  0,
  100,
  200,
  120,
  0
};
int a = 7.5, b = 13, i;



int IK(){
  
  
  for(int n=0;n<4;n++)
  {
    thetaX[n]=atan(x[n]/c[n]);
    phi[n]=atan(y[n]/c[n]);
    d[n]=sqrt(pow(x[n],2)+pow(y[n],2)+pow((c[n]-z[n]),2));
  }
  
  for(int n=0;n<4;n++)
  {
  A[n]=acos((pow(b,2)+pow(d[n],2)-pow(a,2))/(2*b*d[n]));
  B[n]=acos((pow(a,2)+pow(d[n],2)-pow(b,2))/(2*a*d[n]));
  C[n]=acos((pow(b,2)+pow(a,2)-pow(d[n],2))/(2*b*a));
  
  
  B[n]=-thetaX[n]+B[n]; //Komabo B and add korbo theta
  
  }


   pwm.setPWM(12,0,angletopulse(servoposition[12]+(phi[3]*180/3.1416)));
    pwm.setPWM(13,0,angletopulse(servoposition[13]-(B[3]*180/3.1416)));
    pwm.setPWM(14,0,angletopulse(servoposition[14]-180+(C[3]*180/3.1416)));

    pwm.setPWM(4,0,angletopulse(servoposition[4]+(phi[1]*180/3.1416)));
    pwm.setPWM(5,0,angletopulse(servoposition[5]+(B[1]*180/3.1416)));
    pwm.setPWM(6,0,angletopulse(servoposition[6]+180-(C[1]*180/3.1416)));


    pwm.setPWM(0,0,angletopulse(servoposition[0]-(phi[0]*180/3.1416)));
    pwm.setPWM(1,0,angletopulse(servoposition[1]-(B[0]*180/3.1416)));
    pwm.setPWM(2,0,angletopulse(servoposition[2]-180+(C[0]*180/3.1416)));

   
    pwm.setPWM(8,0,angletopulse(servoposition[8]-(phi[2]*180/3.1416)));
    pwm.setPWM(9,0,angletopulse(servoposition[9]+(B[2]*180/3.1416)));
    pwm.setPWM(10,0,angletopulse(servoposition[10]+180-(C[2]*180/3.1416)));
  
}

int angletopulse(int ang) {
  int pulse = map(ang, 0, 270, SERVOMIN, SERVOMAX);
  return pulse;
}

int standup() {

 pwm.setPWM(0, 0, angletopulse(200));//  increasing goes inside
  pwm.setPWM(1, 0, angletopulse(200));//  increasing goes backward
  pwm.setPWM(2, 0, angletopulse(70));//  increasing goes forward

  pwm.setPWM(4, 0, angletopulse(110));//increasing goes outside
  pwm.setPWM(5, 0, angletopulse(90));//  increasing goes forward
  pwm.setPWM(6, 0, angletopulse(90));//  increasing goes backward

  pwm.setPWM(8, 0, angletopulse(100));//increasing goes inside
  pwm.setPWM(9, 0, angletopulse(140));//  increasing goes forward
  pwm.setPWM(10, 0, angletopulse(140));//  increasing goes backward

  pwm.setPWM(12, 0, angletopulse(100)); //increasing goes outside
  pwm.setPWM(13, 0, angletopulse(200)); //increasing goes backward
  pwm.setPWM(14, 0, angletopulse(120));//  increasing goes forward

  // 
  //

  delay(1000);

}


int sit(){


  pwm.setPWM(0, 0, angletopulse(200));//  increasing goes inside
  pwm.setPWM(1, 0, angletopulse(260));//  increasing goes backward
  pwm.setPWM(2, 0, angletopulse(130));//  increasing goes forward

  pwm.setPWM(4, 0, angletopulse(110));//increasing goes outside
  pwm.setPWM(5, 0, angletopulse(45));//  increasing goes forward
  pwm.setPWM(6, 0, angletopulse(45));//  increasing goes backward

  pwm.setPWM(8, 0, angletopulse(100));//increasing goes inside
  pwm.setPWM(9, 0, angletopulse(90));//  increasing goes forward
  pwm.setPWM(10, 0, angletopulse(90));//  increasing goes backward

  pwm.setPWM(12, 0, angletopulse(100)); //increasing goes outside
  pwm.setPWM(13, 0, angletopulse(260)); //increasing goes backward
  pwm.setPWM(14, 0, angletopulse(180));//  increasing goes forward

  // 
  //

  delay(1000);

}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  BT.begin(9600);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  c[0] = c[1] = c[2] = c[3] = 14;
  x[0] = x[1] = x[2] = x[3] = 0;
  z[0] = z[1] = z[2] = z[3] = 0;
  y[0] = y[1] = y[2] = y[3] = 0;
  Serial.println("read for command");

  delay(10);

  standup();

}

char command;

void loop() {
  if (BT.available()) {
    command = (BT.read());

    if(command=='0'){ // to standup
      standup();
    }

    if (command == '1') { // to test
      pwm.setPWM(0, 0, angletopulse(180));
      pwm.setPWM(1, 0, angletopulse(200));
      pwm.setPWM(2, 0, angletopulse(70));

      pwm.setPWM(4, 0, angletopulse(110));
      pwm.setPWM(5, 0, angletopulse(90));
      pwm.setPWM(6, 0, angletopulse(70));

      pwm.setPWM(8, 0, angletopulse(100));
      pwm.setPWM(9, 0, angletopulse(140));
      pwm.setPWM(10, 0, angletopulse(140));

      pwm.setPWM(12, 0, angletopulse(100));
      pwm.setPWM(13, 0, angletopulse(200));
      pwm.setPWM(14, 0, angletopulse(120));
    }

    if (command == 'm') { //walk  forward

      for (x[0] = -ellipse_a, x[1] = -ellipse_a, x[2] = ellipse_a, x[3] = ellipse_a; x[0] <= ellipse_a, x[1] <= ellipse_a, x[2] >= -ellipse_a, x[3] >= -ellipse_a; x[0] += 0.5, x[1] += 0.5, x[2] -= 0.5, x[3] -= 0.5) {
        z[0] = ellipse_b * sqrt(1 - (pow(x[0], 2) / pow(ellipse_a, 2)));
        z[1] = ellipse_b * sqrt(1 - (pow(x[1], 2) / pow(ellipse_a, 2)));
        IK();
        delay(5);

      }

      delay(200);

      for (x[0] = ellipse_a, x[1] = ellipse_a, x[2] = -ellipse_a, x[3] = -ellipse_a; x[0] >= -ellipse_a, x[1] >= -ellipse_a, x[2] <= ellipse_a, x[3] <= ellipse_a; x[0] -= 0.5, x[1] -= 0.5, x[2] += 0.5, x[3] += 0.5) {
        z[2] = ellipse_b * sqrt(1 - (pow(x[2], 2) / pow(ellipse_a, 2)));
        z[3] = ellipse_b * sqrt(1 - (pow(x[3], 2) / pow(ellipse_a, 2)));
        IK();

        delay(5);

      }
      delay(200);

    }

    if(command = 's'){
      sit();
    }

  }else{
    standup();
  }

}
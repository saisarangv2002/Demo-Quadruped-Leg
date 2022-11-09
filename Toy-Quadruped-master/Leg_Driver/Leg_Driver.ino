#include<Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define pi 3.141592653589793238462643383279

#define servoFreq 50
#define minPulseWidth 102
#define maxPulseWidthMG996 450

#define thighServoHome 45
#define crankServoHome 90

#define T 90
#define O 65
#define C 40
#define L 120
#define R 40
#define CU 63.654148805222384

/* PCA9685 Servo Driver Object*/
Adafruit_PWMServoDriver driver;

/*-------------------THIGH OBJECT ----------------------*/

class Thigh{
  private:
  int thigh_motor;

  public:
  //float thigh_curr;

  /*Control joint variable alpha 
    Enter angle in degrees
  */
  void drive(float angle){
    int p = ((90 - angle)/270)*(maxPulseWidthMG996 - minPulseWidth) + minPulseWidth;
    //thigh_curr = angle;
    driver.setPWM(thigh_motor, 0, p);
  }

  /* Set the motor channel associated with thigh*/ 
  void setMotor(int m){
    thigh_motor = m;
  }

  /*Initialise thigh with the motor number that controls it
    Initialised by 0, but can be changed by setMotor
  */
  Thigh(){
    //thigh_home = pi/4;
    //thigh_curr = thigh_home;

    thigh_motor = 0;
  }
};

/*------------------- CRANK OBJECT -----------------*/
class Crank{
  private:
  int crank_motor; 

  public:
  //float crank_curr;

  /*Control joint variable beta 
    Enter angle in degrees
  */
  void drive(float angle){
    int p = ((-angle + crankServoHome)/270)*(maxPulseWidthMG996 - minPulseWidth) + minPulseWidth;
    //crank_curr = angle;
    driver.setPWM(crank_motor, 0, p);
  }

  /*Set the motor channel associated with crank*/
  void setMotor(int m){
    crank_motor = m;
  }

  /*Initialise crank with the motor number that controls it
    Initialised by 0, but can be changed later using setMotor
  */
  Crank(){
    //crank_home = pi/2;
    //crank_curr = crank_home;
    crank_motor = 0;
  }
};


/*-------------------------- MECHANISM CLASS --------------------------*/
class Mechanism{

  public:

  /*Drivable Links of the mechanism*/
  Thigh thigh;
  Crank crank;

  /*Variables that describe the state of the leg*/
  float theta;
  float phi;
  float alpha;
  float beta;
  float delta;
  float gamma;
  float x;
  
  float f;
  float t;

  /*Constraints on beta, depends on alpha*/
  float betaLOW;
  float betaHIGH;
  float betaHOME;

  /*Constraints on alpha*/
  float alphaLOW;
  float alphaHIGH;
  float alphaHOME;

  /*Toe coordinates*/
  float toe_x;
  float toe_y;

  /*Calculate state variables*/
  void updateState(){

    f = sqrt(O*O + T*T - 2*O*T*cos(alpha));

    theta = asin(T*sin(alpha)/f);
    delta = asin(O*sin(alpha)/f);

    t = sqrt(f*f + C*C - 2*f*C*cos(pi - theta - beta));

    x = acos((R*R + CU*CU - 2*t*t)/(2*R*CU));
    phi = asin(CU*sin(x)/t) + asin(C*sin(pi - theta - beta)/t);

    gamma = 2*pi - delta - phi - 1.9198;

    toe_x = T*cos(alpha) - L*cos(gamma - alpha);
    toe_y = T*sin(alpha) + L*sin(gamma - alpha);
  }

  /*Calculate limits on crank*/
  void findCrankLims(){
    f = sqrt(O*O + T*T - 2*O*T*cos(alpha));
    theta = asin(T*sin(alpha)/f);

    betaHIGH = pi - theta - acos(((C+CU)*(C+CU) + f*f -R*R)/(2*(C+CU)*f));
    betaLOW = pi - theta - acos((C*C + f*f -(R+CU)*(R+CU))/(2*C*f));

    //betaHIGH = betaHIGH - ((float)45/180)*pi;
    
  }

  /*Attain the state (alpha, beta)*/
  void attainState(float a, float b){

    thigh.drive(a*180/pi);
    crank.drive(b*180/pi);

    alpha = a;
    beta = b;

    updateState();
  }

  /*Initialise mechanism with thigh and crank motor channels*/
  void init(int m1, int m2){
    thigh.setMotor(m1);
    crank.setMotor(m2);

    alphaHOME = pi/4;
    betaHOME = 0;
    
    alpha = alphaHOME;
    beta = betaHOME;

    alphaHIGH = pi/2;
    alphaLOW = pi/4;

    findCrankLims();

    attainState(alpha, beta);

    delay(2000);
  }

  /*Go along the outline of the workspace*/
  void locus(int delay_t){
    
    attainState(alpha, betaLOW + 0.08);
    
    for(float a=alphaHOME; alpha<=alphaHIGH; a += 0.02){
      alpha = a;
      findCrankLims();
      attainState(alpha, betaLOW + 0.1);

      delay(delay_t);
    }

    for(float b=betaLOW+0.1; b <= betaHIGH; b += 0.02){
      attainState(alpha, b);

      delay(delay_t);
    }

    for(float a=alphaHIGH; alpha>=alphaHOME+0.1; a -= 0.02){
      alpha = a;
      findCrankLims();

      attainState(a, betaHIGH - 0.1);

      delay(delay_t);
    }

    for(float b=betaHIGH-0.1; b >= betaLOW+0.05; b -= 0.02){
      attainState(alpha, b);

      delay(delay_t);
    }

  }

  /*Perform walk motion*/
  void walkMotion(int delay_t){

    attainState(alpha, betaLOW + 0.08);
    
    for(float a=alphaHOME; alpha<=((float)75*pi/180); a += 0.02){
      alpha = a;
      findCrankLims();
      attainState(alpha, betaLOW + 0.1);

      delay(delay_t);
    }

    for(float b=betaLOW+0.1; b <= betaHIGH - ((float)40/180)*pi; b += 0.02){
      attainState(alpha, b);

      delay(delay_t);
    }

    for(float a=((float)75*pi/180); alpha>=alphaHOME+0.1; a -= 0.02){
      alpha = a;
      findCrankLims();

      attainState(a, betaHIGH - ((float)40/180)*pi);

      delay(delay_t);
    }

    for(float b = betaHIGH - ((float)40/180)*pi; b >= betaLOW+0.05; b -= 0.02){
      attainState(alpha, b);

      delay(delay_t);
    }

  }


  /*Go to home position*/
  void goHome(){
    attainState(alphaHOME, betaHOME);
  }
  
};

/*Create mechanism object*/
Mechanism m;

void setup() {

  //Serial.begin(9600);

  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(servoFreq);

  /*Inialise mechanism*/
  m.init(9, 12);

  /*Go home and wait for 1 second*/
  m.goHome();

  delay(1000);
}

void loop() {

  /*Trace locus*/
  //m.walkMotion(5);
}

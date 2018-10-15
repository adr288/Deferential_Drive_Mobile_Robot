
class Ultrasonic
{
  public:
    Ultrasonic(int pin);
    void DistanceMeasure(void);
    float microsecondsToMillimeters(void);
    float microsecondsToInches(void);
  private:
    int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
    float durationOnePin;// the Pulse time received;
};

//Encoders

# define encoderRight 2
# define encoderLeft 3


//Ultrasonic Sersor Pins:


//Front
#define trigPin1 6
#define echoPin1 7

//Right
#define trigPin2 4
#define echoPin2 5

//Left
#define trigPin3 9
#define echoPin3 8


//Right End
Ultrasonic ultrasonicRightEnd(11);

//LeftEnd
Ultrasonic ultrasonicLeftEnd(10);




#include <Servo.h>
Servo servoLeft;          // Define left servo
Servo servoRight;         // Define right servo

//Encoder Variabls
volatile int countR = 0;
volatile int countL = 0;




//Constans
// L is distance of an arbitrary point wrt wheels' axis mid point
// l is the distance between two wheels
// R is the Radious of wheels

const float pi = 3.1416, R = 33.25, l = 96, L = 120,  phiInt = 1.5708 , Vmax = 23; // phiInt is the phi initial


//Goal's Position
int xGoal = 0 , yGoal = 3000;



//Global Variables initialization and declrations

double dL, dR, dC,  u1, u2 ;
double  eX, eY, e, xC = 0, yC = 0, eC, eXD = 0, eYD = 0, k1, k2, kI1, kI2, kD1, kD2, fW = 800000;
double phiDesier, eW = 0, eWOld = 0, eWC = 0, eWD = 0;
double eXOld = xGoal, eYOld = yGoal - L, goalAngle, obsAngle;
double v = 0, w = 0, wR, wL, sigR, sigL, x = L * cos(phiInt), y = L * sin(phiInt), phi = phiInt;


// Time
int t = 0; // t is the conter which counts every 10ms

//Distance Sensor Variables
double duration, rightDistance, leftDistance,  frontDistance, rightEndDistance, leftEndDistance;
double xObs, yObs, rD, lD, fD, rED, lED, d = 450;



void setup()
{
  Serial.begin (9600);


  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);


  servoRight.attach(12);  // Set left servo to digital pin 12
  servoLeft.attach(13);  // Set left servo to digital pin 13

  // Right encoder
  attachInterrupt(digitalPinToInterrupt(encoderRight), doEncoderRight, RISING);
  pinMode(encoderRight, INPUT);
  digitalWrite(encoderRight, HIGH);  // turn on pullup resistor

  // Left encoder
  attachInterrupt(digitalPinToInterrupt(encoderLeft), doEncoderLeft, RISING);
  pinMode(encoderRight, INPUT);
  digitalWrite(encoderLeft, HIGH);




  servoLeft.write(1500);
  servoRight.write(1500);
  countR = 0;
  countL = 0;
  Serial.print("Start");
  delay(5000);
}



void loop()
{

  //Robot's Position
  phi = phiInt + (dR - dL) / l;
  y =   dC * sin(phi) + L * sin(phi);
  x =   dC * cos(phi) + L * cos(phi);

  // Check distances
  frontDistance = distance(trigPin1, echoPin1);
  rightDistance = distance(trigPin2, echoPin2);
  leftDistance = distance(trigPin3, echoPin3);

  ultrasonicRightEnd.DistanceMeasure();// get the current signal time;
  rightEndDistance = ultrasonicRightEnd.microsecondsToMillimeters();//convert the time to centimeters

  ultrasonicLeftEnd.DistanceMeasure();// get the current signal time;
  leftEndDistance = ultrasonicLeftEnd.microsecondsToMillimeters();//convert the time to centimeters




  if (rightDistance < d || leftDistance < d || frontDistance < d || rightEndDistance < d || leftEndDistance < d )
  {

    do
    {
      declareDistances();

      goalAngle = getGoalAngle();
      obsAngle = getObsticleAngle();


      if ((obsAngle - goalAngle) >= 0)
      {
        fwCC();
        applyToSystem();
        Serial.print("u1 = ");
        Serial.print(u1);
        Serial.print(" - u2 = ");
        Serial.print(u2);
        Serial.print(" - Error = ");
        Serial.print(e);


        Serial.println(" Follow Wall CC");
        delay(10);

      }

      else if ((obsAngle - goalAngle) < 0)
      {
        fwC();
        applyToSystem();
        Serial.print("u1 = ");
        Serial.print(u1);
        Serial.print(" - u2 = ");
        Serial.print(u2);
        Serial.print(" - Error = ");
        Serial.print(e);

        Serial.println(" Follow Wall C");
        delay(10);

      }

      frontDistance = distance(trigPin1, echoPin1);
      rightDistance = distance(trigPin2, echoPin2);
      leftDistance = distance(trigPin3, echoPin3);

      ultrasonicRightEnd.DistanceMeasure();// get the current signal time;
      rightEndDistance = ultrasonicRightEnd.microsecondsToMillimeters();//convert the time to centimeters

      ultrasonicLeftEnd.DistanceMeasure();// get the current signal time;
      leftEndDistance = ultrasonicLeftEnd.microsecondsToMillimeters();//convert the time to centimeters


      if (rightDistance < 50 || leftDistance < 50 || frontDistance < 50 || rightEndDistance < 50 || leftEndDistance < 50)
      {
       
       do {
          avoidObsticle();
          applyToSystem();
          Serial.print("u1 = ");
          Serial.print(u1);
          Serial.print(" - u2 = ");
          Serial.print(u2);
          Serial.println(" Avoid Obssticle");
          /* Serial.print(leftDistance);
            Serial.print(" - ");
            Serial.print(frontDistance);
            Serial.print(" - ");
            Serial.print(rightDistance);
            Serial.print(" - ");
            Serial.print(rightEndDistance);
            Serial.print(" - ");
            Serial.print(leftEndDistance);

          */
          frontDistance = distance(trigPin1, echoPin1);
          rightDistance = distance(trigPin2, echoPin2);
          leftDistance = distance(trigPin3, echoPin3);

          ultrasonicRightEnd.DistanceMeasure();// get the current signal time;
          rightEndDistance = ultrasonicRightEnd.microsecondsToMillimeters();//convert the time to centimeters

          ultrasonicLeftEnd.DistanceMeasure();// get the current signal time;
          leftEndDistance = ultrasonicLeftEnd.microsecondsToMillimeters();//convert the time to centimeters
          delay(10);

        } while (rightDistance < (50 + 10) || leftDistance < (50 + 10) || frontDistance < (50 + 10) || rightEndDistance < (50 + 10) || leftEndDistance < (50 + 10) );

      }

    } while (rightDistance < (d + 50) || leftDistance < (d + 50) || frontDistance < (d + 50) || rightEndDistance < (d + 50) || leftEndDistance < (d + 50));
  }


  else
  {


    goToGoal();
    applyToSystem();
    Serial.print("X = ");
    Serial.print(x);
    Serial.print(" - Y = ");
    Serial.print(y);
    Serial.println(" - Go To Goal");

    /* Serial.print(leftDistance);
      Serial.print(" - ");
      Serial.print(frontDistance);
      Serial.print(" - ");
      Serial.print(rightDistance);
      Serial.print(" - ");
      Serial.print(rightEndDistance);
      Serial.print(" - ");
      Serial.print(leftEndDistance);


    */
    // Check new distances

    frontDistance = distance(trigPin1, echoPin1);
    rightDistance = distance(trigPin2, echoPin2);
    leftDistance = distance(trigPin3, echoPin3);

    ultrasonicRightEnd.DistanceMeasure();// get the current signal time;
    rightEndDistance = ultrasonicRightEnd.microsecondsToMillimeters();//convert the time to centimeters

    ultrasonicLeftEnd.DistanceMeasure();// get the current signal time;
    leftEndDistance = ultrasonicLeftEnd.microsecondsToMillimeters();//convert the time to centimeters

    delay(10);
  }



  //Stop Condition
  if ((abs(y - (yGoal)) < 50 && abs(x - (xGoal)) < 50))
  {
    while ((abs(y - (yGoal)) < 50 && abs(x - (xGoal)) < 50))
    {
      servoLeft.write(1500);
      servoRight.write(1500);
      Serial.println("Stop");

    }
  }


  // Check new distances

  frontDistance = distance(trigPin1, echoPin1);
  rightDistance = distance(trigPin2, echoPin2);
  leftDistance = distance(trigPin3, echoPin3);

  ultrasonicRightEnd.DistanceMeasure();// get the current signal time;
  rightEndDistance = ultrasonicRightEnd.microsecondsToMillimeters();//convert the time to centimeters

  ultrasonicLeftEnd.DistanceMeasure();// get the current signal time;
  leftEndDistance = ultrasonicLeftEnd.microsecondsToMillimeters();//convert the time to centimeters

}




// Go to Goal Funtion
void goToGoal()
{
  eX = xGoal - x;
  eY = yGoal - y;


  e = sqrt(sq(eX) + sq(eY));

  k1 = Vmax * (1 - (exp(-20 * sq(e)))) / e;
  k2 = Vmax * (1 - (exp(-20 * sq(e)))) / e;

  //Robot Control Commands
  u1 = k1 * (xGoal - x); //   (xGoal - x) is xError (dp/dt=kp)
  u2 = k2 * (yGoal - y); // (yGoal-y) is yError
}


//Object Avoidance Function
void avoidObsticle()
{
  //Object position Position
  xObs = (rD - lD) * 0.707 + rED - lED;
  yObs = fD + ( rD + lD) * 0.707 ;


  // Distance from Goal
  e = sqrt(sq(xObs) + sq(yObs));

  //Control Gains
  k1 = 400000 / (e * (sq(e) + 0.001));
  k2 = 400000 / (e * (sq(e) + 0.001));

  u1 = -k1 * (xObs);
  u2 = -k2 * (yObs);
}

void fwC()
{
  //Object's Position
  xObs = (rD - lD) * 0.707 + rED - lED;
  yObs = fD + ( rD + lD) * 0.707 ;


  // Distance from Goal
  e = sqrt(sq(xObs) + sq(yObs));

  //Control Gains
  k1 = fW / (e * (sq(e) + 0.001));
  k2 = fW / (e * (sq(e) + 0.001));

  u1 = -k1 * (xObs); // x is positive for clockwise
  u2 = k2 * (yObs);
}


void fwCC()
{
  //Object position Position
  xObs = (rD - lD) * 0.707 + rED - lED ;
  yObs = fD + ( rD + lD) * 0.707 ;



  // Distance from Goal
  e = sqrt(sq(xObs) + sq(yObs));

  //Control Gains
  k1 = fW / (e * (sq(e) + 0.001));
  k2 = fW / (e * (sq(e) + 0.001));

  u1 = -k1 * (xObs);
  u2 =  k2 * (yObs); //y is positive for counter clockwise


}


void applyToSystem()
{

  v = u1 * cos(-phi) - u2 * sin(-phi);
  w = (1 / L) * (u1 * sin(-phi) + u2 * cos(-phi));




  wR = (2 * v + l * w) / (2 * R);
  wL = (2 * v - l * w) / (2 * R);


  sigR = 1500 + 130.43 * wR; //  Based on 0.69 rev/sec as max on right wheel
  sigL = 1500 + 275.36 * wL; //  Based on 0.69 rev/sec as max on left wheel

  servoLeft.write(sigL);
  servoRight.write(3000 - sigR);

}




void applyToSystem2()
{
  phiDesier = atan2(u2, u1);

  eWOld = eW;

  eW = phiDesier - phi;
  eW = atan2(sin(eW), cos(eW));
  eWC = eWC + eW;
  eWD = eW - eWOld;


  v = sqrt((u1) + sq(u2));
  w = 0.26 * eW + 0.03 * eWC + 0.1 * eWD;



  //left and right wheel velocities
  wR = (2 * v + l * w) / (2 * R);
  wL = (2 * v - l * w) / (2 * R);

  sigR = 1500 + 130.43 * wR; //  Based on 0.69 rev/sec as max on right wheel
  sigL = 1500 + 275.36 * wL; //  Based on 0.69 rev/sec as max on left wheel

  servoLeft.write(sigL);
  servoRight.write(3000 - sigR);
}




double getGoalAngle()
{

  return atan((yGoal - y) / (xGoal - x));
}

double getObsticleAngle()
{

  return atan2((fD + (rD + lD) * 0.707) , ((rD - lD) * 0.707) + rED - lED);
}


//Encoder Functions

void doEncoderRight()
{
  if (sigR < 1500)
  {
    countR--;
  }
  else if (sigR > 1500)
  {
    countR++;
  }

  // Wheel Distance Travled
  dR = 2 * pi * R * countR / 8; //Right
  dC = (dL + dR) / 2;
}


void doEncoderLeft()
{
  if (sigL > 1500)
  {
    countL ++;
  }
  else if (sigL < 1500)
  {
    countL--;
  }

  // Wheel Distance Travled
  dL = 2 * pi * R * countL / 8;  //Left
  dC = (dL + dR) / 2;
}

void declareDistances()
{
  if (rightDistance < (d+52) )
  {
    rD = rightDistance;
  }

  else
  {
    rD = 0;
  }


  if (leftDistance < (d+52) )
  {
    lD = leftDistance;
  }
  else
  {
    lD = 0;
  }

  if (frontDistance < (d+52))
  {
    fD = frontDistance;
  }
  else
  {
    fD = 0;
  }

  if (rightEndDistance < (d+52) )
  {
    rED = rightEndDistance;
  }

  else
  {
    rED = 0;
  }

  if (leftEndDistance < (d+52) )
  {
    lED = leftEndDistance;
  }

  else
  {
    lED = 0;
  }
}


float distance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return duration * 34 / 200;
}






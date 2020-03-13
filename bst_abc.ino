#include <PinChangeInt.h>
#include <TimerOne.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;


#define INA1 6
#define INA2 5
#define PWMA 4

#define INB1 7
#define INB2 8
#define PWMB 9

#define PinA_left   11  //Interrupt 0
#define PinA_right  13 //Interrupt 1


int time;
byte inByte;
int num;
double Setpoint;
double Setpoints, Outputs = 0;




//********************angle data*********************//
float Q;
float Angle_ax;
float Angle_ay;

//********************angle data*********************//

//***************Kalman_Filter*********************//
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5;
float dt = timeChange * 0.001;
//***************Kalman_Filter*********************//

//*********************************************
//******************** speed count ************
//*********************************************

volatile long count_right = 0;
volatile long count_left = 0;
int speedcc = 0;

//////////////////////Pulse calculation/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
/////////////////////Pulse calculation////////////////////////////

//////////////Steering and rotation parameters///////////////////////////////
int turncount = 0;
float turnoutput = 0;
//////////////Steering and rotation parameters///////////////////////////////

//////////////Wifi control volume///////////////////
#define run_car     '1'
#define back_car    '2'
#define left_car    '3'
#define right_car   '4'
#define stop_car    '0'
/*Yhe car status enumeration*/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
}
enCarState;
int incomingByte = 0;
String inputString = "";         //It used to store received content
boolean newLineReceived = false;
boolean startBit  = false;
int g_carstate = enSTOP; //  1run 2back 3left 4right 0stop
String returntemp = ""; //It used to store return value
boolean g_autoup = false;
int g_uptimes = 5000;

int front = 0;
int back = 0;
int turnl = 0;
int turnr = 0;
int spinl = 0;
int spinr = 0;
int bluetoothvalue;
//////////////bluetoothvalue///////////////////


int speedka, speedki;
#define tombol1 A9
#define tombol2 A8
#define tombol3 A11
#define led1 A10
#define led2 A12
#define led3 A14
#define tombolka 12
#define tombolki 14

//////////////////Ultrasonic velocity//////////////////


//////////////////////Pulse calculation///////////////////////
void countpluse()
{

  lz = count_left;
  rz = count_right;

  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }


  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  //When the interrupt is entered every 5ms, the pulse number is superimposed.
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam = balancecar.pulseright + balancecar.pulseleft;
}
////////////////////Pulse calculation///////////////////////



//////////////////////////////////////

//////////////////////////////////////

//////////////////////////////////////////////////////////
//////////////////Interrupt timing 5ms////////////////////
/////////////////////////////////////////////////////////
void timerISR()
{
  sei();

  countpluse();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                                           //IIC obtained MPU6050 six-axis data: ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);  //Getting Angle  and kaman filtering
  angleout();

  speedcc++;
  if (speedcc >= 8)                                //40ms access speed loop control
  {
    Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4)                                //40ms access the rotation control
  {
    turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);      //Rotator function
    turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs, turnoutput * 0.7, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, INA1, INA2, INB1, INB2, PWMA, PWMB);

}
//////////////////////////////////////////////////////////
//////////////////Interrupt timing 5ms///////////////////
/////////////////////////////////////////////////////////

// ===  initial setting    ===
void setup()
{
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);


  digitalWrite(INA1, 0);
  digitalWrite(INA2, 1);
  digitalWrite(INB1, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);
  pinMode(PinA_right, INPUT);

  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600);
  mpu.initialize();
  delay(2);
  balancecar.pwm1 = 0;
  balancecar.pwm2 = 0;

  Timer1.initialize(5 * 1000); //5ms
  Timer1.attachInterrupt(timerISR);


  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
  attachPinChangeInterrupt(PinA_left, Code_left, CHANGE);
  hitung_treshold();

  Serial3.print("\n");
  Serial3.println("TES TES TES TES TES TES TES TES TES");
}

////////////////////////////////////////turn//////////////////////////////////
void ResetCarState()
{
  turnl = 0;
  turnr = 0;
  front = 0;
  back = 0;
  spinl = 0;
  spinr = 0;
  turnoutput = 0;
}


unsigned long previousMillis = 0;
const long interval = 15;

// ===       Main loop       ===
void loop()
{     
     
    Serial.print(kalmanfilter.angle);
//      Serial3.print(";");
//      Serial3.print(balancecar.pwm1,0);
//      Serial3.print(";");
//      Serial3.print(balancecar.pwm2,0);
      Serial.print("\n");
//

//  unsigned long currentMillis = millis();
//  if (currentMillis - previousMillis >= interval)
//  {
//    previousMillis = currentMillis;
//    g_carstate = enSTOP;
//  }
//  else
//  {
//    
//
//  }
//baca_sensor();
 jalanKanan();
 // jalanKiri();
  
  switch (g_carstate)
  {
    case enSTOP: turnl = 0; turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
    case enRUN: ResetCarState(); front = 30; break;
    case enLEFT: turnl = 1; break;
    case enRIGHT: turnr = 1; break;
    case enBACK: ResetCarState(); back = -30; break;
    case enTLEFT: spinl = 1; break;
    case enTRIGHT: spinr = 1; break;
    default: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
  }

}


void Code_left()
{
  count_left ++;

}

void Code_right()
{
  count_right ++;
}



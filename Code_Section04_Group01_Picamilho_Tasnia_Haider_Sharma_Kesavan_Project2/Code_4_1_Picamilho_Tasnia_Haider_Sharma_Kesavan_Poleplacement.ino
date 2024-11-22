#include <Encoder.h>
#define ENA 5 // left motor
#define ENB 6 // right motor
#define BIN_1 7 // right movement
#define AIN_1 8 // left movement
#define STBY 3 // enable to turn on
#define ENCODER_PIN_A 18  // clk pin
#define ENCODER_PIN_B 2   // data pin

Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B); //connecting encnder + pins
const float degrees/Count = 180.0 / 40; // 180 degrees divided by 40 counts for reloution -> to ratio 360 deg/20pulses
const int minPWM = 100; //overcoming deadzone 
double pendang = 0; /ccurent pend angle positon
double angvel = 0;    
double prevang = 0; //last angle stored
double u = 0;//motor control

//Pole placement gains calc'd through matlab
const double K1 = 0.0480 // ang
const double K2 = -0.1562; // ang vel

void setup() {
    pinMode(ENA, OUTPUT);  //sppeed
    pinMode(ENB, OUTPUT); //speed
    pinMode(BIN_1, OUTPUT); //direction
    pinMode(AIN_1, OUTPUT); //direction
    pinMode(STBY, OUTPUT); //enable
    digitalWrite(STBY, HIGH);  //making motors high to start rotating

    Serial.begin(9600);
}

void loop() {
    long encoder = myEnc.read(); //reading encoder position by pulse rotation
    double angle = encoder * degrees/Count; //ratio of 180/40 

    //angular velocity
    angvel = (angle - prevang) / 0.01; //change in ang/time
    pendang = angle; // Current angle
    prevAngle = angle;

    u = -(K1 * pendang + K2 * angvel);//u = -K1 * x1 - K2 * x2 gain formula

int pwmValue = constrain(abs(u), minPWM, 255);  //
  if (u > 0) {
    forward(pwmValue); //u is pos
    }
  else if (u < 0) {
    backward(pwmValue); //u is neg
  } 
else {stop(); //when u=0
  }

    //printing values
    Serial.print("Angle: ");
    Serial.print(pendang);
    Serial.print("Angular Velocity: ");
    Serial.print(angvel);
    Serial.print("Control Input (u):");
    Serial.println(u);
    delay(10); 
}

void forward(int Pulse) {
    analogWrite(ENA, Pulse);
    analogWrite(ENB, Pulse);
    digitalWrite(AIN_1, HIGH);
    digitalWrite(BIN_1, HIGH);
}

void backward(int Pulse) {
    analogWrite(ENA, Pulse);
    analogWrite(ENB, Pulse);
    digitalWrite(AIN_1, LOW);
    digitalWrite(BIN_1, LOW);
}

void stop() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

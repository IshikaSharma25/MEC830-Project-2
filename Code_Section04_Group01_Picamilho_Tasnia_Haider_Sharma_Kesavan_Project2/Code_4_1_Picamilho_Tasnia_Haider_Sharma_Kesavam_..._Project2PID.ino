#include <Encoder.h>
#include <PID_v1.h>

// Pin definitions
#define ENA 5       // left motor
#define ENB 6       // right motor
#define BIN_1 7     // right movement
#define AIN_1 8     // left movement
#define STBY 3      // driver enable pin for motor rotation
#define ENCODER_PIN_A 18  // Encoder clk pin
#define ENCODER_PIN_B 2   // Encoder data pin

Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);// connecting encoder + pins
const float degreesPerCount = 180.0 / 40; // 180 divided by 40 counts for revolution -> to ratio 360 deg/20 pulses 

// PID
double Setpoint = 90, Input, Output; //90 deg is stable position
double  Kp = 5, Ki = 0.0236, Kd = 0.6;// Kp = 1.2, Ki = 0.01, Kd = 0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //input=encoder angle, output=pid correction, direct=direction


const int minPWM = 100; // overcoming motor deadzone

void setup() {
    Serial.begin(9600);
    pinMode(ENA, OUTPUT); //sppeed
    pinMode(ENB, OUTPUT); //speed
    pinMode(BIN_1, OUTPUT); //direction
    pinMode(AIN_1, OUTPUT); //direction
    pinMode(STBY, OUTPUT); //enable 
    digitalWrite(STBY, HIGH); //making motors high to start rotating

    myPID.SetMode(AUTOMATIC); //-> library solves error 
    myPID.SetOutputLimits(-255, 255); // pos for forward, neg for reverse
}

void loop() {
    long encoder = myEnc.read(); //reading encoder positiong by pulse rotation
    double angle = encoder * degreesPerCount; //ratio of 180/40

 //apply low band filter to filter out high noise (supply power, vibration etc)
   // static double prevang = 0; //clearing variable -static to hold value in all function calls
  //  angle = 0.8 * prevang + 0.2 * angle; // 80% to previous valie and 20% to current ang -> filtering
   // prevang = angle;

    Input = angle;//pnput for pid
    myPID.Compute();

    // Motor control mapping
    //limit program to stop after 45 deg (point of no return angle) was reached
   if (Output >= 45) {
    forward(constrain(abs(Output), minPWM, 255));
} else if (Output <= -45) {
    backward(constrain(abs(Output), minPWM, 255));
} else {
    stop();
}

  
    Serial.print("Encoder: ");
    Serial.print(encoder);
    Serial.print( "Angle (deg): ");
    Serial.print(angle);
    Serial.print("PID Output: ");
    Serial.println(Output);
    delay(10); 
}

// Motor control functions
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
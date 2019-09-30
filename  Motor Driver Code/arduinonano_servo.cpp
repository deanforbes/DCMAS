// Matt and Dean's servo code
// As at 0013 02/04/16
// Tidied up 18/06/16
// Pins are for Nano Clone

#include <digitalWriteFast.h>

#define encoderA  2  // PD2
#define encoderB  8  // PB0

#define HBridge_EnablePin     6   //  PD6

#define HBridge_HighPin 9 // PB1
#define HBridge_LowPin 10 // PB2

#define Step_Input_Pin              3  // PD3;
#define Dir_Input_Pin               4 // PD4;

volatile long encoder0Pos = 0;

long target = 0;
long target1 = 0;
int max_motor_torque=212;

//  correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
//  PID controller constants

float KP = 6.0 ; // PID parameter P
float KI = 0.1; // PID parameter I
float KD = 1.3; // PID parameter D

int lastError = 0;
int sumError = 0;

//Integral term min/max
int iMax = 100;
int iMin = 0;

long previousTarget = 0;
long previousMillis = 0;        // will store last time LED was updated
long interval = 5;           // interval at which to blink (milliseconds)

bool newStep = false;
bool oldStep = false;
bool dir = false;

void setPwmFrequency(int pin, int divisor) {
    byte mode;
    if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
        switch(divisor) {
        case 1: mode = 0x01; break;
        case 8: mode = 0x02; break;
        case 64: mode = 0x03; break;
        case 256: mode = 0x04; break;
        case 1024: mode = 0x05; break;
        default: return;
        }
        if(pin == 5 || pin == 6) {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        } else {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    } else if(pin == 3 || pin == 11) {
        switch(divisor) {
        case 1: mode = 0x01; break;
        case 8: mode = 0x02; break;
        case 32: mode = 0x03; break;
        case 64: mode = 0x04; break;
        case 128: mode = 0x05; break;
        case 256: mode = 0x06; break;
        case 1024: mode = 0x7; break;
        default: return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}


void setup() { 
    pinModeFast(2, INPUT);
    pinModeFast(encoderA, INPUT);
    pinModeFast(encoderB, INPUT);

    pinModeFast(HBridge_HighPin, OUTPUT);
    pinModeFast(HBridge_LowPin, OUTPUT);

    pinModeFast(Step_Inout_Pin, INPUT);
    pinModeFast(Dir_Input_Pin, INPUT);

    setPwmFrequency(HBridge_EnablePin,8);

    attachInterrupt(0, readEncoders, CHANGE);
    attachInterrupt(1, handleStepInput, RISING);

    Serial.begin (115200);
    Serial.println("start");

} 

void loop(){

    while (Serial.available() > 0) {
        KP = Serial.parseFloat();
        KD = Serial.parseFloat();
        KI = Serial.parseFloat();

        Serial.println("Setting P");
        Serial.println(KP);
        Serial.println("Setting D");
        Serial.println(KD);
        Serial.println("Setting I");
        Serial.println(KI);
    }

    if(millis() - previousTarget > 1000){
        Serial.print(encoder0Pos);
        Serial.print(',');
        Serial.println(target1);
        previousTarget=millis();
    }

    target = target1;
    docalc();
}

void docalc() {

    if (millis() - previousMillis > interval)
    {
        previousMillis = millis();

        long error = encoder0Pos - target ;

        // Do
        long PIDResult = KP * error + KD * (error - lastError) +KI * (sumError);

        lastError = error;
        sumError += error;

        //scale the sum for the integral term
        if(sumError > iMax) {
            sumError = iMax;
        }
        else if(sumError < iMin){
            sumError = iMin;
        }

        if(PIDResult > 0){
            digitalWriteFast2 ( HBridge_HighPin ,HIGH );    //write PC1 HIGH
            digitalWriteFast2 ( HBridge_LowPin ,LOW );    //write PC1 HIGH
        }

        if(PIDResult < 0){
            digitalWriteFast2 ( HBridge_HighPin , LOW );  //write PC1 LOW
            digitalWriteFast2 ( HBridge_LowPin ,HIGH );    //write PC1 HIGH
            PIDResult = -1 * PIDResult;
        }

        int motor_torque_output = map(PIDResult,0,max_motor_torque,0,255);
        if( motor_torque_output >= 255) motor_torque_output=255;
        analogWrite ( HBridge_EnablePin,  motor_torque_output );
    }
}

void readEncoders(){
    if (((PIND&B0000100)>>2) == HIGH) {   // channel A low to high transition
        if ((PINB&B0000001) == LOW) {
            encoder0Pos-- ;         // CCW
        }
        else {
            encoder0Pos++ ;         // CW
        }
    }
    else                                        // channel A high to low transition
    {
        if ((PINB&B0000001) == LOW) {   // check channel B to see which way; if(digitalRead(encoderPinB)==LOW){.... read PB0
            encoder0Pos++ ;
        }
        else {
            encoder0Pos-- ;
        }

    }

}

void handleStepInput(){
    dir=digitalReadFast2(Dir_Input_Pin});
    if (dir) target1++;
    else target1--;
}


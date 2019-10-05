#include <Servo.h>

volatile uint8_t prev1;
volatile uint8_t prev2;
volatile uint32_t risingEdge1[12];
volatile uint32_t risingEdge2[6];
volatile uint32_t uSec[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int dict_channel[6] = {0, 2, 5, 6, 10, 11};
int throttle = 0;
int steering = 0;
int readingtoSend = 0;
int armExtention = 0;
int gapper = 0;
int differential = 0;
int speed_turn = 0;
int throttleDirection = 3;
int calculatedSpeed = 0;
bool check;
int grip = 0;
//Pin map
int ePinL = 6; //PWM
int ePinR = 5; //PWM
int motorPinL1 = A1;
int motorPinL2 = A0;
int motorPinR1 = 9; //NON PWM
int motorPinR2 = 3; //NON PWM

Servo clawservo; // create servo object to control a servo
Servo armservo;

//For pin PCINT0  PCINT4 PCINT5 or 8 12 13
ISR(PCINT0_vect)
{
    uint32_t now = micros();
    uint8_t curr = PINB; // current state of the 6 input bits
    uint8_t changed = curr ^ prev1;
    int channel = 6;
    for (uint8_t mask = 0x01; mask; mask <<= 1)
    {
        if (changed & mask)
        {
            if (curr & mask)
            {
                risingEdge1[channel] = now;
            }
            else
            {
                uSec[channel] = now - risingEdge1[channel];
            }
        }
        channel++;
    }
    prev1 = curr;
}

//For pin PCINT18 PCINT20 PCINT23 or 2 6 7
ISR(PCINT2_vect)
{
    uint32_t now = micros();
    uint8_t curr2 = PIND;
    uint8_t changed = curr2 ^ prev2;
    int channel2 = 0;
    for (uint8_t mask = 0x04; mask; mask <<= 1)
    {
        if (changed & mask)
        {
            if (curr2 & mask)
            {
                risingEdge2[channel2] = now;
            }
            else
            {
                uSec[channel2] = now - risingEdge2[channel2];
            }
        }
        channel2++;
    }
    prev2 = curr2;
}

void setup()
{
    Serial.begin(9600);

    clawservo.attach(10);
    armservo.attach(11);

    pinMode(2, INPUT);  //ch1
    pinMode(4, INPUT);  //ch2
    pinMode(7, INPUT);  //ch3
    pinMode(8, INPUT);  //ch4
    pinMode(12, INPUT); //ch5
    pinMode(13, INPUT); //ch6
    pinMode(ePinL, OUTPUT);
    pinMode(ePinR, OUTPUT);
    pinMode(motorPinL1, OUTPUT);
    pinMode(motorPinL2, OUTPUT);
    pinMode(motorPinR1, OUTPUT);
    pinMode(motorPinR2, OUTPUT);

    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    // cli();
    PCMSK2 |= (1 << PCINT18);
    PCMSK2 |= (1 << PCINT20);
    PCMSK2 |= (1 << PCINT23);
    // PCMSK0 = 0x31; //BEWARE
    PCMSK0 |= (1 << PCINT0);
    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);

    // Bit 2 = enable PC vector 2 (PCINT23..16)
    // Bit 1 = enable PC vector 1 (PCINT14..8)
    // Bit 0 = enable PC vector 0 (PCINT7..0)
    PCICR |= (1 << PCIE2);
    PCICR |= (1 << PCIE0);
    // sei();
    Serial.println("PROGRAM STARTS");
}

int zeroError(int reading)
{
    if (-20 < reading && reading < 20)
    {
        readingtoSend = 0;
    }
    else
    {
        readingtoSend = reading;
    }
    if (reading < -255)
    {
        readingtoSend = -255;
    }
    if (reading > 255)
    {
        readingtoSend = 255;
    }
    return readingtoSend;
}
//Function to control direction of movement
// Direction -> 1 for forward and 2 for back and 3 for steer controll forward and 4 for steer controll backward
void pedal(int direction, bool right = true)
{
    if (direction == 1)
    {
        digitalWrite(motorPinL2, HIGH);
        digitalWrite(motorPinL1, LOW);
        digitalWrite(motorPinR2, HIGH);
        digitalWrite(motorPinR1, LOW);
    }
    else if (direction == 2)
    {
        digitalWrite(motorPinL1, HIGH);
        digitalWrite(motorPinL2, LOW);
        digitalWrite(motorPinR1, HIGH);
        digitalWrite(motorPinR2, LOW);
    }
    else if (direction == 3)
    {
        if (right)
        {
            digitalWrite(motorPinL2, HIGH);
            digitalWrite(motorPinL1, LOW);
            digitalWrite(motorPinR1, HIGH);
            digitalWrite(motorPinR2, LOW);
        }
        else
        {
            digitalWrite(motorPinL1, HIGH);
            digitalWrite(motorPinL2, LOW);
            digitalWrite(motorPinR2, HIGH);
            digitalWrite(motorPinR1, LOW);
        }
    }
    else if (direction == 4)
    {
        if (right)
        {
            digitalWrite(motorPinL1, HIGH);
            digitalWrite(motorPinL2, LOW);
            digitalWrite(motorPinR2, HIGH);
            digitalWrite(motorPinR1, LOW);
        }
        else
        {
            digitalWrite(motorPinL2, HIGH);
            digitalWrite(motorPinL1, LOW);
            digitalWrite(motorPinR1, HIGH);
            digitalWrite(motorPinR2, LOW);
        }
    }
}
//Function to control the steer of the car
//Amount -> Value between 0-255 | steerDirection -> true for right and false for left
//Bias -> Percentage reading of amount by which other motor function
//moveDirection -> true if moving forward "turn" and for back turn "false"
void steer(int amount, bool steerDirection, int bias, int moveDirection)
{
    // Serial.print(amount);
    // Serial.print("\t");
    // Serial.print(steerDirection);
    // Serial.print("\t");
    // Serial.print(bias);
    // Serial.print("\t");
    // Serial.print(moveDirection);
    // Serial.println("");
    calculatedSpeed = amount - bias;
    if (calculatedSpeed < 0)
    {
        calculatedSpeed = 0;
    }
    if (-7 < calculatedSpeed && calculatedSpeed < 7)
    {
        calculatedSpeed = 0;
    }
    Serial.println(calculatedSpeed);
    if (steerDirection)
    {
        pedal(moveDirection, steerDirection);
        analogWrite(ePinL, amount);
        analogWrite(ePinR, calculatedSpeed);
        Serial.println("ePinR " + String(calculatedSpeed));
        Serial.println("ePinL " + String(amount));
    }
    else if (!steerDirection)
    {
        pedal(moveDirection, steerDirection);
        analogWrite(ePinR, amount);
        analogWrite(ePinL, calculatedSpeed);
        Serial.println("ePinR " + String(amount));
        Serial.println("ePinL " + String(calculatedSpeed));
    }
}

void loop()
{
    // Serial.flush(); //Check for outgoing signal
    // for (int index = 0; index < 6; index++)
    // {
    //     Serial.print(uSec[dict_channel[index]]);
    //     Serial.print("\t");
    // }
    // Serial.println();

    if (uSec[dict_channel[0]] || uSec[dict_channel[1]] || uSec[dict_channel[2]] || uSec[dict_channel[3]] || uSec[dict_channel[4]] || uSec[dict_channel[5]])
    {
        steering = map(uSec[dict_channel[0]], 1072, 1856, -255, 255);
        throttle = map(uSec[dict_channel[1]], 1110, 1752, -255, 255);
        steering = zeroError(steering);
        throttle = zeroError(throttle);
        Serial.print(String(throttle));
        Serial.print("\t");
        Serial.println(String(steering));
        armExtention = map(uSec[dict_channel[2]], 1144, 1782, 0, 190);
        armExtention = zeroError(armExtention);
        if (armExtention > 180)
        {
            armExtention = 180;
        }
        // Serial.println(armExtention);
        armservo.write(armExtention);
        if (uSec[dict_channel[3]] > 1060 && uSec[dict_channel[3]] < 1410)
        {

            grip -= 10;
            clawservo.write(grip);
            if (grip < 0)
            {
                grip = 0;
            }
            delay(250);
        }
        if (uSec[dict_channel[3]] > 1440 && uSec[dict_channel[3]] < 1870)
        {
            grip += 20;
            clawservo.write(grip);
            if (grip > 180)
            {
                grip = 180;
            }
            delay(250);
        }
        differential = map(uSec[dict_channel[5]], 1000, 2010, 180, 50);
        differential = zeroError(differential);
        speed_turn = map(uSec[dict_channel[4]], 1000, 2010, 76, 255);
        speed_turn = zeroError(speed_turn);
        //     //FROM HERE Platform Code***************************************************
        if (throttle < -150)
        {
            throttleDirection = 4;
        }
        if (throttle < -10)
        {

            //reverse
            if (steering > -80 && steering < 80)
            {
                pedal(2);
                analogWrite(ePinR, abs(throttle));
                analogWrite(ePinL, abs(throttle));
            }
        }
        else if (throttle > 0)
        {
            //forward
            if (steering > -80 && steering < 80)
            {
                pedal(1);
                analogWrite(ePinR, throttle);
                analogWrite(ePinL, throttle);
            }
        }
        else
        {
            throttleDirection = 3;
            analogWrite(ePinR, 0);
            analogWrite(ePinL, 0);
        }
        if (steering < -80)
        {
            steer(speed_turn, false, differential, throttleDirection); //LEFT
        }
        else if (steering > 80)
        {
            steer(speed_turn, true, differential, throttleDirection); //RIGHT
        }
    }
}

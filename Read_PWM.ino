#include <Servo.h>

volatile uint8_t prev1;
volatile uint8_t prev2;
volatile uint32_t risingEdge1[12];
volatile uint32_t risingEdge2[6];
volatile uint32_t uSec[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //the main dictonary which sotres the values

int dict_channel[6] = {0, 2, 5, 6, 10, 11}; //Dictonary containing the ID of dict "uSec" we are interested in
int throttle = 0;
int steering = 0;
int readingtoSend = 0; //return var in zeroError function
int armExtention = 0;
int gapper = 0;
int differential = 0;
int speed_turn = 0;
int throttleDirection = 3; //To keep track weather we want to move forward or backward or turn
int calculatedSpeed = 0;   //Obtained from differential and speed_turn
int grip = 0;
int previousarmpos = 0; //Save the previous arm extension value to be able to use when the L switch is on locked position

//Pin map
int ePinL = 6; //PWM
int ePinR = 5; //PWM
int motorPinL1 = A1;
int motorPinL2 = A0;
int motorPinR1 = 9;
int motorPinR2 = 3;

Servo clawservo; // create servo object to control a servo
Servo armservo;

//Helps to reduce the fluctuations in the reading due to improper signal sent or calcutated on the
// microcontroller and also keep max value possible inside proper interval
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
    //To restrict the reading going above the desierd range
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
// Right     -> true is want to go right and false if not
void pedal(int direction, bool right = true)
{
    if (direction == 1) //Go straight
    {
        digitalWrite(motorPinL2, HIGH);
        digitalWrite(motorPinL1, LOW);
        digitalWrite(motorPinR2, HIGH);
        digitalWrite(motorPinR1, LOW);
    }
    else if (direction == 2) //Go back
    {
        digitalWrite(motorPinL1, HIGH);
        digitalWrite(motorPinL2, LOW);
        digitalWrite(motorPinR1, HIGH);
        digitalWrite(motorPinR2, LOW);
    }
    else if (direction == 3) //Turing while going straight
    {
        //which direction to turn
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
    else if (direction == 4) //Turning while going back
    {
        //which direction to turn
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
    calculatedSpeed = amount - bias; //Calculating the difference of speed we want between 2 different sides while turning

    //Negative speed is 0 for us
    if (calculatedSpeed < 0)
    {
        calculatedSpeed = 0;
    }

    //To reduce fluctuations
    if (calculatedSpeed < 7)
    {
        calculatedSpeed = 0;
    }
    if (steerDirection)
    {
        pedal(moveDirection, steerDirection);
        analogWrite(ePinL, amount);
        analogWrite(ePinR, calculatedSpeed);
    }
    else if (!steerDirection)
    {
        pedal(moveDirection, steerDirection);
        analogWrite(ePinR, amount);
        analogWrite(ePinL, calculatedSpeed);
    }
}

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

    clawservo.attach(11);
    armservo.attach(10);

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

    cli();
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
    sei();

    Serial.println("PROGRAM STARTS");
    clawservo.write(45); //initial values
    armservo.write(10);  //intital values
}

void loop()
{

    //Check if any channel is sending readin or not
    if (uSec[dict_channel[0]] || uSec[dict_channel[1]] || uSec[dict_channel[2]] || uSec[dict_channel[3]] || uSec[dict_channel[4]] || uSec[dict_channel[5]])
    {
        //Remaping the values from -255 to 255
        steering = map(uSec[dict_channel[0]], 1072, 1856, -255, 255);
        throttle = map(uSec[dict_channel[1]], 1110, 1752, -255, 255);

        // Reduce fluctuation and take care of values going above 255
        steering = zeroError(steering);
        throttle = zeroError(throttle);

        // remapping values from 0 to 160 degree
        armExtention = map(uSec[dict_channel[2]], 1144, 1782, 0, 160);

        if (armExtention > 160)
        {
            armExtention = 160;
        }

        if (armExtention > 0)
        {
            armservo.write(armExtention);
            previousarmpos = armExtention;
        }
        //When the armExtention button is place on lock use the previous reading so as to remain on that reading
        else if (armExtention < 0)
        {

            armservo.write(previousarmpos);
        }

        //When the gripper channel is shifted to left
        if (uSec[dict_channel[3]] > 1060 && uSec[dict_channel[3]] < 1410)
        {

            grip -= 20;

            //Dont allow reading to go less than 0degree
            if (grip < 0)
            {
                grip = 0;
            }
            clawservo.write(grip);
            delay(150);
        }

        //When the gripper channel is shifted to right
        else if (uSec[dict_channel[3]] > 1440 && uSec[dict_channel[3]] < 1870)
        {
            grip += 7; //Close the claw at less degree per itteration
            //Dont allow reading to go more than 45degree or else servo starts heating up
            if (grip > 45)
            {
                grip = 45;
            }
            clawservo.write(grip);
            delay(150);
        }

        differential = map(uSec[dict_channel[5]], 1000, 2010, 180, 50);
        differential = zeroError(differential);
        speed_turn = map(uSec[dict_channel[4]], 1000, 2010, 76, 255);
        speed_turn = zeroError(speed_turn);

        if (throttle < -150) //less than -150 as to not start going reverse as soon as you tilt the throttle stick by just 1 degree
        {
            throttleDirection = 4;
        }
        if (throttle < -10)
        {

            //go only reverse not left and right even if the stick is turned a bit left or right
            if (steering > -80 && steering < 80)
            {
                pedal(2);
                analogWrite(ePinR, abs(throttle));
                analogWrite(ePinL, abs(throttle));
            }
        }
        else if (throttle > 0)
        {
            ////go only forward not left and right even if the stick is turned a bit left or right
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

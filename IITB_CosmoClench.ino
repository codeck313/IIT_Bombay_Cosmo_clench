#include <Servo.h>

volatile uint8_t prev1;
volatile uint8_t prev2;
volatile uint32_t risingEdge1[12];
volatile uint32_t risingEdge2[6];
volatile uint32_t uSec[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //the main dictonary which sotres the values

int dict_channel[6] = {0, 2, 5, 6, 10, 11}; //Dictonary containing the ID of dict "uSec" we are interested in
int readingtoSend = 0;                      //return var in zeroError function

//Motor Motion
int throttle = 0;
int steering = 0;
int turn = 0;
int enableL = 6;
int IN1_left = A4;
int IN2_left = A5;
int IN1_right = A3;
int IN2_right = A2;
int motor_pwm_left = 0;
int motor_pwm_right = 0;
int motor_pwm = 0;
int steerDirection = 0;
int transverseDirection = 0;
int transverse_control = 0;
int direction_control = 0;

//Claw Motion
int gripper_status = 0;
int arm_status = 0;
int arm_control = 0;
int base_status = 0;
int gripper_pin = 3;
int arm_pin = 5;
int base_pin = 10;
int previous_gripperS = 0;
int sensor1 = A0;
int sensor2 = A1;
Servo gripper_servo; // create servo object to control a servo
Servo arm_servo;
Servo base_servo;

//Helps to reduce the fluctuations in the reading due to improper signal sent or calcutated on the
// microcontroller and also keep max value possible inside proper interval

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
    gripper_servo.attach(gripper_pin);
    arm_servo.attach(arm_pin);
    base_servo.attach(base_pin);

    pinMode(enableL, OUTPUT);
    pinMode(IN1_left, OUTPUT);
    pinMode(IN2_left, OUTPUT);
    pinMode(IN1_right, OUTPUT);
    pinMode(IN2_right, OUTPUT);
    // pinMode(sensor1, INPUT);
    // pinMode(sensor2, INPUT);
    pinMode(2, INPUT);
    pinMode(4, INPUT);
    pinMode(7, INPUT);
    pinMode(8, INPUT);
    pinMode(12, INPUT);
    pinMode(13, INPUT);

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
    gripper_servo.write(0);
    arm_servo.write(0);
    base_servo.write(92);
    delay(100);
}
int zeroError(int reading, int max, int min, int midError = 20, int terminalError = 0)
{
    if (-midError < reading && reading < midError)
    {
        readingtoSend = 0;
    }
    else
    {
        readingtoSend = reading;
    }
    //To restrict the reading going above the desierd range
    if (reading < min + terminalError)
    {
        readingtoSend = min;
    }
    if (reading > max - terminalError)
    {
        readingtoSend = max;
    }
    return readingtoSend;
}

void pedal(int direction = 0, int sDirection = 0)
{
    // Serial.println("Direction - " + String(direction) + " Steer - " + String(sDirection));
    if (direction == 1 && sDirection == 0) //Go straight
    {
        digitalWrite(IN1_left, HIGH);
        digitalWrite(IN2_left, LOW);
        digitalWrite(IN1_right, HIGH);
        digitalWrite(IN2_right, LOW);
        Serial.println("straight");
    }
    else if (direction == 2 && sDirection == 0) //Go back
    {
        digitalWrite(IN1_left, LOW);
        digitalWrite(IN2_left, HIGH);
        digitalWrite(IN1_right, LOW);
        digitalWrite(IN2_right, HIGH);
        Serial.println("back");
    }
    else if (direction == 1 && sDirection == 1) //Turing while going straight
    {
        digitalWrite(IN1_left, HIGH);
        digitalWrite(IN2_left, LOW);
        digitalWrite(IN1_right, LOW);
        digitalWrite(IN2_right, HIGH);
        Serial.println("left");
    }
    else if (direction == 1 && sDirection == 2)
    {
        digitalWrite(IN1_left, LOW);
        digitalWrite(IN2_left, HIGH);
        digitalWrite(IN1_right, HIGH);
        digitalWrite(IN2_right, LOW);
        Serial.println("right");
    }
    else if (direction == 2 && sDirection == 1) //Turning while going back
    {
        digitalWrite(IN1_left, LOW);
        digitalWrite(IN2_left, HIGH);
        digitalWrite(IN1_right, HIGH);
        digitalWrite(IN2_right, LOW);
        Serial.println("*left");
    }
    else if (direction == 2 && sDirection == 2)
    {
        digitalWrite(IN1_left, HIGH);
        digitalWrite(IN2_left, LOW);
        digitalWrite(IN1_right, LOW);
        digitalWrite(IN2_right, HIGH);
        Serial.println("*right");
    }
    else if (sDirection == 3)
    {
        digitalWrite(IN1_left, HIGH);
        digitalWrite(IN2_left, LOW);
        digitalWrite(IN1_right, LOW);
        digitalWrite(IN2_right, HIGH);
        Serial.println("left");
    }
    else if (sDirection == 4)
    {
        digitalWrite(IN1_left, LOW);
        digitalWrite(IN2_left, HIGH);
        digitalWrite(IN1_right, HIGH);
        digitalWrite(IN2_right, LOW);
        Serial.println("right");
    }
    else
    {
        digitalWrite(IN1_left, LOW);
        digitalWrite(IN2_left, LOW);
        digitalWrite(IN1_right, LOW);
        digitalWrite(IN2_right, LOW);
    }
}

void loop()
{
    // for (int index = 0; index < 6; index++)
    // {
    //     Serial.print(uSec[dict_channel[index]]);
    //     Serial.print("\t");
    // }
    // Serial.println();
    // Check if any channel is sending readin or not
    if (uSec[dict_channel[0]] || uSec[dict_channel[1]] || uSec[dict_channel[2]] || uSec[dict_channel[3]] || uSec[dict_channel[4]] || uSec[dict_channel[5]])
    {

        throttle = map(uSec[dict_channel[2]], 1140, 1812, 0, 100);
        throttle = zeroError(throttle, 100, 0, 0, 7);
        motor_pwm = map(throttle, 0, 100, 99, 255);
        if (motor_pwm > 99)
        {
            // Serial.println("Throttle at " + String(motor_pwm));
            analogWrite(enableL, motor_pwm);
        }
        else
        {
            analogWrite(enableL, 0);
        }

        // remapping values from 0 to 160 degree
        transverse_control = map(uSec[dict_channel[1]], 1105, 1750, -255, 255);
        transverse_control = zeroError(transverse_control, 255, -255);
        //When the motion control channel is shifted to up
        if (transverse_control > 80)
        {
            transverseDirection = 1;
        }

        //When the motion control channel is shifted to down
        else if (transverse_control < -80)
        {

            transverseDirection = 2;
        }
        else
        {
            transverseDirection = 0;
        }

        //When the motion control channel is shifted to left
        // Serial.println(uSec[dict_channel[0]]);
        direction_control = map(uSec[dict_channel[0]], 1108, 1856, -255, 255);
        direction_control = zeroError(direction_control, 255, -255);
        // Serial.println("Direction - " + String(transverse_control) + " Steer - " + String(direction_control));
        // -1108,1856, 1464
        if (transverseDirection == 0 && direction_control < -200)
        {
            steerDirection = 4;
        }
        else if (transverseDirection == 0 && direction_control > 200)
        {
            steerDirection = 3;
        }
        else if (direction_control > 150)
        {
            steerDirection = 1;
            // Serial.println("THIS IS GIVING          1");
        }

        //When the motion control channel is shifted to right
        else if (direction_control < -150)
        {
            steerDirection = 2;
            // Serial.println("THIS IS GIVING          2");
            // // TODO
            // Serial.println("CLOSE CLAW at" + String(gripper_status));
            // gripper_status += 5; //Close the claw at less degree per itteration
            // //Dont allow reading to go more than 45degree or else servo starts heating up
            // if (gripper_status > 180)
            // {
            //     gripper_status = 180;
            // }
            // gripper_servo.write(gripper_status);
            // // Serial.println(grip);
            // delay(75);
        }
        else
        {
            steerDirection = 0;
        }
        pedal(transverseDirection, steerDirection);

        base_status = map(uSec[dict_channel[4]], 1008, 2028, 180, 0);
        base_status = zeroError(base_status, 180, 0, 0, 5);
        gripper_status = map(uSec[dict_channel[5]], 1000, 2010, 2, 75);
        gripper_status = zeroError(gripper_status, 75, 2, 0, 0);

        base_servo.write(base_status);
        // Serial.println(base_status);
        // Serial.println(gripper_status);
        if (previous_gripperS != gripper_status)
        {
            previous_gripperS = gripper_status;
            gripper_servo.write(gripper_status);
        }
        arm_control = map(uSec[dict_channel[3]], 1064, 1864, -255, 255);
        // Reduce fluctuation and take care of values going above 255
        arm_control = zeroError(arm_control, 255, -255, 30);
        // Serial.println(arm_control);
        // 148 - 0-25 132-180 ,
        if (arm_control < -140)
        {
            // Serial.println("MOVE CLAW UP " + String(arm_status));
            arm_status -= 5; //Close the claw at less degree per itteration
            //Dont allow reading to go more than 45degree or else servo starts heating up
            if (arm_status < 0)
            {
                arm_status = 0;
            }
            arm_servo.write(arm_status);
            delay(100);
            //TODO
        }
        else if (arm_control > 140)
        {
            //TODO
            // Serial.println("MOVE CLAW DOWN" + String(arm_status));
            arm_status += 5; //Close the claw at less degree per itteration
            //Dont allow reading to go more than 45degree or else servo starts heating up
            if (arm_status > 178)
            {
                arm_status = 178;
            }
            if ((base_status > 97 && arm_status > 148) || (base_status < 41 && arm_status > 148))
            {
                arm_status = 148;
            }
            arm_servo.write(arm_status);
            // Serial.println(grip);
            delay(100);
        }
        // Serial.println(arm_status);
        // Serial.println("Sensor1 " + String(analogRead(sensor1)));
        // Serial.println("Sensor2 " + String(analogRead(sensor2)));
    }
}

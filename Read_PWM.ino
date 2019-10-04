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

    pinMode(2, INPUT);
    pinMode(4, INPUT);
    pinMode(7, INPUT);
    pinMode(8, INPUT);
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    pinMode(9, OUTPUT);
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
}

int zeroError(int reading)
{
    if (-5 < reading && reading < 5)
    {
        readingtoSend = 0;
    }
    else
    {
        readingtoSend = reading;
    }
    return readingtoSend;
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
    steering = map(uSec[dict_channel[0]], 1072, 1856, -255, 255);
    throttle = map(uSec[dict_channel[1]], 1110, 1752, -255, 255);
    steering = zeroError(steering);
    throttle = zeroError(throttle);
    Serial.print(String(throttle));
    Serial.print("\t");
    Serial.println(String(steering));
    armExtention = map(uSec[dict_channel[2]], 1144, 1782, 0, 100);
    Serial.print("armExtention \t");
    Serial.println(armExtention);
    if (uSec[dict_channel[3]] > 1060 && uSec[dict_channel[3]] < 1410)
    {
        Serial.println("Gripper OPEN");
    }
    if (uSec[dict_channel[3]] > 1440 && uSec[dict_channel[3]] < 1850)
    {
        Serial.println("Gripper CLOSED");
    }
    differential = map(uSec[dict_channel[4]], 1000, 2010, 10, 50);
    speed_turn = map(uSec[dict_channel[5]], 1000, 2010, 10, 50);
}

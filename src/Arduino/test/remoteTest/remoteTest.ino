#include <Arduino.h>

#define IBUS_RX_PIN 16
#define MOTOR_LEFT_PWM 12
#define MOTOR_RIGHT_PWM 13

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

#define NUM_CHANNELS 6
uint16_t channels[NUM_CHANNELS] = {1500}; // defaults

HardwareSerial ibusSerial(1); // Use UART1

void setup()
{
    Serial.begin(115200);

    ibusSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1); // RX only

    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_LEFT_PWM, 0);
    ledcAttach(MOTOR_RIGHT_PWM, 1);
}

uint16_t readUInt16(uint8_t *data, int offset)
{
    return data[offset] | (data[offset + 1] << 8);
}

bool readIBUSPacket()
{
    static uint8_t buf[32];
    if (ibusSerial.available() >= 32)
    {
        if (ibusSerial.read() != 0x20)
            return false; // header
        if (ibusSerial.read() != 0x40)
            return false; // length

        for (int i = 0; i < 30; i++)
        {
            buf[i] = ibusSerial.read();
        }

        for (int ch = 0; ch < NUM_CHANNELS; ch++)
        {
            channels[ch] = readUInt16(buf, ch * 2);
        }
        return true;
    }
    return false;
}

int mapToPWM(int pulse, bool bidirectional = false)
{
    if (bidirectional)
    {
        return map(pulse, 1000, 2000, -255, 255);
    }
    else
    {
        return map(pulse, 1000, 2000, 0, 255);
    }
}

void setMotorPWM(int pwm_left, int pwm_right)
{
    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);
    ledcWrite(0, abs(pwm_left));
    ledcWrite(1, abs(pwm_right));
}

void loop()
{
    if (readIBUSPacket())
    {
        int throttle = mapToPWM(channels[1], true); // e.g., CH2 = Throttle
        int steering = mapToPWM(channels[0], true); // e.g., CH1 = Steering

        int left_motor = throttle + steering;
        int right_motor = throttle - steering;
        setMotorPWM(left_motor, right_motor);

        Serial.print("Channels: ");
        for (int i = 0; i < NUM_CHANNELS; ++i)
        {
            Serial.printf("CH%d: %d  ", i + 1, channels[i]);
        }
        Serial.printf("=> L: %d  R: %d\n", left_motor, right_motor);
    }
}

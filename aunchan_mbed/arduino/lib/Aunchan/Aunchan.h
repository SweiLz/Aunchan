#ifndef _AUNCHAN_H_
#define _AUNCHAN_H_

#include <Arduino.h>

#define I2C_SDA PB9
#define I2C_SCL PB8

#define LED_PIN PC1
#define BATTERY_PIN PC0

#define M1_PWM_OUT_COMP_A PC8
#define M1_PWM_OUT_COMP_B PB1
#define M1_ENC_A PA11
#define M1_ENC_B PA12

#define M2_PWM_OUT_COMP_A PA7
#define M2_PWM_OUT_COMP_B PC6
#define M2_ENC_A PC4
#define M2_ENC_B PC5

class Motor
{
  private:
    int8_t pwmA, pwmB, encA, encB;
    volatile int32_t enc_count;
    int32_t last_enc_count, diff_enc_count;

  public:
    Motor(int8_t pwmA_pin, int8_t pwmB_pin, int8_t encA_pin, int8_t encB_pin)
    {
        pwmA = pwmA_pin;
        pwmB = pwmB_pin;
        encA = encA_pin;
        encB = encB_pin;
        pinMode(pwmA, PWM);
        pinMode(pwmB, PWM);
        pinMode(encA, INPUT);
        pinMode(encB, INPUT);
        last_enc_count = 0;
        enc_count = 0;
        diff_enc_count = 0;
    }
    int32_t Diff()
    {
        diff_enc_count = enc_count - last_enc_count;
        last_enc_count = enc_count;
        return diff_enc_count;
    }
    int32_t getDiff()
    {
        return diff_enc_count;
    }
    int32_t getCount()
    {
        return enc_count;
    }
    void resetCount()
    {
        enc_count = 0;
    }
    void intEnc()
    {
        if (digitalRead(encA))
            digitalRead(encB) ? enc_count++ : enc_count--;
        else
            digitalRead(encB) ? enc_count-- : enc_count++;
    }
    void int2Enc()
    {
        if (digitalRead(encB))
            digitalRead(encA) ? enc_count-- : enc_count++;
        else
            digitalRead(encA) ? enc_count++ : enc_count--;
    }
    void setPWM(int16_t pwm)
    {
        pwm = constrain(pwm, -32767, 32767);
        if (pwm == 0)
        {
            pwmWrite(pwmA, 65535);
            pwmWrite(pwmB, 65535);
        }
        else
        {
            pwmWrite(pwmA, 32767 - pwm);
            pwmWrite(pwmB, 32767 + pwm);
        }
    }
};

class PID
{
  private:
    float Kp, Ki, Kd;

    int32_t maxError;

  public:
    int32_t error, sum_error, dif_error, last_error;
    PID()
    {
        Kp = 18 ;
        Ki = 4;
        Kd = 10;
        // maxError = 400;
    }
    void setPID(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = Ki;
        Kd = Kd;
    }

    int32_t computePID(int32_t target, int32_t current,int32_t comp = 0)
    {
        error = target - current;
        sum_error += error;
        // sum_error = constrain(sum_error, -maxError, maxError);
        dif_error = error - last_error;
        last_error = error;
        return comp + Kp * error + Ki * sum_error + Kd * dif_error;
    }
};

class Aunchan
{
  private:
    bool ledState;
    float batteryState;

  public:
    Motor *motor[2];
    PID *pid[2];

    Aunchan();

    float getBattery(void);
    void setLED(bool state);
    void speedControl(int16_t m1_lin_vel, int16_t m2_lin_vel);
    void motorDrive(int16_t m1_pwm, int16_t m2_pwm);
};

Aunchan::Aunchan()
{
    pinMode(LED_PIN, OUTPUT);
    motor[0] = new Motor(M1_PWM_OUT_COMP_A, M1_PWM_OUT_COMP_B, M1_ENC_A, M1_ENC_B);
    motor[1] = new Motor(M2_PWM_OUT_COMP_A, M2_PWM_OUT_COMP_B, M2_ENC_A, M2_ENC_B);
    pid[0] = new PID();
    pid[1] = new PID();
}

float Aunchan::getBattery(void)
{
    batteryState = constrain(0.0089 * analogRead(BATTERY_PIN) - 0.032, 0, 13);
    return batteryState;
}

void Aunchan::setLED(bool state)
{
    ledState = state;
    digitalWrite(LED_PIN, ledState);
}

void Aunchan::speedControl(int16_t pos_sp_m1, int16_t pos_sp_m2)
{
    // int16_t m1_pwm = pid[0]->computePID(m1_diff_sp, motor[0]->Diff(),14*m1_diff_sp);
    // int16_t m2_pwm = pid[1]->computePID(m2_diff_sp, motor[1]->Diff(),14*m2_diff_sp);
    int16_t m1_pwm = pid[0]->computePID(pos_sp_m1,motor[0]->getCount());
    int16_t m2_pwm = pid[1]->computePID(pos_sp_m2,motor[1]->getCount());
    motorDrive(m1_pwm, m2_pwm);
}

void Aunchan::motorDrive(int16_t m1_pwm, int16_t m2_pwm)
{
    m1_pwm = constrain(m1_pwm, -10000, 10000);
    m2_pwm = constrain(m2_pwm, -10000, 10000);
    m1_pwm = map(m1_pwm, -10000, 10000, -32767, 32767);
    m2_pwm = map(m2_pwm, -10000, 10000, -32767, 32767);
    motor[0]->setPWM(m1_pwm);
    motor[1]->setPWM(m2_pwm);
}

#endif
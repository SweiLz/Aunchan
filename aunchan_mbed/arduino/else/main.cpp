#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>

#define LED PC1
#define R_PWM_OUT_COMP_A PC8
#define R_PWM_OUT_COMP_B PB1
#define R_ENC_A PA11
#define R_ENC_B PA12
#define L_ENC_A PC4
#define L_ENC_B PC5

// ros::NodeHandle nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// char hello[13] = "hello world!";

#define TICK_PER_REV 390

volatile int bLED = LOW;
volatile int32_t count = 0,count_temp,count_temp2=0,count_last=0,count_x=0;
volatile float r_vel,r_vel_l =0;
long time_start;
HardwareTimer timer(1);
void R_Drive(int16_t pwm);

void interruptFunction() {
    if(digitalRead(R_ENC_A))
        digitalRead(R_ENC_B) ? count++:count--;
    else
        digitalRead(R_ENC_B) ? count--:count++;
}
void interruptFunction2() {
    if(digitalRead(R_ENC_B))
        digitalRead(R_ENC_A) ? count--:count++;
    else
        digitalRead(R_ENC_A) ? count++:count--;
}


int16_t pidCompute(int16_t target, int16_t current)
{
    return 0;

}

void handler_led()
{
    // digitalWrite(LED,bLED);
    // bLED = !bLED;
    count_temp = count;
    r_vel = (count_temp - count_last);
    count_last = count_temp;
}

void setup() {
    // nh.initNode();
    // nh.advertise(chatter);
    pinMode(LED,OUTPUT);
    pinMode(R_ENC_A,INPUT);
    pinMode(R_ENC_B,INPUT);
    pinMode(R_PWM_OUT_COMP_A, PWM);
    pinMode(R_PWM_OUT_COMP_B, PWM);
    timer.pause();
    timer.setPeriod(10000);
    timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH1, 1);
    timer.attachCompare1Interrupt(handler_led);
    timer.refresh();
    timer.resume();

    attachInterrupt(R_ENC_A, interruptFunction, CHANGE);
    attachInterrupt(R_ENC_B, interruptFunction2, CHANGE);

    Serial1.begin(9600);
    Serial1.println("Hello World");
    delay(5000);
}


// class Motor()

void R_Drive(int16_t pwm)
{
    pwm = constrain(pwm,-10000,10000);
    pwm = map(pwm,-10000,10000,-32767,32767);
    if(pwm == 0){
        pwmWrite(R_PWM_OUT_COMP_A,65535);
        pwmWrite(R_PWM_OUT_COMP_B,65535);
    }
    else{
        pwmWrite(R_PWM_OUT_COMP_A,32767-pwm);
        pwmWrite(R_PWM_OUT_COMP_B,32767+pwm);
    }
}
int16_t target=-10,error,l_error=0,s_error=0;
void loop() {
    // digitalWrite(LED,bLED);
    // Serial1.println(count_temp);
    
    // count_x += 10;
    // int16_t pw = (r_vel-1000)*0.28;
    error=target-r_vel;
    s_error += error;
    s_error = constrain(s_error ,-50,50);
    int16_t pwm_out = map(target,-100,100,-15000,15000)+(error)*250+(s_error*50)-(error-l_error)*50;
    Serial1.print(r_vel);
    Serial1.print(' ');
    Serial1.println(s_error);

    R_Drive(pwm_out);
    l_error = error;
    // r_vel_l = r_vel;
    delay(100);
    

    // str_msg.data = hello;
    // chatter.publish(&str_msg);
    // nh.spinOnce();
    // for(int i =0;i<=65535;i++)
    // {
    //     R_Drive(i);
    //     delay(1);
    // }
    // pwmWrite(R_PWM_OUT_COMP_A, 32768);
    // pwmWrite(R_PWM_OUT_COMP_B, 32768);
    // pwmWrite(R_PWM_OUT_COMP_A, 0);
    // pwmWrite(R_PWM_OUT_COMP_B, 65535);
    // pwmWrite(R_PWM_OUT_COMP_A, 65535);
    // pwmWrite(R_PWM_OUT_COMP_B, 0);
    // R_Drive(1000);
    // digitalWrite(PC1, HIGH);  
    // delay(500);             
    // digitalWrite(PC1, LOW);    
    // delay(500);
    
    // pwmWrite(R_PWM_OUT_COMP_A,32767+pwm);
    // pwmWrite(R_PWM_OUT_COMP_B,32767-pwm);
    // R_Drive(0);
    // delay(500);
    // for(int i=0;i<=1000;i+=50)
    // {
    //     R_Drive(i);
    //     delay(100);
    // }         
    // for(int i=1000;i>=-1000;i-=50)
    // {
    //     R_Drive(i);
    //     delay(100);
    // }    
    // for(int i=-1000;i<=0;i+=50)
    // {
    //     R_Drive(i);
    //     delay(100);
    // }  
    // digitalWrite(PC1, HIGH);  
    // delay(500);             
    // digitalWrite(PC1, LOW);    
    // delay(500);
    // R_Drive(0);
    // delay(500);             
    // Serial1.print("Hi, ");
    // Serial1.println(count);

}

// #define PWM_OUT PA8       //PWM output
// #define PWM_OUT_COMP PB13 //complementary output

//   pinMode(PWM_OUT, PWM); //Ideally these would be done as bit wise as I dont know exactly whats being set here..
//   pinMode(PWM_OUT_COMP, PWM);

  
//   timer_dev *t = TIMER1; //refers t to Timer 8 memory location, how to read back?
//   timer_reg_map r = t->regs;
   
//   bitSet(r.adv->CCER,0); //this should enable complimentary outputs
//   bitSet(r.adv->CCER,2);
//   pwmWrite(PWM_OUT, 10);

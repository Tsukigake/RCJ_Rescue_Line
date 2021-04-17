#include "Wire.h"
#include "VL53L0X.h"
#include "setup_mega.h"

void main_motor_control_system(double,double,double,long,int);
void sub_motor_control_system(int,int,int);
void MDM66126CH_control_system(int,int,int,int,long,int);
void MDM67H4504CH_control_system(int,int,int,int,long,int);
void LDM46165CH_contorl_system(int,int,int,int,long);

//motor controal system
#define RPM_63_1 225
#define wide_a 0
#define wide_b 0
#define Rw 0

//MDM66126CH and MDM67H4504CH GPIO setup
typedef struct{
    const int GPIO_IN_1[6] = {0,0,0,0,0,0};
    const int GPIO_IN_2[6] = {0,0,0,0,0,0};
    const int GPIO_IN_PWM[6] = {0,0,0,0,0,0};
    const bool active = false;
} MDM66126CH_n;
MDM66126CH_n MDM66126CH;
typedef struct{
    const int GPIO_IN_1[4] = {3,6,8,12};
    const int GPIO_IN_2[4] = {2,5,7,11};
    const bool active = true;
} MDM67H4504CH_n;
MDM67H4504CH_n MDM67H4504CH;

//PTM750225CH and LDM46165CH GPIO setup
typedef struct{
    const uint8_t GPIO_IN_DIGITAL[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
    const int GPIO_IN_ANALOG[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
} PTM750225CH_n;
PTM750225CH_n PTM750225CH;
typedef struct{
    const uint8_t GPIO_IN_WHITE[2] = {28,30};
    const uint8_t GPIO_IN_RGB[3] = {26,24,22};
} LDM46165CH_n;
LDM46165CH_n LDM46165CH;

//VL53L0X setup
#define SENSOR_NUM 8
#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)
const int GPIO_MASK_ARRAY[SENSOR_NUM] = {0,0,0,0,0,0,0,0};
VL53L0X gSensor[SENSOR_NUM];

int waiting_time = 0;

void setup()
{
    //moter,ft sensor,led pinmode setup
    if(MDM66126CH.active == false && MDM67H4504CH.active == true){
        for(int i = 0;i < 6;i++){
            pinMode(MDM66126CH.GPIO_IN_1[i],OUTPUT);
            pinMode(MDM66126CH.GPIO_IN_2[i],OUTPUT);
        }   
    }
    for(int i = 0;i < 13;i++) pinMode(PTM750225CH.GPIO_IN_DIGITAL[i],OUTPUT);    
    for(int i = 0;i < 2;i++) pinMode(LDM46165CH.GPIO_IN_WHITE[i],OUTPUT);
    for(int i = 0;i < 3;i++) pinMode(LDM46165CH.GPIO_IN_RGB[i],OUTPUT);

    //I2C setup
    Wire.begin(7);
}

void loop() 
{ 
    //LDM46165CH_contorl_system(off,off,off,on,1000);
    MDM67H4504CH_control_system(0,0,0,0,0,no_motion);
    delay(200);
}

void main_motor_control_system(double speed,double running_angle,double spin_angle,long time,int final_motion){
    int PHIw[4];
    double PHIwR[4],PHIwS[4],Vx,Vy;

    if(spin_angle == 0){
        //Angle running only.
        //The unit of output value is speed.
        Vx = speed * cos(running_angle);
        Vy = speed * sin(running_angle);
        PHIw[0] = (Vx - Vy - (wide_a + wide_b) * spin_angle) / Rw;
        PHIw[1] = (Vx + Vy - (wide_a + wide_b) * spin_angle) / Rw;
        PHIw[2] = (Vx - Vy + (wide_a + wide_b) * spin_angle) / Rw;
        PHIw[3] = (Vx + Vy + (wide_a + wide_b) * spin_angle) / Rw;
    } else if(running_angle == 0){
        //Spin turn only.
        //The unit of output value is speed.
        Vx = 0;
        Vy = 0;
        PHIw[0] = (((Vx - Vy - (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        PHIw[1] = (((Vx + Vy - (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        PHIw[2] = (((Vx - Vy + (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        PHIw[3] = (((Vx + Vy + (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        for(int i = 0;i < 4; i++){
            constrain(PHIw[i],-225,225);
        }
    } else {
        //Angle running and spin turn.
        //The unit of output value is speed.
        Vx = speed * cos(running_angle);
        Vy = speed * sin(running_angle);
        PHIwR[0] = (Vx - Vy - (wide_a + wide_b) * spin_angle) / Rw;
        PHIwR[1] = (Vx + Vy - (wide_a + wide_b) * spin_angle) / Rw;
        PHIwR[2] = (Vx - Vy + (wide_a + wide_b) * spin_angle) / Rw;
        PHIwR[3] = (Vx + Vy + (wide_a + wide_b) * spin_angle) / Rw;
        Vx = 0;
        Vy = 0;
        PHIw[0] = (((Vx - Vy - (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        PHIw[1] = (((Vx + Vy - (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        PHIw[2] = (((Vx - Vy + (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        PHIw[3] = (((Vx + Vy + (wide_a + wide_b) * spin_angle) / Rw) / time) / (RPM_63_1 / 60 / 1000) * 225;
        for(int i = 0;i < 4; i++){
            constrain(PHIw[i],-225,225);
            PHIw[i] = PHIwR[i] + PHIwS[i];
        }
    }
    if(MDM67H4504CH.active == true);
}

void sub_motor_control_system(int mode,int angle,int speed){
    int now_time;
    now_time = millis();
    if(waiting_time > now_time){
        delay(waiting_time - now_time);
    }
    waiting_time = 0;
    constrain(angle,-360,360);
    map(speed,0,100,0,255);
    constrain(speed,0,255);
    Wire.beginTransmission(8);
    Wire.write(mode);
    Wire.endTransmission();
    delay(10);
    if(angle < 0) angle = angle + 1000;
    Wire.beginTransmission(8);
    Wire.write(angle);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(8);
    Wire.write(speed);
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(8,6);
    while(Wire.available()){
        waiting_time =  Wire.read();
    }
    waiting_time += millis(); 
}

void MDM67H4504CH_control_system(int a,int b,int c,int d,long time,int final_motion){
    int num[4] = {a,b,c,d};
    for(int i = 0;i < 4;i++){
        if(num[i] != 0){
            if(num[i] > 0){
                constrain(num[i],0,100);
                analogWrite(MDM67H4504CH.GPIO_IN_1[i],map(num[i],0,100,12,255));
                analogWrite(MDM67H4504CH.GPIO_IN_2[i],0);
            } else if(num[i] < 0){
                num[i] = num[i] * -1;
                constrain(num[i],0,100);
                analogWrite(MDM67H4504CH.GPIO_IN_1[i],0);
                analogWrite(MDM67H4504CH.GPIO_IN_2[i],map(num[i],0,100,12,255));
            }
            if(time != 0) delay(time);
        }
    }
    if(final_motion != 0){
        for(int i = 0;i < 4;i++){
            if(num[i] == 0){
                if(final_motion == 1){
                    analogWrite(MDM67H4504CH.GPIO_IN_1[i],0);
                    analogWrite(MDM67H4504CH.GPIO_IN_2[i],0);
                }else if(final_motion == 2){
                    analogWrite(MDM67H4504CH.GPIO_IN_1[i],255);
                    analogWrite(MDM67H4504CH.GPIO_IN_2[i],255);
                }
            }
        }
    }
}

void MDM66126CH_control_system(int x_a,int x_b,int y_a,int y_b,int z,long time,int final_motion){
    int num[5] = {x_a,x_b,y_a,y_b,z};
    for(int i = 0;i < 5;i++){
        if(num[i] != 0){
            if(num[i] > 0){
                constrain(num[i],0,100);
                digitalWrite(MDM66126CH.GPIO_IN_1[i],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[i],LOW);
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],map(num[i],0,100,0,255));
            } else if(num[1] < 0){
                num[i] = num[i] * -1;
                constrain(num[i],0,100);
                digitalWrite(MDM66126CH.GPIO_IN_1[i],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[i],HIGH);
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],map(num[i],0,100,0,255));
            }
            if(time != 0) delay(time);
        }
    }
    if(final_motion != 0){
        for(int i = 0;i < 6;i++){
            if(num[1] == 0){
                if(final_motion == 1){
                    digitalWrite(MDM66126CH.GPIO_IN_1[i],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[i],LOW);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[i],255);
                } else if(final_motion == 2){
                    digitalWrite(MDM66126CH.GPIO_IN_1[i],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[i],HIGH);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[i],0);
                }
            }
        }   
    }
}

void LDM46165CH_contorl_system(int r,int g,int b,int w,long time){
    int rgb[3] = {r,g,b};
    if(w == 1){
        digitalWrite(LDM46165CH.GPIO_IN_WHITE[0],HIGH);
        digitalWrite(LDM46165CH.GPIO_IN_WHITE[1],HIGH);
    } else if(w == 0){
        digitalWrite(LDM46165CH.GPIO_IN_WHITE[0],LOW);
        digitalWrite(LDM46165CH.GPIO_IN_WHITE[1],LOW);
    }
    for(int i = 0;i < 4;i++){
        if(rgb[i] == 1){
            digitalWrite(LDM46165CH.GPIO_IN_RGB[i],HIGH);
        } else if(rgb[i] == 0){
            digitalWrite(LDM46165CH.GPIO_IN_RGB[i],LOW);
        }
    }
    if(time != 0) delay(time);
}



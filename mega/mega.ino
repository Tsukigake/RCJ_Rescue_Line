/*
The MIT License (MIT)

Copyright (c) 2015 bpyamasinn.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "VL53L0X.h"
#include "Wire.h"
#include "setup_mega.h"

void main_motor_control_system(double,double,double,long,int);
void MDM66126CH_control_system(int,int,int,int,long,int);
void MDM67H4504CH_control_system(int,int,int,int,long,int);
void LDM46165CH_contorl_system(int,int,int,int,long);
int Get_Button_State(int);
void Color_sensor_get_value(int);
void Vl53L0X_contorl_system(int,int);
int M5stack_contorl_system(int);

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
    const uint8_t GPIO_IN_DIGITAL[13] = {53,51,49,47,45,43,41,39,37,35,33,31,29};
    const uint8_t GPIO_IN_ANALOG[12] = {15,14,13,12,11,10,9,8,7,6,5,4};
    const uint8_t GPIO_IN_Color[4] = {15,6,4,13};
} PTM750225CH_n;
PTM750225CH_n PTM750225CH;
typedef struct{
    const uint8_t GPIO_IN_WHITE[2] = {28,30};
    const uint8_t GPIO_IN_RGB[3] = {26,24,22};
} LDM46165CH_n;
LDM46165CH_n LDM46165CH;

//sensor value
typedef struct{
    int color[4];
    int reflect_state[25];
    int reflect_value[12];
    int distance[8];
} Sensor_Value_n;
Sensor_Value_n Sensor_Value;

//rescue zone value
typedef struct{
    int VL53L0X_first_situation_x;
    int VL53L0X_first_situation_y;
    int VL53L0X_first_situation_angle;
    int Reference_angle_x;
    int Reference_angle_y;
} rescue_zone_n;
rescue_zone_n rescue_zone;

//coordinate map
int coordinate_map_first_zone[120][90];
int coordinate_map_second_zone[120][90];

//VL53L0X setup
#define SENSOR_NUM 8
#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)
const int GPIO_MASK_ARRAY[SENSOR_NUM] = {38,40,42,44,46,48,50,52};
bool VL53L0X_GPIO_ERROR[8];
const bool VL53L0X_SERIAL_OUTPUT = true;
VL53L0X gSensor[SENSOR_NUM];

void setup()
{
    Wire.begin(7);  //I2C setup
    Serial.begin(9600); //serial setup

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

    //button setup
    pinMode(34,INPUT);
    pinMode(32,INPUT);

    //VL53L0X void setup
    for(int i = 0;i < SENSOR_NUM;i++){
        pinMode(GPIO_MASK_ARRAY[i], OUTPUT);
        digitalWrite(GPIO_MASK_ARRAY[i], LOW);
    }
    for(int i = 0;i < SENSOR_NUM;i++){   // センサを初期化
        pinMode(GPIO_MASK_ARRAY[i], INPUT);
        if(gSensor[i].init() == true){
            gSensor[i].setTimeout(500);
            int address = ADDRESS_00 + (i * 2);
            gSensor[i].setAddress(address);
            VL53L0X_GPIO_ERROR[i] = false;
            if(VL53L0X_SERIAL_OUTPUT == true){
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" ok");
            }
        } else {
            VL53L0X_GPIO_ERROR[i] = true;
            if(VL53L0X_SERIAL_OUTPUT == true){
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" error");
            }
        }
    }
}

void loop() 
{ 
    //VL53L0X_contorl_system(distance_mode,DEFAULT);
    //Serial.println(Sensor_Value.distance[0]);
    delay(50);
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
    if(MDM67H4504CH.active == true){
        MDM67H4504CH_control_system(PHIw[0],PHIw[2],PHIw[3],PHIw[4],time,final_motion);
    }
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
            if(num[i] == 0){
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

int Get_Button_State(int num){
    int state;
    if(num == GREEN){
        state = digitalRead(32);
        if(state == 1){
            return 0;
        } else {
            return 1;
        }
    }
    if(num == RED){
        state = digitalRead(34);
        if(state == 1){
            return 0;
        } else {
            return 1;
        }
    }
}

void Color_sensor_get_value(int continue_time){
    long red[4],green[4],blue[4];
    for(int p = 0;p < continue_time;p++){
        LDM46165CH_contorl_system(ON,OFF,OFF,OFF,1); //red
        for(int i = 0;i < 4;i++){
            if(continue_time == 1){
                red[i] = analogRead(PTM750225CH.GPIO_IN_Color[i]); //get red reflect
            } else {
                red[i] = (red[i] + analogRead(PTM750225CH.GPIO_IN_Color[i])) / 2; //get red reflect
            }
        }
        delay(1);
        LDM46165CH_contorl_system(OFF,ON,OFF,OFF,1); //green
        for(int i = 0;i < 4;i++){
            if(continue_time == 1){
                green[i] = analogRead(PTM750225CH.GPIO_IN_Color[i]); //get green reflect
            } else {
                green[i] = (green[i] + analogRead(PTM750225CH.GPIO_IN_Color[i])) / 2; //get green reflect
            }
        }
        delay(1);
        LDM46165CH_contorl_system(OFF,OFF,ON,OFF,1); //blue
        for(int i = 0;i < 4;i++){
            if(continue_time == 1){
                blue[i] = analogRead(PTM750225CH.GPIO_IN_Color[i]); //get blue reflect
            } else {
                blue[i] = (blue[i] + analogRead(PTM750225CH.GPIO_IN_Color[i])) / 2; //get blue reflect
            }
        }
        delay(1);
        LDM46165CH_contorl_system(OFF,OFF,OFF,OFF,0);
    }
    for(int i = 0;i < 4;i++){
        if(red[i] > green[i] && red[i] > blue[i]){
            if(green[i] > blue[i]){
                Sensor_Value.color[i] = (60 * (green[i] - blue[i]) / (red[i] -blue[i])) % 360;
            } else if(blue[i] > green[i]){
                Sensor_Value.color[i] = (60 * (green[i] - blue[i]) / (red[i] - green[i])) % 360;
            } else if(blue[i] == green[i]){
                Sensor_Value.color[i] = (60 * (green[i] - blue[i]) / 0) % 360;
            }
        } else if(green[i] > red[i] && green[i] > blue[i]){
            if(red[i] > blue[i]){
                Sensor_Value.color[i] = (60 * (blue[i] - red[i]) / (green[i] - blue[i])) + 120;
            } else if(blue[i] > red[i]){
                Sensor_Value.color[i] = (60 * (blue[i] - red[i]) / (green[i] - red[i])) + 120;
            } else if(blue[i] == red[i]){
                Sensor_Value.color[i] = (60 * (blue[i] - red[i]) / 0) + 120;
            }
        } else if(blue[i] > red[i] && blue[i] > green[i]){
            if(red[i] > green[i]){
                Sensor_Value.color[i] = (60 * (red[i] - green[i]) / (blue[i] - green[i])) + 240;
            } else if(green[i] > red[i]){
                Sensor_Value.color[i] = (60 * (red[i] - green[i]) / (blue[i] - red[i])) + 240; 
            } else if(green[i] == red[i]){
                Sensor_Value.color[i] = (60 * (red[i] - green[i]) / 0) + 240;
            }
        } else if(red[i] == green[i] && green[i] == blue[i] && blue[i] == red[i]){
            Sensor_Value.color[i] = 0;
        }
    }
}

void VL53L0X_contorl_system(int mode,int VL53L0X_setting){
    int VL53L0X_distace[8];
    bool change_setup = false;
    bool high_speed_mode;
    int M5_gyro_yaw;
    switch(mode){
    case distance_mode:
        switch(VL53L0X_setting){
        case DEFAULT:
            if(change_setup == true){
                for(int i = 0;i < SENSOR_NUM;i++){
                    if(VL53L0X_GPIO_ERROR[i] == false){
                        if(gSensor[i].timeoutOccurred() == false){
                            gSensor[i].setSignalRateLimit(0.25);
                            gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                            gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                            gSensor[i].setMeasurementTimingBudget(33);
                            gSensor[i].setMeasurementTimingBudget(200);
                        }
                    }
                }
                change_setup = false;
            }
            break;
        case LONG_RANGE:
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        // lower the return signal rate limit (default is 0.25 MCPS)
                        gSensor[i].setSignalRateLimit(0.1);
                        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,18);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,14);
                    }
                }
            }
            change_setup = true;
            break;
        case HIGH_SPEED:
            high_speed_mode = true;
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        // reduce timing budget to 20 ms (default is about 33 ms)
                        gSensor[i].setMeasurementTimingBudget(20000);
                    }
                }
                change_setup = true;
            }
        case HIGH_ACCURACY:
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        // increase timing budget to 200 ms
                        gSensor[i].setMeasurementTimingBudget(200000);
                    }
                }
                change_setup = true;
            }
        default:
            break;
        }
        for(int i = 0;i < SENSOR_NUM;i++){
            Sensor_Value.distance[i] = 0;
            if(high_speed_mode == true){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    Sensor_Value.distance[i] = gSensor[i].readRangeSingleMillimeters();
                    if(gSensor[i].timeoutOccurred()){
                        Sensor_Value.distance[i] = -1;
                    }
                }
                high_speed_mode = false;
            } else if(high_speed_mode == false){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    for(int p = 0;p < 2;p++){
                        Sensor_Value.distance[i] += gSensor[i].readRangeSingleMillimeters();
                        if(p == 1){
                            Sensor_Value.distance[i] = Sensor_Value.distance[i] / 2;
                        }   
                    }
                    if(gSensor[i].timeoutOccurred()){
                        Sensor_Value.distance[i] = -1;
                    }
                } else if(VL53L0X_GPIO_ERROR[i] == true){
                    Sensor_Value.distance[i] = -1;
                }
            }
        }
        if(change_setup == true){
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        gSensor[i].setSignalRateLimit(0.25);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                        gSensor[i].setMeasurementTimingBudget(33);
                        gSensor[i].setMeasurementTimingBudget(200);
                    }
                }
            }
            change_setup = false;
        }
        break;
    case coordinate_setup_mode:
        if(change_setup == true){
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        gSensor[i].setSignalRateLimit(0.25);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                        gSensor[i].setMeasurementTimingBudget(33);
                        gSensor[i].setMeasurementTimingBudget(200);
                    }
                }
            }
            change_setup = false;
        }
        for(int i = 0;i < SENSOR_NUM;i++){
            VL53L0X_distace[i] = 0;
            for(int p = 0;p < 2;p++){
                VL53L0X_distace[i] += gSensor[i].readRangeSingleMillimeters();
                if(p == 1){
                    VL53L0X_distace[i] = VL53L0X_distace[i] / 2;
                }
            }
            if(gSensor[i].timeoutOccurred()){
                VL53L0X_distace[i] = -1;
            }
        }
        if(VL53L0X_distace[7] > 90 || VL53L0X_distace[8] > 90){
            if(abs(VL53L0X_distace[5] - VL53L0X_distace[6]) != 0){
                if(VL53L0X_distace[5] > VL53L0X_distace[6]){
                    while(abs(VL53L0X_distace[5] - VL53L0X_distace[6]) < 10){
                        main_motor_control_system(0,0,-10,100,stop);
                        VL53L0X_distace[5] = gSensor[5].readRangeSingleMillimeters();
                        VL53L0X_distace[6] = gSensor[6].readRangeSingleMillimeters();
                    }
                } else if(VL53L0X_distace[5] < VL53L0X_distace[6]){
                    while(abs(VL53L0X_distace[5] - VL53L0X_distace[6]) < 10){
                        main_motor_control_system(0,0,10,100,stop);
                        VL53L0X_distace[5] = gSensor[5].readRangeSingleMillimeters();
                        VL53L0X_distace[6] = gSensor[6].readRangeSingleMillimeters();
                    }
                }
            }
            if(VL53L0X_distace[5] == 8190 || VL53L0X_distace[6] == 8190){
                while(VL53L0X_distace[5] != 8190 && VL53L0X_distace[6] != 8190){
                    main_motor_control_system(30,0,0,200,stop);
                    VL53L0X_distace[5] = gSensor[5].readRangeSingleMillimeters();
                    VL53L0X_distace[6] = gSensor[6].readRangeSingleMillimeters();
                }
            }
        }
        M5_gyro_yaw = M5stack_contorl_system(M5_GYRO,yaw);
        for(int i = 0;i < SENSOR_NUM;i++){
            
        }
        break;
    case coordinate_mode:
        switch(VL53L0X_setting){
        case DEFAULT:
            if(change_setup == true){
                for(int i = 0;i < SENSOR_NUM;i++){
                    if(VL53L0X_GPIO_ERROR[i] == false){
                        if(gSensor[i].timeoutOccurred() == false){
                            gSensor[i].setSignalRateLimit(0.25);
                            gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                            gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                            gSensor[i].setMeasurementTimingBudget(33);
                            gSensor[i].setMeasurementTimingBudget(200);
                        }
                    }
                }
                change_setup = false;
            }
            break;
        case LONG_RANGE:
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        // lower the return signal rate limit (default is 0.25 MCPS)
                        gSensor[i].setSignalRateLimit(0.1);
                        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,18);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,14);
                    }
                }
            }
            change_setup = true;
            break;
        case HIGH_SPEED:
            high_speed_mode = true;
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        // reduce timing budget to 20 ms (default is about 33 ms)
                        gSensor[i].setMeasurementTimingBudget(20000);
                    }
                }
                change_setup = true;
            }
        case HIGH_ACCURACY:
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        // increase timing budget to 200 ms
                        gSensor[i].setMeasurementTimingBudget(200000);
                    }
                }
                change_setup = true;
            }
        default:
            break;
        }
        for(int i = 0;i < SENSOR_NUM;i++){
            Sensor_Value.distance[i] = 0;
            if(high_speed_mode == true){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    VL53L0X_distace[i] = gSensor[i].readRangeSingleMillimeters();
                    if(gSensor[i].timeoutOccurred()){
                        VL53L0X_distace[i] = -1;
                    }
                }
                high_speed_mode = false;
            } else if(high_speed_mode == false){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    for(int p = 0;p < 2;p++){
                        VL53L0X_distace[i] += gSensor[i].readRangeSingleMillimeters();
                        if(p == 1){
                            VL53L0X_distace[i] = VL53L0X_distace[i] / 2;
                        }   
                    }
                    if(gSensor[i].timeoutOccurred()){
                        VL53L0X_distace[i] = -1;
                    }
                } else if(VL53L0X_GPIO_ERROR[i] == true){
                    VL53L0X_distace[i] = -1;
                }
            }
        }
        if(change_setup == true){
            for(int i = 0;i < SENSOR_NUM;i++){
                if(VL53L0X_GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        gSensor[i].setSignalRateLimit(0.25);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                        gSensor[i].setMeasurementTimingBudget(33);
                        gSensor[i].setMeasurementTimingBudget(200);
                    }
                }
            }
            change_setup = false;
        }
        break;
    default:
        break;
    }    
}

int M5stack_contorl_system(int mode,int a){

}


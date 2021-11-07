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

#include "Wire.h"
#include "setup_mega.h"
#include "Arduino_due_port_register.h"
#include "VL53L0X.h"
#include <stdlib.h>

void Main_motor_control_system(double,double,double,long,int);
void MDM66126CH_control_system(int,int,int,int,long,int);
void MDM67H4504CH_control_system(int,int,int,int,long,int);
void LDM46165CH_control_system(int,int,int,int,long);
int Get_Button_State(int);
void Color_sensor_get_value(int);
void VL53L0X_get_distance(int,int);
void Lin_sensor_get_value(int,int);
void Lin_sensor_calibration(int,int);
int I2C_control_system(int,int);

//main system
typedef struct{
    const bool acceleration = true;
    const bool I2C_Active = false;
    int start_time = 0;     
    int time = 0;
    int error_code = 0;
    int mode = 0; 
    int bord_type = Arduino_Mega_2560;
} Main_system_n;
Main_system_n Main_system;
double point_A[3] = {0,3,90};

//motor controal system
#define Main_Motor_RPM 210
#define wide_a 172 / 2
#define wide_b 152 / 2
#define Rw 77
#define Pulse_interval 100
double main_speed = 50;

//MDM66126CH and MDM67H4504CH GPIO setup
typedef struct{
    const int GPIO_IN_1[6] = {0,0,0,0,0,0};
    const int GPIO_IN_2[6] = {0,0,0,0,0,0};
    const int GPIO_IN_PWM[6] = {0,0,0,0,0,0};
    const bool active = false;
} MDM66126CH_n;
MDM66126CH_n MDM66126CH;
typedef struct{
    const int GPIO_IN_1[4] = {3,8,11,5};
    const int GPIO_IN_2[4] = {2,7,12,6};
    const bool active = true;
} MDM67H4504CH_n;
MDM67H4504CH_n MDM67H4504CH;
//PTM750225CH and LDM46165CH GPIO setup
typedef struct{
    const uint8_t Target_value = 300;
    const uint8_t GPIO_IN_DIGITAL[13] = {53,51,49,47,45,43,41,39,37,35,33,31,29};
    const uint8_t GPIO_IN_ANALOG[12] = {15,14,13,12,11,10,8,8,7,6,5,4};
    const uint8_t GPIO_IN_Color[4] = {13,4,13,6};
    const int posision_mode_1[5][5] = {(1,2,1,2,1),(2,2,1,2,2),(1,1,2,1,1),(2,2,1,2,2),(2,2,1,2,2),(1,2,1,2,1)};
    const int posision_mode_2[5][5] = {(1,1,1,1,1),(1,2,2,2,1),(1,2,2,2,1),(1,2,2,2,1),(1,2,2,2,1),(1,1,1,1,1)};
    int Calibration_value[12];
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
    int Digital_reflect[5][5];
    int Analog_reflect[12];
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

//VL53L0X setup
#define SENSOR_NUM 8
#define ADDRESS_DEFALUT 0x29 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)
const int GPIO_MASK_ARRAY[SENSOR_NUM] = {38,40,42,44,46,48,50,52};
VL53L0X gSensor[SENSOR_NUM];
typedef struct{
    bool GPIO_ERROR[8];
    const bool SERIAL_OUTPUT = true;
} VL53L0X_console_n;
VL53L0X_console_n Vl53L0X_console;

void setup(){
    Wire.begin(17);  //I2C setup
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

    //random randomseed
    randomSeed(analogRead(0));

    Serial.println("12");
    
    //VL53L0X void setup
    /*for(int i = 0;i < SENSOR_NUM;i++){
        pinMode(GPIO_MASK_ARRAY[i], OUTPUT);
        digitalWrite(GPIO_MASK_ARRAY[i],LOW);
        delay(100);
    }
    Serial.println("12");
    for(int i = 0;i < SENSOR_NUM;i++){   // センサを初期化
        pinMode(GPIO_MASK_ARRAY[i],INPUT);
        if(gSensor[i].init()){
            gSensor[i].setTimeout(500);
            int address = ADDRESS_00 + (i * 2);
            gSensor[i].setAddress(address);
            Vl53L0X_console.GPIO_ERROR[i] = false;
            if(Vl53L0X_console.SERIAL_OUTPUT == true){
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" ok");
            } else {
                Vl53L0X_console.GPIO_ERROR[i] = false;
            }
        } else {
            Vl53L0X_console.GPIO_ERROR[i] = true;
            if(Vl53L0X_console.SERIAL_OUTPUT == true){
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" error");
            } else {
                Vl53L0X_console.GPIO_ERROR[i] = true;
            }
        }
        delay(100);
    }*/
}

void loop(){
    LDM46165CH_control_system(ON,OFF,OFF,OFF,1000);
    
    //digitalWrite(52,LOW);
    //digitalWrite(48,LOW);
    delay(1000);
}

void test_loop(){ 
    for(;;){
        //main system
        switch(Main_system.mode){
        case Line_tracking:
            int count = 0;
            Lin_sensor_get_value(DIGITAL,1);
            for(int x = 0;x < 5;x++){
                for(int y = 0;y < 5;y++){
                    if(PTM750225CH.posision_mode_2[x][y] == 1){
                        count++;
                    }
                }
            }
            switch(count){
            case Blank_Line:
                break;
            case Edge_Line:
                break;
            case Normal_Line:
                //least-square method
                //get Average_value
                double Average_value_x[3] = {0,0,0};
                double Average_value_y[3] = {0,0,0};
                Lin_sensor_get_value(DIGITAL,1);
                for(int x = 0;x < 5;x++){
                    for(int y = 0;y < 5;y++){
                        if(Sensor_Value.Digital_reflect[x][y] == 1){
                            Average_value_x[0] += (double)x - 2;
                            Average_value_x[1]++;
                            Average_value_y[0] += (double)y - 1; 
                            Average_value_y[1]++;
                        }
                    }
                }
                Average_value_x[2] = Average_value_x[0] / Average_value_x[1];
                Average_value_y[2] = Average_value_y[0] / Average_value_y[1]; 
                //get Deviation
                double *Deviation_x;
                double *Deviation_y;
                int Deviation_count = 0;
                Deviation_x = (double *)malloc(sizeof(double) * Average_value_x[1]);
                if(Deviation_x == NULL) exit(0);
                Deviation_y = (double *)malloc(sizeof(double) * Average_value_y[1]);
                if(Deviation_y == NULL) exit(0);
                for(int x = 0;x < 5;x++){
                    for(int y = 0;y < 5;y++){
                        if(Sensor_Value.Digital_reflect[x][y] == 1){
                            Deviation_x[Deviation_count] = (double)x - Average_value_x[2];
                            Deviation_y[Deviation_count] = (double)y - Average_value_y[2];
                            Deviation_count++;
                        }
                    }
                }
                //get Verianse
                double Variance_x;
                double Verianse_sum_x;
                for(int i = 0;i < Average_value_x[1];i++){
                    Verianse_sum_x += Deviation_x[i] * Deviation_x[i];
                }
                Variance_x = Verianse_sum_x / Average_value_x[1];
                //get Covariance
                double Covariance;
                double Covariance_sum;
                for(int i = 0;i < Average_value_x[1];i ++){
                    Covariance_sum += Deviation_x[i] * Deviation_y[i];
                }
                Covariance = Covariance_sum / Average_value_x[1];
                //get Inclination
                double Inclination;
                Inclination = Covariance / Variance_x;
                //get Segment
                double Segment;
                Segment = Average_value_y[2] - Inclination * Average_value_x[2];
                //memory relese
                free(Deviation_x);
                free(Deviation_y);
                //get intersecting point
                double spin_angle = 0;
                double a = Inclination,b = Segment;
                double point_B[3] = {0,0,0},point_B_1[3] = {0,0,0},point_B_2[3] = {0,0,0}; //point{x,y,rad}
                double point_C[3] = {0,0,NULL};
                point_B_1[0] = ((-1 * a * b) + sqrt(2 * pow(a * b,2.0) - pow(b,2.0) - pow(a,3.0) + a)) / (1 + pow(a,2.0));
                point_B_1[1] = a * point_B_1[0] + b;
                point_B_1[2] = tan(point_B_1[1] / point_B_1[0]);
                point_B_2[0] = ((-1 * a * b) - sqrt(2 * pow(a * b,2.0) - pow(b,2.0) - pow(a,3.0) + a)) / (1 + pow(a,2.0));
                point_B_2[1] = a * point_B_2[0] + b;
                point_B_2[2] = tan(point_B_2[1] / point_B_2[0]);
                if(abs(point_A[2] - point_B_1[2]) < 90 || abs(point_A[2] - point_B_1[2]) > 180){
                    if(abs(point_A[2] - point_B_1[2]) < 90){
                        memcpy(point_B,point_B_1,sizeof(point_B_1));
                        memcpy(point_A,point_B,sizeof(point_B));
                    } else {
                        if(360 - abs(point_A[2] - point_B_1[2]) < 90){
                            memcpy(point_B,point_B_1,sizeof(point_B_1));
           
                            memcpy(point_A,point_B,sizeof(point_B));
                        }
                    }
                }
                if(abs(point_A[2] - point_B_2[2]) < 90 || abs(point_A[2] - point_B_2[2]) > 180){
                    if(abs(point_A[2] - point_B_2[2]) < 90){
                        memcpy(point_B,point_B_2,sizeof(point_B_2));
                        memcpy(point_A,point_B,sizeof(point_B));
                    } else {
                        if(360 - abs(point_A[2] - point_B_2[2]) < 90){
                            memcpy(point_B,point_B_2,sizeof(point_B_2));
                            memcpy(point_A,point_B,sizeof(point_B));
                        }
                    }
                }
                point_C[0] = -1 * (b / a);

            case Crossroads_Line:
                break;
            default:
                break;
            }
            break;
        case Detect_Green_crossroads:
            break;
        case Detect_obstacle:
            break;
        case Victim_zone:
            break;
        case Termination:
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            LDM46165CH_control_system(ON,OFF,OFF,OFF,500);
            LDM46165CH_control_system(OFF,ON,OFF,OFF,500);
            LDM46165CH_control_system(OFF,OFF,ON,OFF,500);
            LDM46165CH_control_system(OFF,OFF,OFF,ON,500);
            LDM46165CH_control_system(OFF,OFF,ON,OFF,0);
            while(Get_Button_State(GREEN) == ON ){
                delay(200);
            }
            LDM46165CH_control_system(OFF,OFF,OFF,OFF,0);
            delay(10000);
            break;
        default:
            break;
        }
    }
}

int Get_Button_State(int num){
    int state;
    if(Main_system.acceleration == true){
        if(num == GREEN){
            if(PORTC & _BV(PC5)){
                return 1;
            } else {
                return 0;
            }
        }
        if(num == RED){
            if(PORTC & _BV(PC3)){
                return 1;
            } else {
                return 0;
            }
        }
    } else if(Main_system.acceleration == false){
        if(num == GREEN){
            state = digitalRead(32);
            if(state == 1){
                return 1;
            } else {
                return 0;
            }
        }
        if(num == RED){
            state = digitalRead(34);
            if(state == 1){
                return 1;
            } else {
                return 0;
            }
        }
    }
}

void Lin_sensor_get_value(int mode,int time){
    int count_a = 0,count_d = 0;
    int temporary_value_a[12];
    if(mode == ANALOG){
        LDM46165CH_control_system(OFF,OFF,OFF,ON,0);
        for(int i = 0;i < time;i++){
            for(int p = 0;p < 12;p++){
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                temporary_value_a[p] += analogRead(PTM750225CH.GPIO_IN_ANALOG[p]);
            }
        }
        for(int i = 0;i < 12;i++){
            Sensor_Value.Analog_reflect[i] = temporary_value_a[i] / mode + PTM750225CH.Calibration_value[i]; 
        }
        LDM46165CH_control_system(OFF,OFF,OFF,OFF,0);
    } else if(mode == DIGITAL){
        if(Main_system.acceleration == true){
            LDM46165CH_control_system(OFF,OFF,OFF,ON,0);
            for(int x = 0;x < 5;x++){
                for(int y = 0;y < 5;y++){
                    if(PTM750225CH.posision_mode_1[x][y] == 1){
                        count_a += PTM750225CH.posision_mode_1[x][y];
                        ADCSRA = ADCSRA & 0xf8;
                        ADCSRA = ADCSRA | 0x04;
                        if(ANALOG_THRESHOLD > analogRead(PTM750225CH.GPIO_IN_ANALOG[count_a])){
                            Sensor_Value.Digital_reflect[x][y] = 1;
                        } else {
                            Sensor_Value.Digital_reflect[x][y] = 0;
                        }
                    }
                }
            }
            if(PORTB & _BV(PB0)){
                Sensor_Value.Digital_reflect[0][1] = 1;    //I/O 53
            } else {
                Sensor_Value.Digital_reflect[0][1] = 0;    //I/O 53
            }
            if(PORTB & _BV(PB2)){
                Sensor_Value.Digital_reflect[0][3] = 1;    //I/O 51
            } else {
                Sensor_Value.Digital_reflect[0][3] = 0;    //I/O 51
            }
            if(PORTL & _BV(PL0)){
                Sensor_Value.Digital_reflect[1][0] = 1;    //I/O 49
            } else {
                Sensor_Value.Digital_reflect[1][0] = 0;    //I/O 49
            }
            if(PORTL & _BV(PL2)){
                Sensor_Value.Digital_reflect[1][1] = 1;    //I/O 47
            } else {
                Sensor_Value.Digital_reflect[1][1] = 0;    //I/O 47
            }
            if(PORTL & _BV(PL4)){
                Sensor_Value.Digital_reflect[1][3] = 1;    //I/O 45
            } else {
                Sensor_Value.Digital_reflect[1][3] = 0;    //I/O 45
            }
            if(PORTL & _BV(PL6)){
                Sensor_Value.Digital_reflect[1][4] = 1;    //I/O 43
            } else {
                Sensor_Value.Digital_reflect[1][4] = 0;    //I/O 43
            }
            if(PORTG & _BV(PG0)){
                Sensor_Value.Digital_reflect[2][2] = 1;    //I/O 41
            } else {
                Sensor_Value.Digital_reflect[2][2] = 0;    //I/O 41
            }
            if(PORTG & _BV(PG2)){
                Sensor_Value.Digital_reflect[3][0] = 1;    //I/O 33
            } else {
                Sensor_Value.Digital_reflect[3][0] = 0;    //I/O 33
            }
            if(PORTC & _BV(PC4)){
                Sensor_Value.Digital_reflect[3][1] = 1;    //I/O 35
            } else {
                Sensor_Value.Digital_reflect[3][1] = 0;    //I/O 35
            }
            if(PORTC & _BV(PC2)){
                Sensor_Value.Digital_reflect[3][3] = 1;    //I/O 37
            } else {
                Sensor_Value.Digital_reflect[3][3] = 0;    //I/O 37
            }
            if(PORTC & _BV(PC0)){
                Sensor_Value.Digital_reflect[3][4] = 1;    //I/O 39
            } else {
                Sensor_Value.Digital_reflect[3][4] = 0;    //I/O 39
            }
            if(PORTC & _BV(PC6)){
                Sensor_Value.Digital_reflect[4][1] = 1;    //I/O 31
            } else {
                Sensor_Value.Digital_reflect[4][1] = 0;    //I/O 31
            }
            if(PORTA & _BV(PA7)){
                Sensor_Value.Digital_reflect[4][3] = 1;    //I/O 29
            } else {
                Sensor_Value.Digital_reflect[4][3] = 0;    //I/O 29
            }
            LDM46165CH_control_system(OFF,OFF,OFF,OFF,0);
        } else if(Main_system.acceleration == false){
            LDM46165CH_control_system(OFF,OFF,OFF,ON,0);
            for(int x = 0;x < 5;x++){
                for(int y = 0;y < 5;y++){
                    if(PTM750225CH.posision_mode_1[x][y] == 1){
                        count_a += PTM750225CH.posision_mode_1[x][y]; 
                        if(ANALOG_THRESHOLD > analogRead(PTM750225CH.GPIO_IN_ANALOG[count_a])){
                            Sensor_Value.Digital_reflect[x][y] = 1;
                        } else {
                            Sensor_Value.Digital_reflect[x][y] = 0;
                        }
                    } else if(PTM750225CH.posision_mode_1[x][y] == 2){
                        count_d += PTM750225CH.posision_mode_1[x][y] / 2;
                        Sensor_Value.Digital_reflect[x][y] = digitalRead(PTM750225CH.GPIO_IN_DIGITAL[count_d]);
                    }
                }
            }
            LDM46165CH_control_system(OFF,OFF,OFF,OFF,0);
        }

    }
}

void Lin_sensor_calibration(int output,int num){
    
}

void Color_sensor_get_value(int continue_time){
    long red[4],green[4],blue[4];
    for(int p = 0;p < continue_time;p++){
        LDM46165CH_control_system(ON,OFF,OFF,OFF,1); //red
        for(int i = 0;i < 4;i++){
            if(continue_time == 1){
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                red[i] = analogRead(PTM750225CH.GPIO_IN_Color[i]); //get red reflect
            } else {
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                red[i] = (red[i] + analogRead(PTM750225CH.GPIO_IN_Color[i])) / 2; //get red reflect
            }
        }
        LDM46165CH_control_system(OFF,ON,OFF,OFF,1); //green
        for(int i = 0;i < 4;i++){
            if(continue_time == 1){
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                green[i] = analogRead(PTM750225CH.GPIO_IN_Color[i]); //get green reflect
            } else {
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                green[i] = (green[i] + analogRead(PTM750225CH.GPIO_IN_Color[i])) / 2; //get green reflect
            }
        }
        LDM46165CH_control_system(OFF,OFF,ON,OFF,1); //blue
        for(int i = 0;i < 4;i++){
            if(continue_time == 1){
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                blue[i] = analogRead(PTM750225CH.GPIO_IN_Color[i]); //get blue reflect
            } else {
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                blue[i] = (blue[i] + analogRead(PTM750225CH.GPIO_IN_Color[i])) / 2; //get blue reflect
            }
        }
        LDM46165CH_control_system(OFF,OFF,OFF,OFF,0);
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

void VL53L0X_get_distance(int mode,int VL53L0X_setting){
    int VL53L0X_distace[8];
    bool change_setup = true;
    bool high_speed_mode = false;
    switch(VL53L0X_setting){
    case DEFAULT:
        if(change_setup == true){
            //buck to default
            for(int i = 0;i < SENSOR_NUM;i++){
                if(Vl53L0X_console.GPIO_ERROR[i] == false){
                    if(gSensor[i].timeoutOccurred() == false){
                        gSensor[i].setSignalRateLimit(0.25);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                        gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                        gSensor[i].setMeasurementTimingBudget(33);
                    }
                }
            }
            change_setup = false;
        }
        break;
    case LONG_RANGE:
        for(int i = 0;i < SENSOR_NUM;i++){
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
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
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                if(gSensor[i].timeoutOccurred() == false){
                    // reduce timing budget to 20 ms (default is about 33 ms)
                    gSensor[i].setMeasurementTimingBudget(20000);
                }
            }
            change_setup = true;
        }
    case HIGH_ACCURACY:
        for(int i = 0;i < SENSOR_NUM;i++){
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
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
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                Sensor_Value.distance[i] = gSensor[i].readRangeSingleMillimeters();
                if(gSensor[i].timeoutOccurred()){
                    Vl53L0X_console.GPIO_ERROR[i] = true;
                }
            }
            high_speed_mode = false;
        } else if(high_speed_mode == false){
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                for(int p = 0;p < 2;p++){
                    Sensor_Value.distance[i] += gSensor[i].readRangeSingleMillimeters();
                    if(p == 1){
                        Sensor_Value.distance[i] = Sensor_Value.distance[i] / 2;
                    }   
                }
                if(gSensor[i].timeoutOccurred()){
                    Vl53L0X_console.GPIO_ERROR[i] = true;
                }
            }
        }
    }
    if(change_setup == true){
        //buck to default
        for(int i = 0;i < SENSOR_NUM;i++){
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                if(gSensor[i].timeoutOccurred() == false){
                    gSensor[i].setSignalRateLimit(0.25);
                    gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                    gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                    gSensor[i].setMeasurementTimingBudget(33);
                }
            }
        }
        change_setup = false;
    }
}

void Main_motor_control_system(double speed,double running_angle,double spin_angle,long time,int final_motion){
    double PHIw[4];
    double PHIwR[4],PHIwS[4],PHIwP[4],Vx,Vy;
    long pulse = 0;
    double pulse_angle = 0,running_angle_m = 0,running_angle_n = 0;
    if(spin_angle == 0){
        //Angle running only.
        running_angle = running_angle / 180 * M_PI;
        Vx = speed * cos(running_angle);
        Vy = speed * sin(running_angle);
        PHIw[0] = (1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        PHIw[1] = (1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        PHIw[2] = (1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        PHIw[3] = (1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        for(int i = 0;i < 4; i++){
            constrain(PHIw[i],-225,225);
        }
        if(MDM67H4504CH.active == true){
            MDM67H4504CH_control_system(PHIw[0],PHIw[1],PHIw[2],PHIw[3],time,final_motion);
        }
    } else if(running_angle == 0){
        //Spin turn only.
        Vx = 0;
        Vy = 0;
        PHIw[0] = ((1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        PHIw[1] = ((1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        PHIw[2] = ((1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        PHIw[3] = ((1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        for(int i = 0;i < 4; i++){
            constrain(PHIw[i],-225,225);
        }
        if(MDM67H4504CH.active == true){
            MDM67H4504CH_control_system(PHIw[0],PHIw[1],PHIw[2],PHIw[3],time,final_motion);
        }
    } else {
        //Angle running and spin turn.
        running_angle_m = running_angle / 180 * M_PI;
        Vx = speed * cos(running_angle_m);
        Vy = speed * sin(running_angle_m);
        PHIwR[0] = (1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        PHIwR[1] = (1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        PHIwR[2] = (1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        PHIwR[3] = (1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        Vx = 0;
        Vy = 0;
        PHIwS[0] = ((1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        PHIwS[1] = ((1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        PHIwS[2] = ((1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        PHIwS[3] = ((1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle) / Rw) / RR_Spead_195 / time;
        pulse = time / Pulse_interval;
        pulse_angle = spin_angle / pulse;
        for(int i = 0;i < 4;i++){
            PHIwR[i] = PHIwR[i] * 2;
            PHIwS[i] = PHIwS[i] * 2;
            constrain(PHIwR[i],-225,225);
            constrain(PHIwS[i],-255,255);
        }
        MDM67H4504CH_control_system(PHIwR[0],PHIwR[1],PHIwR[2],PHIwR[3],Pulse_interval,no_motion);
        MDM67H4504CH_control_system(PHIwS[0],PHIwS[1],PHIwS[2],PHIwS[3],Pulse_interval,no_motion);
        for(int i = 1;i < pulse - 1;i++){
            MDM67H4504CH_control_system(PHIwS[0],PHIwS[1],PHIwS[2],PHIwS[3],Pulse_interval,no_motion);
            running_angle_n = running_angle - pulse_angle * i;
            if(abs(running_angle_n) > 360){
                if(running_angle_n > 360) running_angle_n = (running_angle_n - 360) / 180 * M_PI;
                if(running_angle_n < -360) running_angle_n = (running_angle_n + 360) / 180 * M_PI;
            }
            Vx = speed * cos(running_angle_n);
            Vy = speed * sin(running_angle_n);
            PHIwP[0] = (1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
            PHIwP[1] = (1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
            PHIwP[2] = (1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
            PHIwP[3] = (1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
            for(int p = 0;p < 4;p++){
                constrain(PHIwP[p],-225,225);
            }
            MDM67H4504CH_control_system(PHIwP[0],PHIwP[1],PHIwP[2],PHIwP[3],Pulse_interval,no_motion);
        }
        MDM67H4504CH_control_system(PHIwR[0],PHIwR[1],PHIwR[2],PHIwR[3],Pulse_interval,stop);
        running_angle_n = running_angle - pulse_angle * pulse;
        if(abs(running_angle_n) > 360){
            if(running_angle_n > 360) running_angle_n = (running_angle_n - 360) / 180 * M_PI;
            if(running_angle_n < -360) running_angle_n = (running_angle_n + 360) / 180 * M_PI;
        }
        Vx = speed * cos(running_angle_n);
        Vy = speed * sin(running_angle_n);
        PHIwP[0] = (1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        PHIwP[1] = (1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        PHIwP[2] = (1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        PHIwP[3] = (1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle * 0);
        for(int i = 0;i < 4;i++){
            constrain(PHIwP[i],-225,225);
        }
        MDM67H4504CH_control_system(PHIwP[0],PHIwP[1],PHIwP[2],PHIwP[3],Pulse_interval,final_motion);
    }
}

void MDM67H4504CH_control_system(int a,int b,int c,int d,long time,int final_motion){
    int num[4] = {a,b,c,d};
    for(int i = 0;i < 4;i++){
        if(num[i] != 0){
            if(num[i] > 0){
                constrain(num[i],0,100);
                analogWrite(MDM67H4504CH.GPIO_IN_1[i],num[i]);
                analogWrite(MDM67H4504CH.GPIO_IN_2[i],0);
            } else if(num[i] < 0){
                num[i] = num[i] * -1;
                constrain(num[i],0,100);
                analogWrite(MDM67H4504CH.GPIO_IN_1[i],0);
                analogWrite(MDM67H4504CH.GPIO_IN_2[i],num[i]);
            }
        } else if(num[i] == 0){
            analogWrite(MDM67H4504CH.GPIO_IN_1[i],0);
            analogWrite(MDM67H4504CH.GPIO_IN_2[i],0);
        }
    }
    if(time != 0) delay(time);
    if(final_motion != 0){
        for(int i = 0;i < 4;i++){
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

void LDM46165CH_control_system(int r,int g,int b,int w,long time){
    int rgb[3] = {r,g,b};
    if(r != -1){
        if(Main_system.acceleration == true){
            if(Main_system.bord_type == Arduino_Mega_2560){
                if(w == 1){
                    PORTA |= _BV(PA6);  
                    PORTC |= _BV(PC7);  
                } else {
                    PORTA &= ~_BV(PA6); 
                    PORTC &= ~_BV(PC7); 
                }
                if(r == 1){
                    PORTA |= _BV(PA4);  
                } else {
                    PORTA &= ~_BV(PA4); 
                }
                if(g == 1){
                    PORTA |= _BV(PA2);
                } else {
                    PORTA &= ~_BV(PA2);
                }
                if(b == 1){
                    PORTA |= _BV(PA0);
                } else {
                    PORTA &= ~_BV(PA0);
                }
            } else if(Main_system.bord_type == Arduino_Due){
                if(w == 1){
                    PORTD |= _BV(PD3);  
                    PORTD |= _BV(PD9);  
                } else {
                    PORTD &= ~_BV(PD3); 
                    PORTD &= ~_BV(PD9); 
                }
                if(r == 1){
                    PORTD |= _BV(PD1);  
                } else {
                    PORTD &= ~_BV(PD1); 
                }
                if(g == 1){
                    PORTA |= _BV(PA15);
                } else {
                    PORTA &= ~_BV(PA15);
                }
                if(b == 1){
                    PORTB |= _BV(PB26);
                } else {
                    PORTB &= ~_BV(PB26);
                }
            }
        } else if(Main_system.acceleration == false){
            if(w == 1){
                digitalWrite(LDM46165CH.GPIO_IN_WHITE[0],HIGH);
                digitalWrite(LDM46165CH.GPIO_IN_WHITE[1],HIGH);
            } else if(w == 0){
                digitalWrite(LDM46165CH.GPIO_IN_WHITE[0],LOW);
                digitalWrite(LDM46165CH.GPIO_IN_WHITE[1],LOW);
            }
            for(int i = 0;i < 3;i++){
                if(rgb[i] == 1){
                    digitalWrite(LDM46165CH.GPIO_IN_RGB[i],HIGH);
                } else if(rgb[i] == 0){
                    digitalWrite(LDM46165CH.GPIO_IN_RGB[i],LOW);
                }
            }
        }
        if(time != 0) delay(time);
    } else {
        //Back code(random)
        if(Main_system.bord_type == Arduino_Mega_2560){
            switch(random(4)){
            case 0:
                PORTA |= _BV(PA4);
                delay(time);
                PORTA &= ~_BV(PA4); 
            case 1:
                PORTA |= _BV(PA2);
                delay(time);
                PORTA &= ~_BV(PA2);
            case 2:
                PORTA |= _BV(PA0);
                delay(time);
                PORTA &= ~_BV(PA0);
            case 3:
                PORTA |= _BV(PA6);  
                PORTC |= _BV(PC7);
                delay(time);
                PORTA &= ~_BV(PA6); 
                PORTC &= ~_BV(PC7); 
            }
        } else if(Main_system.bord_type == Arduino_Due){
            switch(random(4)){
            case 0:
                PORTD |= _BV(PD1);
                delay(time);
                PORTD &= ~_BV(PD1); 
            case 1:
                PORTA |= _BV(PA15);
                delay(time);
                PORTA &= ~_BV(PA15);
            case 2:
                PORTB |= _BV(PB26);
                delay(time);
                PORTB &= ~_BV(PB26);
            case 3:
                PORTD |= _BV(PD3);  
                PORTD |= _BV(PD9);
                delay(time);
                PORTD &= ~_BV(PD3); 
                PORTD &= ~_BV(PD9); 
            }
        }
    }
}

int I2C_control_system(int target,int num){
    switch(target){
    case Arduino_nano:
        //port 18
        break;
    case M5stack_core2:
        //port 19
        break;
    case BSCS:
        //port 20
        break;
    }
}


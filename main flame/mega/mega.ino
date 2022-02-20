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
#include "Arduino.h"
#include "Wire.h"
#include "setup_mega.h"
#include "VL53L0X.h"
#include <stdlib.h>

void loop_control_system(int);
void Main_motor_control_system(double,double,double,long,int);
void MDM66126CH_control_system(int,int,int,int,long,int);
void MDM67H4504CH_control_system(int,int,int,int,long,int);
void LDM46165CH_control_system(int,int,int,int,long);
int I2C_control_system(int,int);
int Get_Button_State(int);
void VL53L0X_get_distance(int);
void Color_sensor_get_value(int);
void Lin_sensor_get_value(int,int);
void Line_Sensor_serial_print();
void Line_Sensor_get_threshold();

//main system
typedef struct{
    const bool acceleration = true;
    const bool I2C_Active = false;
    const bool Serial_line_active = true;
    int start_time = 0;     
    int time = 0;
    int error_code = 0;
    int mode = 0; 
} Main_system_n;
Main_system_n Main_system;

//motor controal system
#define Main_Motor_RPM 300
#define wide_a 172 / 2
#define wide_b 152 / 2
#define Rw 30
#define Pulse_interval 100
long mcs_time = 0;
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
    const int GPIO_IN_1[4] = {6,8,11,2};
    const int GPIO_IN_2[4] = {5,7,12,3};
    const bool active = true;
} MDM67H4504CH_n;
MDM67H4504CH_n MDM67H4504CH;
//PTM750225CH and LDM46165CH GPIO setup
typedef struct{
    const int Target_value = 300;
    const int GPIO_IN_DIGITAL[13] = {53,51,49,47,45,43,41,39,37,35,33,31,29};
    const int GPIO_IN_ANALOG[12] = {11,10,9,8,7,6,5,4,3,2,1,0};
    const int GPIO_IN_Color[4] = {11,9,2,0};
    const int posision_mode_1[5][5] = {{1,2,1,2,1},{2,2,1,2,2},{1,1,2,1,1},{2,2,1,2,2},{1,2,1,2,1}};
    const int posision_mode_2[5][5] = {{1,1,1,1,1},{1,2,2,2,1},{1,2,2,2,1},{1,2,2,2,1},{1,1,1,1,1}};
    int threshold_value[12] = {0,0,0,0,0,0,0,0,0,0,0};
} PTM750225CH_n;
PTM750225CH_n PTM750225CH;
typedef struct{
    const int GPIO_IN_WHITE[2] = {28,30};
    const int GPIO_IN_RGB[3] = {26,24,22};
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
VL53L0X tof_sensor;
typedef struct{
    bool GPIO_ERROR[8];
    const bool SERIAL_OUTPUT = true;
} VL53L0X_console_n;
VL53L0X_console_n Vl53L0X_console;

void setup(){
    Wire.begin(17);  //I2C setup
    Serial.begin(9600); //serial setup

    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);

    //moter,ft sensor,led pinmode setup
    if(MDM66126CH.active == false && MDM67H4504CH.active == true){
        for(int i = 0;i < 6;i++){
            pinMode(MDM66126CH.GPIO_IN_1[i],OUTPUT);
            pinMode(MDM66126CH.GPIO_IN_2[i],OUTPUT);
        }   
    }
    for(int i = 0;i < 13;i++) pinMode(PTM750225CH.GPIO_IN_DIGITAL[i],INPUT);    
    for(int i = 0;i < 2;i++) pinMode(LDM46165CH.GPIO_IN_WHITE[i],OUTPUT);
    for(int i = 0;i < 3;i++) pinMode(LDM46165CH.GPIO_IN_RGB[i],OUTPUT);

    //button setup
    pinMode(34,INPUT);
    pinMode(32,INPUT);

    //random randomseed
    //randomSeed(analogRead(0));
    
    //VL53L0X void setup
    /*for(int i = 0;i < SENSOR_NUM;i++){
        pinMode(GPIO_MASK_ARRAY[i],OUTPUT);
        digitalWrite(GPIO_MASK_ARRAY[i],HIGH);
        delay(100);
        tof_sensor.setTimeout(500);
        if(!tof_sensor.init()){
            if(Vl53L0X_console.SERIAL_OUTPUT == true){
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" error");
            } else {
                Vl53L0X_console.GPIO_ERROR[i] = true;
            }
        } else {
            if(Vl53L0X_console.SERIAL_OUTPUT == true){
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.println(" ok");
            } else {
                Vl53L0X_console.GPIO_ERROR[i] = false;
            }
        }
        digitalWrite(GPIO_MASK_ARRAY[i],LOW);
    }*/
    //LDM46165CH_control_system(OFF,OFF,ON,OFF,1000);
    LDM46165CH_control_system(ON,OFF,OFF,OFF,0);
    //Line_Sensor_get_threshold();
}

void loop(){
    /*
    loop_control_system(start);
    Lin_sensor_get_value(DIGITAL,1);
    Line_Sensor_serial_print();
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        LDM46165CH_control_system(OFF,OFF,OFF,ON,0);
        int all_white = 0;
        for (int x = 0; x < 5; x++) {
            for (int y = 0; y < 5; y++) {
                all_white += Sensor_Value.Digital_reflect[x][y];
            }
        }
        if (all_white != 0) {
            double Average_value_x[3] = { 0,0,0 };
            double Average_value_y[3] = { 0,0,0 };
            for (int x = 0; x < 5; x++) {
                for (int y = 0; y < 5; y++) {
                    if (Sensor_Value.Digital_reflect[x][y] == 1) {
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
            double* Deviation_x;
            double* Deviation_y;
            int Deviation_count = 0;
            Deviation_x = (double*)malloc(sizeof(double) * Average_value_x[1]);
            if (Deviation_x == NULL) exit(0);
            Deviation_y = (double*)malloc(sizeof(double) * Average_value_y[1]);
            if (Deviation_y == NULL) exit(0);
            for (int x = 0; x < 5; x++) {
                for (int y = 0; y < 5; y++) {
                    if (Sensor_Value.Digital_reflect[x][y] == 1) {
                        Deviation_x[Deviation_count] = ((double)x - 2) - Average_value_x[2];
                        Deviation_y[Deviation_count] = ((double)y - 1) - Average_value_y[2];
                        Deviation_count++;
                    }
                }
            }
            //get Verianse
            double Variance_x = 0;
            double Verianse_sum_x = 0;
            for (int i = 0; i < Average_value_x[1]; i++) {
                Verianse_sum_x += Deviation_x[i] * Deviation_x[i];
            }
            Variance_x = Verianse_sum_x / Average_value_x[1];
            //get Covariance
            double Covariance = 0;
            double Covariance_sum = 0;
            for (int i = 0; i < Average_value_x[1]; i++) {
                Covariance_sum += Deviation_x[i] * Deviation_y[i];
            }
            Covariance = Covariance_sum / Average_value_x[1];
            //memory relese
            free(Deviation_x);
            free(Deviation_y);
            if (Covariance != 0 && Variance_x != 0) {
                //get Inclination
                double Inclination = 0;
                Inclination = Covariance / (Variance_x * Variance_x);
                //get Segment
                double Segment = 0;
                Segment = Average_value_y[2] - Inclination * Average_value_x[2];
                //Calculate parameters
                double a = Inclination, b = Segment;
                double Point_A_a[4] = { 0,0,0,0 }, Point_A_b[4] = { 0,0,0,0 }, Point_A[4] = { 0,0,0,0 }; //{x,y,Inclination,angle}
                //Get the coordinates of the intersection
                Point_A_a[0] = ((a * b * -1) + sqrt(9 * pow(a, 2.0) - pow(b, 2.0) + 9)) / (1 + pow(a, 2.0));
                Point_A_a[1] = Inclination * Point_A_a[0] + Segment;
                Point_A_b[0] = ((a * b * -1) - sqrt(9 * pow(a, 2.0) - pow(b, 2.0) + 9)) / (1 + pow(a, 2.0));
                Point_A_b[1] = Inclination * Point_A_b[0] + Segment;
                //Get the the inclination
                Point_A_a[2] = Point_A_a[1] / Point_A_a[0];
                Point_A_b[2] = Point_A_b[1] / Point_A_b[0];
                //Get the angle
                Point_A_a[3] = atan2(Point_A_a[1], Point_A_a[0]) * 180 / M_PI;
                Point_A_b[3] = atan2(Point_A_b[1], Point_A_b[0]) * 180 / M_PI;
                //Selection of intersections
                if (Point_A_a[1] > 0) {
                    memcpy(Point_A, Point_A_a, sizeof(Point_A_a) * 4);
                }
                else if (Point_A_b[1] > 0) {
                    memcpy(Point_A, Point_A_b, sizeof(Point_A_b) * 4);
                }
                printf("%lf\n", Point_A[3]);
                //Determine parameters
                double running_distance = 0, running_angle = 0, spin_angle = 0, running_time = 0;
                running_distance = sqrt(pow(Point_A[0], 2.0) + pow(Point_A[1], 2.0)) * 10;
                running_time = running_distance / (60 * M_PI / 360) / RR_Spead_195 / main_speed;
                double num;
                num = -1 * Segment / Inclination;
                if (atan2(Point_A[1], Point_A[0] - num) * 180 / M_PI > 0) spin_angle = atan2(Point_A[1], Point_A[0] - num) * 180 / M_PI - 90;
                if (atan2(Point_A[1], Point_A[0] - num) * 180 / M_PI < 0) spin_angle = atan2(Point_A[1], Point_A[0] - num) * 180 / M_PI + 90;
                if (atan2(Point_A[1], Point_A[0] - num) * 180 / M_PI == 90) spin_angle = 0;
                if (Point_A[3] > 0) running_angle = Point_A[3] - 90;
                if (Point_A[3] < 0) running_angle = Point_A[3] + 90;
                if (Point_A[3] == 90) running_angle = 0;
                if(Main_system.Serial_line_active == true){
                    Line_Sensor_serial_print();
                    Serial.print("segment : ");
                    Serial.println(Segment);
                    Serial.print("Inclination : ");
                    Serial.println(Inclination);
                    Serial.print("running_angle : ");
                    Serial.println(running_angle);
                    Serial.print("spin_angle : ");
                    Serial.println(spin_angle);
                }
                Main_motor_control_system(main_speed,running_angle,0,running_time,stop);
                Main_motor_control_system(main_speed,0,spin_angle,running_time,stop);
            } else {
                double Point_AA[4] = { 0,0,0,0 };
                Point_AA[0] = Average_value_x[2];
                Point_AA[1] = sqrt(9 - pow((float)Point_AA[0], 2.0));
                Point_AA[2] = Point_AA[1] / Point_AA[0];
                Point_AA[3] = atan2(Point_AA[1], Point_AA[0]) * 180 / M_PI;
                double x_running_distance = 0, x_running_angle = 0, x_running_time = 0;
                x_running_distance = sqrt(pow((float)Point_AA[0], 2.0) + pow((float)Point_AA[1], 2.0)) * 10;
                x_running_time = x_running_distance / (60 * M_PI / 360) / RR_Spead_195 / main_speed;
                if (Point_AA[3] > 0) x_running_angle = (Point_AA[3] - 90);
                if (Point_AA[3] < 0) x_running_angle = (Point_AA[3] + 90);
                if (Point_AA[3] == 90) x_running_angle = 0;
                if(Main_system.Serial_line_active == true){
                    Line_Sensor_serial_print();
                    Serial.print("running_angle : ");
                    Serial.println(x_running_angle);
                    Serial.println(x_running_time);
                }
                Main_motor_control_system(main_speed,x_running_angle,0,x_running_time,stop);
            }
        }
        else if (all_white == 0) {
            double running_time = 0, running_distance = 0;
            running_distance = sqrt(pow(3, 2.0) + pow(0, 2.0)) * 10;
            running_time = running_distance / (60 * M_PI / 360) / RR_Spead_195 / main_speed;
            Main_motor_control_system(main_speed,0,0,running_time,stop);
        }
        loop_control_system(stop);
    }*/
    delay(10000);
}

void loop_control_system(int num){
    if(num == start){
        for(;;){
            if(Get_Button_State(GREEN) == 1) break;
            if(Get_Button_State(GREEN) == 0) delay(10);
        }
    } else if(num == stop){
        if(Get_Button_State(RED) == 1){
            for(;;){
                if(Get_Button_State(GREEN) == 1) break;
                if(Get_Button_State(GREEN) == 0) delay(10);     
            }
        }
    }
}

int Get_Button_State(int num){
    int state;
    if(Main_system.acceleration == true){
        if(num == RED){
            if(PINC & _BV(PC5)){
                return 0;
            } else {
                return 1;
            }
        }
        if(num == GREEN){
            if(PINC & _BV(PC3)){
                return 0;
            } else {
                return 1;
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
    int temporary_value[12] = {0,0,0,0,0,0,0,0,0,0,0};
    int num[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
    if(time < 1) time = 1;
    if(mode == ANALOG){
        for(int i = 0;i < time;i++){
            for(int p = 0;p < 12;p++){
                ADCSRA = ADCSRA & 0xf8;
                ADCSRA = ADCSRA | 0x04;
                temporary_value[p] += analogRead(PTM750225CH.GPIO_IN_ANALOG[p]);
            }
        }
        for(int i = 0;i < 12;i++){
            Sensor_Value.Analog_reflect[i] = temporary_value[i] / time + PTM750225CH.threshold_value[i]; 
        }
    } else if(mode == DIGITAL){
        if(Main_system.acceleration == true){
            for(int x = 0;x < 5;x++){
                for(int y = 0;y < 5;y++){
                    if(PTM750225CH.posision_mode_1[x][y] == 1){
                        count_a += PTM750225CH.posision_mode_1[x][y];
                        ADCSRA = ADCSRA & 0xf8;
                        ADCSRA = ADCSRA | 0x04;
                        if(ANALOG_THRESHOLD > analogRead(PTM750225CH.GPIO_IN_ANALOG[count_a - 1])){
                            Sensor_Value.Digital_reflect[x][y] = 1;
                        } else {
                            Sensor_Value.Digital_reflect[x][y] = 0;
                        }         
                    }
                }
            }
            Sensor_Value.Digital_reflect[4][2] = 0;
            num[0] = PINB & _BV(PB0);
            if(num[0] == 0){
                Sensor_Value.Digital_reflect[0][1] = 1;    //I/O 53
            } else {
                Sensor_Value.Digital_reflect[0][1] = 0;    //I/O 53
            }
            num[1] = PINB & _BV(PB2);
            if(num[1] == 0){
                Sensor_Value.Digital_reflect[0][3] = 1;    //I/O 51
            } else {
                Sensor_Value.Digital_reflect[0][3] = 0;    //I/O 51
            }
            num[2] = PINL & _BV(PL0);
            if(num[2] == 0){
                Sensor_Value.Digital_reflect[1][0] = 1;    //I/O 49
            } else {
                Sensor_Value.Digital_reflect[1][0] = 0;    //I/O 49
            }
            num[3] = PINL & _BV(PL2);
            if(num[3] == 0){
                Sensor_Value.Digital_reflect[1][1] = 1;    //I/O 47
            } else {
                Sensor_Value.Digital_reflect[1][1] = 0;    //I/O 47
            }
            num[4] = PINL & _BV(PL4);
            if(num[4] == 0){
                Sensor_Value.Digital_reflect[1][3] = 1;    //I/O 45
            } else {
                Sensor_Value.Digital_reflect[1][3] = 0;    //I/O 45
            }
            num[5] = PINL & _BV(PL6);
            if(num[5] == 0){
                Sensor_Value.Digital_reflect[1][4] = 1;    //I/O 43
            } else {
                Sensor_Value.Digital_reflect[1][4] = 0;    //I/O 43
            }
            num[6] = PING & _BV(PG0);
            if(num[6] == 0){
                Sensor_Value.Digital_reflect[2][2] = 1;    //I/O 41
            } else {
                Sensor_Value.Digital_reflect[2][2] = 0;    //I/O 41
            }
            num[7] = PING & _BV(PG2);
            if(num[7] == 0){
                Sensor_Value.Digital_reflect[3][0] = 1;    //I/O 33
            } else {
                Sensor_Value.Digital_reflect[3][0] = 0;    //I/O 33
            }
            num[8] = PINC & _BV(PC0);
            if(num[8] == 0){
                Sensor_Value.Digital_reflect[3][1] = 1;    //I/O 35
            } else {
                Sensor_Value.Digital_reflect[3][1] = 0;    //I/O 35
            }
            num[9] = PINC & _BV(PC2);
            if(num[9] == 0){
                Sensor_Value.Digital_reflect[3][3] = 1;    //I/O 37
            } else {
                Sensor_Value.Digital_reflect[3][3] = 0;    //I/O 37
            }
            num[10] = PINC & _BV(PC4);
            if(num[10] == 0){
                Sensor_Value.Digital_reflect[3][4] = 1;    //I/O 39
            } else {
                Sensor_Value.Digital_reflect[3][4] = 0;    //I/O 39
            }
            num[11] = PINC & _BV(PC6);
            if(num[11] == 0){
                Sensor_Value.Digital_reflect[4][1] = 1;    //I/O 31
            } else {
                Sensor_Value.Digital_reflect[4][1] = 0;    //I/O 31
            }
            num[12] = PINA & _BV(PA7);
            if(num[12] == 0){
                Sensor_Value.Digital_reflect[4][3] = 1;    //I/O 29
            } else {
                Sensor_Value.Digital_reflect[4][3] = 0;    //I/O 29
            }
        } else if(Main_system.acceleration == false){
            for(int x = 0;x < 5;x++){
                for(int y = 0;y < 5;y++){
                    if(PTM750225CH.posision_mode_1[x][y] == 1){
                        count_a += PTM750225CH.posision_mode_1[x][y] - 1; 
                        if(!(x == 4 || y == 2)){
                            if(10 > analogRead(PTM750225CH.GPIO_IN_ANALOG[count_a])){
                                Sensor_Value.Digital_reflect[x][y] = 1;
                            } else {
                                Sensor_Value.Digital_reflect[x][y] = 0;
                            }
                        } else {
                            if(ANALOG_THRESHOLD > analogRead(PTM750225CH.GPIO_IN_ANALOG[count_a])){
                                Sensor_Value.Digital_reflect[x][y] = 1;
                            } else {
                                Sensor_Value.Digital_reflect[x][y] = 0;
                            }
                        }
                    } else if(PTM750225CH.posision_mode_1[x][y] == 2){
                        count_d += PTM750225CH.posision_mode_1[x][y] / 2 - 1;
                        Sensor_Value.Digital_reflect[x][y] = digitalRead(PTM750225CH.GPIO_IN_DIGITAL[count_d]) * 1;
                    }
                }
            }
        }

    }
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
                Sensor_Value.color[i] = (60 * (green[i] - blue[i]) / 1) % 360;
            }
        } else if(green[i] > red[i] && green[i] > blue[i]){
            if(red[i] > blue[i]){
                Sensor_Value.color[i] = (60 * (blue[i] - red[i]) / (green[i] - blue[i])) + 120;
            } else if(blue[i] > red[i]){
                Sensor_Value.color[i] = (60 * (blue[i] - red[i]) / (green[i] - red[i])) + 120;
            } else if(blue[i] == red[i]){
                Sensor_Value.color[i] = (60 * (blue[i] - red[i]) / 1) + 120;
            }
        } else if(blue[i] > red[i] && blue[i] > green[i]){
            if(red[i] > green[i]){
                Sensor_Value.color[i] = (60 * (red[i] - green[i]) / (blue[i] - green[i])) + 240;
            } else if(green[i] > red[i]){
                Sensor_Value.color[i] = (60 * (red[i] - green[i]) / (blue[i] - red[i])) + 240; 
            } else if(green[i] == red[i]){
                Sensor_Value.color[i] = (60 * (red[i] - green[i]) / 1) + 240;
            }
        } else if(red[i] == green[i] && green[i] == blue[i] && blue[i] == red[i]){
            Sensor_Value.color[i] = 0;
        }
    }
}

void VL53L0X_get_distance(int VL53L0X_setting){
    int VL53L0X_distace[8];
    bool change_setup = true;
    bool high_speed_mode = false;
    for(int i = 0;i < SENSOR_NUM;i++){
        switch(VL53L0X_setting){
        case DEFAULT_mode:
            if(change_setup == true){
                //buck to default
                if(Vl53L0X_console.GPIO_ERROR[i] == false){
                    if(tof_sensor.timeoutOccurred() == false){
                        tof_sensor.setSignalRateLimit(0.25);
                        tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                        tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                        tof_sensor.setMeasurementTimingBudget(33);
                    }
                }
                change_setup = false;
            }
            break;
        case LONG_RANGE:
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                if(tof_sensor.timeoutOccurred() == false){
                    // lower the return signal rate limit (default is 0.25 MCPS)
                    tof_sensor.setSignalRateLimit(0.1);
                    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
                    tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,18);
                    tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,14);
                }
            }
            change_setup = true;
            break;
        case HIGH_SPEED:
            high_speed_mode = true;
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                if(tof_sensor.timeoutOccurred() == false){
                    // reduce timing budget to 20 ms (default is about 33 ms)
                    tof_sensor.setMeasurementTimingBudget(20000);
                }
            }
            change_setup = true;
            break;
        case HIGH_ACCURACY:
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                if(tof_sensor.timeoutOccurred() == false){
                    // increase timing budget to 200 ms
                    tof_sensor.setMeasurementTimingBudget(200000);
                }
            }
            change_setup = true;
            break;
        default:
            break;
        }
        Sensor_Value.distance[i] = 0;
        if(high_speed_mode == true){
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                Sensor_Value.distance[i] = tof_sensor.readRangeSingleMillimeters();
                if(tof_sensor.timeoutOccurred()){
                    Vl53L0X_console.GPIO_ERROR[i] = true;
                }
            }
            high_speed_mode = false;
        } else if(high_speed_mode == false){
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                for(int p = 0;p < 2;p++){
                    Sensor_Value.distance[i] += tof_sensor.readRangeSingleMillimeters();
                    if(p == 1){
                        Sensor_Value.distance[i] = Sensor_Value.distance[i] / 2;
                    }   
                }
                if(tof_sensor.timeoutOccurred()){
                    Vl53L0X_console.GPIO_ERROR[i] = true;
                }
            }
        }
        if(change_setup == true){
            //buck to default
            if(Vl53L0X_console.GPIO_ERROR[i] == false){
                if(tof_sensor.timeoutOccurred() == false){
                    tof_sensor.setSignalRateLimit(0.25);
                    tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,14);
                    tof_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,10);
                    tof_sensor.setMeasurementTimingBudget(33);
                }
            }
            change_setup = false;
        }
    }
}

void Main_motor_control_system(double speed,double running_angle,double spin_angle,long time,int final_motion){
    double PHIw[4];
    double PHIwR[4],PHIwS[4],PHIwP[4],Vx,Vy;
    long pulse = 0;
    double pulse_angle = 0,running_angle_m = 0,running_angle_n = 0;
    if(spin_angle == 0){
        //Angle running only.
        running_angle = running_angle to_rad;
        Vx = speed * cos(running_angle);
        Vy = speed * sin(running_angle);
        PHIw[0] = (1 * Vx + -1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        PHIw[1] = (1 * Vx + 1 * Vy + -1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        PHIw[2] = (1 * Vx + -1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        PHIw[3] = (1 * Vx + 1 * Vy + 1 * (wide_a / 2 + wide_b / 2) * spin_angle);
        for(int i = 0;i < 4; i++){
            PHIw[i] = constrain(PHIw[i],-255,255);
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
            PHIw[i] = constrain(PHIw[i],-255,255);
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
            PHIwR[i] = constrain(PHIwR[i],-255,255);
            PHIwS[i] = constrain(PHIwS[i],-255,255);
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
                PHIwP[p] = constrain(PHIwP[p],-255,255);
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
            PHIwP[i] = constrain(PHIwP[i],-255,255);
        }
        MDM67H4504CH_control_system(PHIwP[0],PHIwP[1],PHIwP[2],PHIwP[3],Pulse_interval,final_motion);
    }
    mcs_time = time;
}

void MDM67H4504CH_control_system(int a,int b,int c,int d,long time,int final_motion){
    int num[4] = {a,b,c,d};
    for(int i = 0;i < 4;i++){
        if(num[i] != 0){
            if(num[i] > 0){
                num[i] = constrain(num[i],0,255);
                analogWrite(MDM67H4504CH.GPIO_IN_1[i],num[i]);
                analogWrite(MDM67H4504CH.GPIO_IN_2[i],0);
            } else if(num[i] < 0){
                num[i] = num[i] * -1;
                num[i] = constrain(num[i],0,255);
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
                num[i] = constrain(num[i],0,255);
                digitalWrite(MDM66126CH.GPIO_IN_1[i],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[i],LOW);
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],num[i]);
            } else if(num[1] < 0){
                num[i] = num[i] * -1;
                num[i] = constrain(num[i],0,255);
                digitalWrite(MDM66126CH.GPIO_IN_1[i],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[i],HIGH);
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],num[i]);
            }
            if(time != 0) delay(time);
        }
    }
    if(final_motion != 0){
        for(int i = 0;i < 5;i++){
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
        switch(random(3)){
        case 0:
            PORTA |= _BV(PA4);
            delay(time);
            PORTA &= ~_BV(PA4);
            break;
        case 1:
            PORTA |= _BV(PA2);
            delay(time);
            PORTA &= ~_BV(PA2);
            break;
        case 2:
            PORTA |= _BV(PA0);
            delay(time);
            PORTA &= ~_BV(PA0);
            break;
        case 3:
            PORTA |= _BV(PA6);  
            PORTC |= _BV(PC7);
            delay(time);
            PORTA &= ~_BV(PA6); 
            PORTC &= ~_BV(PC7);
            break;
        }
    }
}

int I2C_control_system(int target,int num){
    int data;
    switch(target){
    case Arduino_nano:
        //port 18
        Wire.requestFrom(Arduino_nano,6);
        while(Wire.available()){
            data =  Wire.read();
        } 
        break;
    case M5stack_core2:
        //port 19
        break;
    case BSCS:
        //port 20
        break;
    }
    return data;
}

void Line_Sensor_serial_print(){
    //Lin_sensor_get_value(DIGITAL,1);
    Serial.println("Digital ");
    for(int i = 4;i >= 0;i--){
        for(int p = 0;p < 5;p++){
            if(p == 4){
                Serial.println(Sensor_Value.Digital_reflect[p][i]);
            } else {
                Serial.print(Sensor_Value.Digital_reflect[p][i]);
                Serial.print("  ");
            }
        }
        Serial.println(" ");
    }
    Lin_sensor_get_value(ANALOG,1);
    Serial.print("Analog ");
    for(int i = 0;i < 11;i++){
        Serial.print(Sensor_Value.Analog_reflect[i]);
        Serial.print(" ");
    }
    Serial.println(Sensor_Value.Analog_reflect[11]);
    delay(100);
}

void Line_Sensor_get_threshold(){
    Main_motor_control_system(50,90,0,2000,stop);
    Main_motor_control_system(50,270,0,0,no_motion);
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        if(Sensor_Value.Digital_reflect[4][1] == 1 && Sensor_Value.Digital_reflect[4][3] == 1){
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            Lin_sensor_get_value(ANALOG,5);
            PTM750225CH.threshold_value[11] = Sensor_Value.Analog_reflect[11] + 5;
            PTM750225CH.threshold_value[10] = Sensor_Value.Analog_reflect[10] + 5;
            PTM750225CH.threshold_value[9] = Sensor_Value.Analog_reflect[9] + 5;     
            break;
        }      
    }
    Main_motor_control_system(40,-90,0,0,no_motion);
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        if(Sensor_Value.Digital_reflect[3][0] == 1 && Sensor_Value.Digital_reflect[3][4] == 1){
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            Lin_sensor_get_value(ANALOG,5);
            PTM750225CH.threshold_value[8] = Sensor_Value.Analog_reflect[8] + 5;
            break;
        }      
    }
    Main_motor_control_system(40,-90,0,0,no_motion);
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        if(Sensor_Value.Digital_reflect[2][2] == 1){
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            Lin_sensor_get_value(ANALOG,5);
            PTM750225CH.threshold_value[7] = Sensor_Value.Analog_reflect[7] + 5;
            PTM750225CH.threshold_value[6] = Sensor_Value.Analog_reflect[6] + 5;
            PTM750225CH.threshold_value[5] = Sensor_Value.Analog_reflect[5] + 5;
            PTM750225CH.threshold_value[4] = Sensor_Value.Analog_reflect[4] + 5;
            break;
        }      
    }
    Main_motor_control_system(40,-90,0,0,no_motion);
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        if(Sensor_Value.Digital_reflect[1][0] == 1 && Sensor_Value.Digital_reflect[1][4] == 1){
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            Lin_sensor_get_value(ANALOG,5);
            PTM750225CH.threshold_value[3] = Sensor_Value.Analog_reflect[3] + 5;
            break;
        }      
    }
    Main_motor_control_system(40,-90,0,0,no_motion);
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        if(Sensor_Value.Digital_reflect[0][1] == 1 && Sensor_Value.Digital_reflect[0][3] == 1){
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            Lin_sensor_get_value(ANALOG,5);
            PTM750225CH.threshold_value[2] = Sensor_Value.Analog_reflect[2] + 5;
            PTM750225CH.threshold_value[1] = Sensor_Value.Analog_reflect[1] + 5;
            PTM750225CH.threshold_value[0] = Sensor_Value.Analog_reflect[0] + 5;     
            break; 
        }     
    }
    Main_motor_control_system(40,90,0,0,no_motion);
    for(;;){
        Lin_sensor_get_value(DIGITAL,1);
        if(Sensor_Value.Digital_reflect[2][0] == 1 && Sensor_Value.Digital_reflect[2][4] == 1){
            MDM67H4504CH_control_system(0,0,0,0,0,stop);
            break;
        }
    }
}


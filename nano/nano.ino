#include "setup_nano.h"
#include "Wire.h"
#include "arduino.h"

void MDM66126CH_control_system(int,int,int,int,int,long,int);
void I2C_control_system_Receive(int);
void I2C_control_system_Request();

#define RPM_motor_x 120
#define RPM_motor_y 0
#define RPM_motor_z 0

//MDM66126CH and MDM67H4504CH GPIO setup
typedef struct{
    const int GPIO_IN_1[6] = {4,8,13,17,0,0};
    const int GPIO_IN_2[6] = {2,7,12,16,0,0};
    const int GPIO_IN_PWM[6] = {3,5,6,9,10,0};
    const bool active = false;
} MDM66126CH_n;
MDM66126CH_n MDM66126CH;

typedef struct{
    int mode = 0;
    int angle = 0;
    int speed = 0;
    bool active = false;
} I2C_Receive_data_n;
I2C_Receive_data_n I2C_Receive_data;

void setup()
{
    Serial.begin(9600);
    Wire.begin(8);
    
    for(int i = 0;i < 6;i++){
        if(i != 4){
            pinMode(MDM66126CH.GPIO_IN_1[i],OUTPUT);
            pinMode(MDM66126CH.GPIO_IN_2[i],OUTPUT);
        }
        pinMode(MDM66126CH.GPIO_IN_PWM[i],OUTPUT);
    }
    
    Wire.onReceive(I2C_control_system_Receive);
    Wire.onRequest(I2C_control_system_Request);
}

void loop()
{
    int time;
    if(I2C_Receive_data.active == true){
        switch(I2C_Receive_data.mode){
            case 1:
                time = I2C_Receive_data.angle * (RPM_motor_x * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(I2C_Receive_data.speed,0,0,0,0,time,stop);
                break;
            case 2:
                time = I2C_Receive_data.angle * (RPM_motor_x * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(0,I2C_Receive_data.speed,0,0,0,time,stop);
                break;
            case 3:
                time = I2C_Receive_data.angle * (RPM_motor_x * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(I2C_Receive_data.speed,I2C_Receive_data.speed,0,0,0,time,stop);
                break;
            case 4:
                time = I2C_Receive_data.angle * (RPM_motor_y * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(0,0,I2C_Receive_data.speed,0,0,time,stop);
                break;
            case 5:
                time = I2C_Receive_data.angle * (RPM_motor_y * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(0,0,0,I2C_Receive_data.speed,0,time,stop);
                break;
            case 6:
                time = I2C_Receive_data.angle * (RPM_motor_y * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(0,0,I2C_Receive_data.speed,I2C_Receive_data.speed,0,time,stop);
                break;
            case 7:
                time = I2C_Receive_data.angle * (RPM_motor_z * 360 / 60000) / 255 * I2C_Receive_data.speed;
                MDM66126CH_control_system(0,0,0,0,I2C_Receive_data.speed,time,stop);
                break;
            default:
                break;
        }
        delay(100);
    }
    delay(100);
}

void I2C_control_system_Receive(int a){     //int a は関数内では使わない。
    if(I2C_Receive_data.mode == 0){
        while(Wire.available()){   
            I2C_Receive_data.mode = Wire.read();   // 1バイトを受信
        }
    } else if(I2C_Receive_data.angle == 0){
        while(Wire.available()){
            I2C_Receive_data.angle = Wire.read();
        }
    } else if(I2C_Receive_data.speed == 0){
        while(Wire.available()){
            I2C_Receive_data.speed = Wire.read();
        }
    }
    if(I2C_Receive_data.speed != 0) I2C_Receive_data.active = true;
}

void I2C_control_system_Request(){
    int time;
    switch(I2C_Receive_data.mode){
        case 1:
            time = I2C_Receive_data.angle * (RPM_motor_x * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        case 2:
            time = I2C_Receive_data.angle * (RPM_motor_x * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        case 3:
            time = I2C_Receive_data.angle * (RPM_motor_x * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        case 4:
            time = I2C_Receive_data.angle * (RPM_motor_y * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        case 5:
            time = I2C_Receive_data.angle * (RPM_motor_y * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        case 6:
            time = I2C_Receive_data.angle * (RPM_motor_y * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        case 7:
            time = I2C_Receive_data.angle * (RPM_motor_z * 360 / 60000) / 255 * I2C_Receive_data.speed;
            break;
        default:
            break;
    }
    time += 200;
    Wire.beginTransmission(7);
    Wire.write(time);
    Wire.endTransmission();
}

void MDM66126CH_control_system(int x_a,int x_b,int y_a,int y_b,int z,long time,int final_motion){
    int num[5] = {x_a,x_b,y_a,y_b,z};
    for(int i = 0;i < 5;i++){
        if(num[i] != 0){
            if(num[i] > 0){
                constrain(num[i],0,100);
                if(i == 4){
                    analogWrite(MDM66126CH.GPIO_IN_1[i],255);
                    analogWrite(MDM66126CH.GPIO_IN_2[i],0);
                } else {
                    digitalWrite(MDM66126CH.GPIO_IN_1[i],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[i],LOW);
                }
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],map(num[i],0,100,0,255));
            } else if(num[i] < 0){
                num[i] = num[i] * -1;
                constrain(num[i],0,100);
                if(i == 4){
                    analogWrite(MDM66126CH.GPIO_IN_1[i],0);
                    analogWrite(MDM66126CH.GPIO_IN_2[i],255);
                } else {
                    digitalWrite(MDM66126CH.GPIO_IN_1[i],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[i],HIGH);
                }
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],map(num[i],0,100,0,255));
            }
            if(time != 0) delay(time);
        }
    }
    if(final_motion != 0){
        for(int i = 0;i < 6;i++){
            if(num[1] == 0){
                if(final_motion == 1){
                    if(i == 4){
                        analogWrite(MDM66126CH.GPIO_IN_1[i],0);
                        analogWrite(MDM66126CH.GPIO_IN_2[i],0);
                    } else {
                        digitalWrite(MDM66126CH.GPIO_IN_1[i],LOW);
                        digitalWrite(MDM66126CH.GPIO_IN_2[i],LOW);
                    }
                    analogWrite(MDM66126CH.GPIO_IN_PWM[i],255);
                } else if(final_motion == 2){
                    if(i == 4){
                        analogWrite(MDM66126CH.GPIO_IN_1[i],255);
                        analogWrite(MDM66126CH.GPIO_IN_2[i],255);
                    } else {
                        digitalWrite(MDM66126CH.GPIO_IN_1[i],HIGH);
                        digitalWrite(MDM66126CH.GPIO_IN_2[i],HIGH);
                    }
                    analogWrite(MDM66126CH.GPIO_IN_PWM[i],0);
                }
            }
        }   
    }
}



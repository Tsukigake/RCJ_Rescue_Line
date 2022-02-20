#include "setup_nano.h"
#include "Wire.h"

void collect_module_control_system(int);
void collect_module_motor_control_system(int,int,int,int,int);
void MDM66126CH_control_system(int,int,int,int,long,int);

//MDM66126CH and MDM67H4504CH GPIO setup
typedef struct{
    const int GPIO_IN_1[4] = {8,16,4,13};
    const int GPIO_IN_2[4] = {7,17,2,12};
    const int GPIO_IN_PWM[4] = {5,9,3,6};
} MDM66126CH_n;
MDM66126CH_n MDM66126CH;

void setup(){
    Serial.begin(9600);
    pinMode(8,OUTPUT);
    pinMode(16,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(17,OUTPUT);
    pinMode(2,OUTPUT);
    pinMode(13,OUTPUT);
}

int poi = 1;

void loop(){

    delay(10000);
}


void collect_module_control_system(int mode){
    switch(mode){
    case system_comand_structure_mode_1:
        collect_module_motor_control_system(0,-20,0,100,brake);
        delay(100);
        collect_module_motor_control_system(70,0,0,70,brake);
        delay(100);
        collect_module_motor_control_system(0,-170,0,60,brake);
        delay(100);
        collect_module_motor_control_system(100,0,0,60,brake);
        delay(100);
        break;
    case system_comand_structure_mode_2:
        collect_module_motor_control_system(-80,0,0,60,brake);
        delay(100);
        collect_module_motor_control_system(0,170,0,70,brake);
        delay(100);
        collect_module_motor_control_system(-70,0,0,60,brake);
        delay(100);
        collect_module_motor_control_system(0,20,0,100,brake);
        delay(100);
        break;
    default:
        break;
    }

}

void collect_module_motor_control_system(int x,int y,int z,int speed,int final_motion){
    for(int i = 0;i < 3;i++){
        switch(i){
        case 1:
            if(x > 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[0],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[0],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_1[1],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[1],LOW);
            } else if(x < 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[0],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[0],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_1[1],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[1],HIGH);
            }
            analogWrite(MDM66126CH.GPIO_IN_PWM[0],speed);
            analogWrite(MDM66126CH.GPIO_IN_PWM[1],speed);
            if(x > 0){
                delay(x / RR_Spead_100 / speed);
            } else if(x < 0){
                delay(x * -1 / RR_Spead_100 / speed);
            }
            if(final_motion != no_motion){
                if(final_motion == stop){
                    digitalWrite(MDM66126CH.GPIO_IN_1[0],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[0],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_1[1],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[1],LOW);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[0],255);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[1],255);
                } else if(final_motion == brake){
                    digitalWrite(MDM66126CH.GPIO_IN_1[0],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[0],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_1[1],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[1],HIGH);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[0],0);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[1],0);
                }
            }
            break;
        case 2:
            if(y > 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[2],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[2],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_1[3],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[3],LOW);
            } else if(y < 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[2],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[2],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_1[3],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[3],HIGH);
            }
            analogWrite(MDM66126CH.GPIO_IN_PWM[2],speed);
            analogWrite(MDM66126CH.GPIO_IN_PWM[3],speed);
            if(y > 0){
                delay(y / RR_Spead_180 / speed);
            } else if(y < 0){
                delay(y * -1 / RR_Spead_180 / speed);
            }
            if(final_motion != no_motion){
                if(final_motion == stop){
                    digitalWrite(MDM66126CH.GPIO_IN_1[2],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[2],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_1[3],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[3],LOW);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[2],255);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[3],255);
                } else if(final_motion == brake){
                    digitalWrite(MDM66126CH.GPIO_IN_1[2],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[2],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_1[3],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[3],HIGH);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[2],0);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[3],0);
                }
            }
            break;
        case 3:
            if(z > 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[4],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[4],LOW);
            } else if(z < 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[4],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[4],HIGH);
            }
            analogWrite(MDM66126CH.GPIO_IN_PWM[2],z);
            if(z > 0){
                delay(z / RR_Spead_180 / speed);
            } else if(z < 0){
                delay((z / RR_Spead_180 / speed) * -1);
            }
            if(final_motion != no_motion){
                if(final_motion == stop){
                    digitalWrite(MDM66126CH.GPIO_IN_1[4],LOW);
                    digitalWrite(MDM66126CH.GPIO_IN_2[4],LOW);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[4],255);
                } else if(final_motion == brake){
                    digitalWrite(MDM66126CH.GPIO_IN_1[4],HIGH);
                    digitalWrite(MDM66126CH.GPIO_IN_2[4],HIGH);
                    analogWrite(MDM66126CH.GPIO_IN_PWM[4],0);
                }
            }
            break;
        default:
            break;
        }
    }
}

void MDM66126CH_control_system(int x_a,int x_b,int y_a,int y_b,long time,int final_motion){
    int num[4] = {x_a,x_b,y_a,y_b};
    for(int i = 0;i < 4;i++){
        if(num[i] != 0){
            if(num[i] > 0){
                digitalWrite(MDM66126CH.GPIO_IN_1[i],HIGH);
                digitalWrite(MDM66126CH.GPIO_IN_2[i],LOW);
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],num[i]);
            } else if(num[i] < 0){
                num[i] = num[i] * -1;
                digitalWrite(MDM66126CH.GPIO_IN_1[i],LOW);
                digitalWrite(MDM66126CH.GPIO_IN_2[i],HIGH);
                analogWrite(MDM66126CH.GPIO_IN_PWM[i],num[i]);
            }
        }
    }
    if(time != 0) delay(time);
    if(final_motion != 0){
        for(int i = 0;i < 4;i++){
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

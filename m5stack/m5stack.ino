#define M5STACK_MPU6886 
#include <M5Stack.h>

int line_sensor_GUI_first();
int line_sensor_GUI_second();
int line_sensor_GUI_third();
int line_sensor_GUI_fourth();

int IMU_sensor_gyro_mode_GUI();
int IMU_sensor_acc_mode_GUI();
int IMU_sensor_gmt_mode_GUI();

int IMU_sensor_gyro_graph_GUI_gyroX();
int IMU_sensor_gyro_graph_GUI_gyroY();
int IMU_sensor_gyro_graph_GUI_gyroZ();

int IMU_sensor_acc_graph_GUI_accX();
int IMU_sensor_acc_graph_GUI_accY();
int IMU_sensor_acc_graph_GUI_accZ();

int reset = 0;

void setup(){
    M5.begin();
    M5.Power.begin();
    M5.IMU.Init();
    M5.Lcd.fillScreen(BLACK);
}

void loop() {
    int num_x = 1;

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,8);
        M5.Lcd.print("M5stack GUI system");
        M5.Lcd.drawFastHLine(5,29,310,BLUE);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(10,38);
        M5.Lcd.print("1.IMU sensor gyro data");
        M5.Lcd.drawFastHLine(5,59,310,BLUE);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(10,68);
        M5.Lcd.print("2.IMU sensor acc data");
        M5.Lcd.drawFastHLine(5,89,310,BLUE);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(10,98);
        M5.Lcd.print("3.IMU sensor gmt data");
        M5.Lcd.drawFastHLine(5,119,310,BLUE);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(10,128);
        M5.Lcd.print("4.line sensor data");
        M5.Lcd.drawFastHLine(5,149,310,BLUE);

        if(M5.BtnA.read() == 1) num_x -= 1;
        if(M5.BtnC.read() == 1) num_x += 1;
        if(num_x < 1) num_x = 4;
        if(num_x > 4) num_x = 1;

        if(num_x == 1){
            M5.Lcd.fillRect(290,32,25,25,GREEN);
        } else {
            M5.Lcd.fillRect(290,32,25,25,BLACK);
        } 
        if(num_x == 2){
            M5.Lcd.fillRect(290,62,25,25,GREEN);
        } else {
            M5.Lcd.fillRect(290,62,25,25,BLACK);
        }
        if(num_x == 3){
            M5.Lcd.fillRect(290,92,25,25,GREEN);
        } else {
            M5.Lcd.fillRect(290,92,25,25,BLACK);
        }
        if(num_x == 4){
            M5.Lcd.fillRect(290,122,25,25,GREEN);
        } else {
            M5.Lcd.fillRect(290,122,25,25,BLACK);
        }

        if(M5.BtnB.read() == 1){
            int i = 1;
            switch (num_x){
            case 1:
                for(;;){
                    M5.Lcd.fillScreen(BLACK);
                    switch (i){
                    case 1:
                        delay(200);
                        i = i + IMU_sensor_gyro_mode_GUI();
                        delay(200);
                        break;
                    case 2:
                        delay(200);
                        i = i + IMU_sensor_gyro_graph_GUI_gyroX();
                        delay(200);
                        break;
                    case 3:
                        delay(200);
                        i = i + IMU_sensor_gyro_graph_GUI_gyroY();
                        delay(200);
                        break;
                    case 4:
                        delay(200);
                        i = i + IMU_sensor_gyro_graph_GUI_gyroZ();
                        delay(200);
                        break;
                    default:
                        break;
                    }
                    if(i < 1) i = 4;
                    if(i > 4) i = 1;
                    if(reset == 1){
                        reset = 0;
                        break;
                    }
                }
                break;
            case 2:
                for(;;){
                    M5.Lcd.fillScreen(BLACK);
                    switch (i){
                    case 1:
                        delay(200);
                        i = i + IMU_sensor_acc_mode_GUI();
                        delay(200);
                        break;
                    case 2:
                        delay(200);
                        i = i + IMU_sensor_acc_graph_GUI_accX();
                        delay(200);
                        break;
                    case 3:
                        delay(200);
                        i = i + IMU_sensor_acc_graph_GUI_accY();
                        delay(200);
                        break;
                    case 4:
                        delay(200);
                        i = i + IMU_sensor_acc_graph_GUI_accZ();
                        delay(200);
                        break;
                    default:
                        break;
                    }
                    if(i < 1) i = 4;
                    if(i > 4) i = 1;
                    if(reset == 1){
                        reset = 0;
                        break;
                    }
                }
                break;
            case 4:
                for(;;){
                    M5.Lcd.fillScreen(BLACK);
                    switch (i){
                    case 1:
                        delay(200);
                        i = i + line_sensor_GUI_first();;
                        delay(200);
                        break;
                    case 2:
                        delay(200);
                        i = i + line_sensor_GUI_second();
                        delay(200);
                        break;
                    case 3:
                        delay(200);
                        i = i + line_sensor_GUI_third();
                        delay(200);
                        break;
                    case 4:
                        delay(200);
                        i = i + line_sensor_GUI_fourth();
                        delay(200);
                        break;
                    default:
                        break;
                    }
                    if(i < 1) i = 4;
                    if(i > 4) i = 1;
                    if(reset == 1){
                        reset = 0;
                        break;
                    }
                }
                break;
            default:
                break;
            }
            M5.Lcd.fillScreen(BLACK);
        }
        delay(100);
    }
}

int line_sensor_GUI_first(){
    int line_sensor_digital[13];
    int line_sensor_pwm[12];
    float temp = 0.0F;

    for(;;){
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("Line sensor data 1");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.Lcd.drawFastHLine(10,35,196,BLUE);
        M5.Lcd.drawFastHLine(10,74,196,BLUE);
        M5.Lcd.drawFastHLine(10,113,196,BLUE);
        M5.Lcd.drawFastHLine(10,152,196,BLUE);
        M5.Lcd.drawFastHLine(10,191,196,BLUE);
        M5.Lcd.drawFastHLine(10,230,196,BLUE);
        M5.Lcd.drawFastVLine(10,35,196,BLUE);
        M5.Lcd.drawFastVLine(49,35,196,BLUE);
        M5.Lcd.drawFastVLine(88,35,196,BLUE);
        M5.Lcd.drawFastVLine(127,35,196,BLUE);
        M5.Lcd.drawFastVLine(166,35,196,BLUE);
        M5.Lcd.drawFastVLine(205,35,196,BLUE);

        for(int i = 0;i < 13;i++){
            line_sensor_digital[i] = random(0,99);
        }
        for(int i = 0;i < 12;i++){
            line_sensor_pwm[i] = random(0,99);
        }

        M5.Lcd.setTextColor(BLUE);

        //first
        if(line_sensor_pwm[0] > 10){
            M5.Lcd.fillRect(14,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,39,32,32,ORANGE);
        }
        if(line_sensor_digital[0] > 11){
            M5.Lcd.fillRect(53,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,39,32,32,GREEN);
        }
        if(line_sensor_pwm[1] > 10){
            M5.Lcd.fillRect(92,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,39,32,32,ORANGE);
        }
        if(line_sensor_digital[1] > 11){
            M5.Lcd.fillRect(131,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,39,32,32,GREEN);
        }
        if(line_sensor_pwm[2] > 10){
            M5.Lcd.fillRect(170,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,39,32,32,ORANGE);
        }

        //second
        if(line_sensor_digital[2] > 11){
            M5.Lcd.fillRect(14,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,78,32,32,GREEN);
        }
        if(line_sensor_digital[3] > 11){
            M5.Lcd.fillRect(53,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,78,32,32,GREEN);
        }
        if(line_sensor_pwm[3] > 10){
            M5.Lcd.fillRect(92,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,78,32,32,ORANGE);
        }
        if(line_sensor_digital[4] > 11){
            M5.Lcd.fillRect(131,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,78,32,32,GREEN);
        }
        if(line_sensor_digital[5] > 11){
            M5.Lcd.fillRect(170,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,78,32,32,GREEN);
        }

        //third
        if(line_sensor_pwm[4] > 10){
            M5.Lcd.fillRect(14,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,117,32,32,ORANGE);
        }
        if(line_sensor_pwm[5] > 10){
            M5.Lcd.fillRect(53,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,117,32,32,ORANGE);
        }
        if(line_sensor_digital[6] > 11){
            M5.Lcd.fillRect(92,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,117,32,32,GREEN);
        }
        if(line_sensor_pwm[6] > 10){
            M5.Lcd.fillRect(131,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,117,32,32,ORANGE);
        }
        if(line_sensor_pwm[7] > 10){
            M5.Lcd.fillRect(170,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,117,32,32,ORANGE);
        }

        //fourth
        if(line_sensor_digital[7] > 11){
            M5.Lcd.fillRect(14,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,156,32,32,GREEN);
        }
        if(line_sensor_digital[8] > 11){
            M5.Lcd.fillRect(53,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,156,32,32,GREEN);
        }
        if(line_sensor_pwm[8] > 10){
            M5.Lcd.fillRect(92,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,156,32,32,ORANGE);
        }
        if(line_sensor_digital[9] > 11){
            M5.Lcd.fillRect(131,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,156,32,32,GREEN);
        }
        if(line_sensor_digital[10] > 11){
            M5.Lcd.fillRect(170,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,156,32,32,GREEN);
        }

        //fifth
        if(line_sensor_pwm[9] > 10){
            M5.Lcd.fillRect(14,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,195,32,32,ORANGE);
        }
        if(line_sensor_digital[11] > 11){
            M5.Lcd.fillRect(53,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,195,32,32,GREEN);
        }
        if(line_sensor_pwm[10] > 10){
            M5.Lcd.fillRect(92,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,195,32,32,ORANGE);
        }
        if(line_sensor_digital[12] > 11){
            M5.Lcd.fillRect(131,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,195,32,32,GREEN);
        }
        if(line_sensor_pwm[11] > 10){
            M5.Lcd.fillRect(170,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,195,32,32,ORANGE);
        }

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,35);
        M5.Lcd.print("sensor A");
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,55);
        M5.Lcd.print("A : --");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,74);
        M5.Lcd.print("sensor B");
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,94);
        M5.Lcd.print("B : --");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,113);
        M5.Lcd.print("sensor C");
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,133);
        M5.Lcd.print("C : --");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,152);
        M5.Lcd.print("sensor D");
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,172);
        M5.Lcd.print("D : --");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.IMU.getTempData(&temp);
        M5.lcd.setCursor(215,191);
        M5.Lcd.print("Temp");
        M5.lcd.setCursor(215,211);
        M5.Lcd.printf("%.2f C",temp);

        delay(100);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int line_sensor_GUI_second(){
    int line_sensor_digital[13];
    int line_sensor_pwm[12];
    float temp = 0.0F;

    for(;;){
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("Line sensor data 2");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.Lcd.drawFastHLine(10,35,196,BLUE);
        M5.Lcd.drawFastHLine(10,74,196,BLUE);
        M5.Lcd.drawFastHLine(10,113,196,BLUE);
        M5.Lcd.drawFastHLine(10,152,196,BLUE);
        M5.Lcd.drawFastHLine(10,191,196,BLUE);
        M5.Lcd.drawFastHLine(10,230,196,BLUE);
        M5.Lcd.drawFastVLine(10,35,196,BLUE);
        M5.Lcd.drawFastVLine(49,35,196,BLUE);
        M5.Lcd.drawFastVLine(88,35,196,BLUE);
        M5.Lcd.drawFastVLine(127,35,196,BLUE);
        M5.Lcd.drawFastVLine(166,35,196,BLUE);
        M5.Lcd.drawFastVLine(205,35,196,BLUE);

        for(int i = 0;i < 13;i++){
            line_sensor_digital[i] = random(0,99);
        }
        for(int i = 0;i < 12;i++){
            line_sensor_pwm[i] = random(0,99);
        }

        M5.Lcd.setTextColor(BLUE);

        //first
        if(line_sensor_pwm[0] > 10){
            M5.Lcd.fillRect(14,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,39,32,32,ORANGE);
        }
        M5.lcd.setCursor(18,43);
        M5.Lcd.print("A");
        if(line_sensor_digital[0] > 11){
            M5.Lcd.fillRect(53,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,39,32,32,GREEN);
        }
        if(line_sensor_pwm[1] > 10){
            M5.Lcd.fillRect(92,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,39,32,32,ORANGE);
        }
        if(line_sensor_digital[1] > 11){
            M5.Lcd.fillRect(131,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,39,32,32,GREEN);
        }
        if(line_sensor_pwm[2] > 10){
            M5.Lcd.fillRect(170,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,39,32,32,ORANGE);
        }
        M5.lcd.setCursor(174,43);
        M5.Lcd.print("B");

        //second
        if(line_sensor_digital[2] > 11){
            M5.Lcd.fillRect(14,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,78,32,32,GREEN);
        }
        if(line_sensor_digital[3] > 11){
            M5.Lcd.fillRect(53,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,78,32,32,GREEN);
        }
        if(line_sensor_pwm[3] > 10){
            M5.Lcd.fillRect(92,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,78,32,32,ORANGE);
        }
        if(line_sensor_digital[4] > 11){
            M5.Lcd.fillRect(131,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,78,32,32,GREEN);
        }
        if(line_sensor_digital[5] > 11){
            M5.Lcd.fillRect(170,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,78,32,32,GREEN);
        }

        //third
        if(line_sensor_pwm[4] > 10){
            M5.Lcd.fillRect(14,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,117,32,32,ORANGE);
        }
        if(line_sensor_pwm[5] > 10){
            M5.Lcd.fillRect(53,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,117,32,32,ORANGE);
        }
        if(line_sensor_digital[6] > 11){
            M5.Lcd.fillRect(92,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,117,32,32,GREEN);
        }
        if(line_sensor_pwm[6] > 10){
            M5.Lcd.fillRect(131,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,117,32,32,ORANGE);
        }
        if(line_sensor_pwm[7] > 10){
            M5.Lcd.fillRect(170,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,117,32,32,ORANGE);
        }

        //fourth
        if(line_sensor_digital[7] > 11){
            M5.Lcd.fillRect(14,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,156,32,32,GREEN);
        }
        if(line_sensor_digital[8] > 11){
            M5.Lcd.fillRect(53,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,156,32,32,GREEN);
        }
        if(line_sensor_pwm[8] > 10){
            M5.Lcd.fillRect(92,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,156,32,32,ORANGE);
        }
        if(line_sensor_digital[9] > 11){
            M5.Lcd.fillRect(131,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,156,32,32,GREEN);
        }
        if(line_sensor_digital[10] > 11){
            M5.Lcd.fillRect(170,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,156,32,32,GREEN);
        }

        //fifth
        if(line_sensor_pwm[9] > 10){
            M5.Lcd.fillRect(14,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,195,32,32,ORANGE);
        }
        M5.lcd.setCursor(18,199);
        M5.Lcd.print("D");
        if(line_sensor_digital[11] > 11){
            M5.Lcd.fillRect(53,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,195,32,32,GREEN);
        }
        if(line_sensor_pwm[10] > 10){
            M5.Lcd.fillRect(92,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,195,32,32,ORANGE);
        }
        if(line_sensor_digital[12] > 11){
            M5.Lcd.fillRect(131,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,195,32,32,GREEN);
        }
        if(line_sensor_pwm[11] > 10){
            M5.Lcd.fillRect(170,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,195,32,32,ORANGE);
        }
        M5.lcd.setCursor(174,199);
        M5.Lcd.print("C");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,35);
        M5.Lcd.print("sensor A");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(215,55);
        M5.Lcd.printf("A : %d",line_sensor_pwm[0]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,74);
        M5.Lcd.print("sensor B");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(215,94);
        M5.Lcd.printf("B : %d",line_sensor_pwm[2]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,113);
        M5.Lcd.print("sensor C");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(215,133);
        M5.Lcd.printf("C : %d",line_sensor_pwm[11]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,152);
        M5.Lcd.print("sensor D");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(215,172);
        M5.Lcd.printf("D : %d",line_sensor_pwm[9]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.IMU.getTempData(&temp);
        M5.lcd.setCursor(215,191);
        M5.Lcd.print("Temp");
        M5.lcd.setCursor(215,211);
        M5.Lcd.printf("%.2f C",temp);

        delay(100);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int line_sensor_GUI_third(){
    int line_sensor_digital[13];
    int line_sensor_pwm[12];
    float temp = 0.0F;

    for(;;){
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("Line sensor data 3");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.Lcd.drawFastHLine(10,35,196,BLUE);
        M5.Lcd.drawFastHLine(10,74,196,BLUE);
        M5.Lcd.drawFastHLine(10,113,196,BLUE);
        M5.Lcd.drawFastHLine(10,152,196,BLUE);
        M5.Lcd.drawFastHLine(10,191,196,BLUE);
        M5.Lcd.drawFastHLine(10,230,196,BLUE);
        M5.Lcd.drawFastVLine(10,35,196,BLUE);
        M5.Lcd.drawFastVLine(49,35,196,BLUE);
        M5.Lcd.drawFastVLine(88,35,196,BLUE);
        M5.Lcd.drawFastVLine(127,35,196,BLUE);
        M5.Lcd.drawFastVLine(166,35,196,BLUE);
        M5.Lcd.drawFastVLine(205,35,196,BLUE);

        for(int i = 0;i < 13;i++){
            line_sensor_digital[i] = random(0,99);
        }
        for(int i = 0;i < 12;i++){
            line_sensor_pwm[i] = random(0,99);
        }

        M5.Lcd.setTextColor(BLUE);

        //first
        if(line_sensor_pwm[0] > 10){
            M5.Lcd.fillRect(14,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,39,32,32,ORANGE);
        }
        if(line_sensor_digital[0] > 11){
            M5.Lcd.fillRect(53,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,39,32,32,GREEN);
        }
        if(line_sensor_pwm[1] > 10){
            M5.Lcd.fillRect(92,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,39,32,32,ORANGE);
        }
        M5.lcd.setCursor(96,43);
        M5.Lcd.print("A");
        if(line_sensor_digital[1] > 11){
            M5.Lcd.fillRect(131,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,39,32,32,GREEN);
        }
        if(line_sensor_pwm[2] > 10){
            M5.Lcd.fillRect(170,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,39,32,32,ORANGE);
        }

        //second
        if(line_sensor_digital[2] > 11){
            M5.Lcd.fillRect(14,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,78,32,32,GREEN);
        }
        if(line_sensor_digital[3] > 11){
            M5.Lcd.fillRect(53,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,78,32,32,GREEN);
        }
        if(line_sensor_pwm[3] > 10){
            M5.Lcd.fillRect(92,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,78,32,32,ORANGE);
        }
        if(line_sensor_digital[4] > 11){
            M5.Lcd.fillRect(131,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,78,32,32,GREEN);
        }
        if(line_sensor_digital[5] > 11){
            M5.Lcd.fillRect(170,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,78,32,32,GREEN);
        }

        //third
        if(line_sensor_pwm[4] > 10){
            M5.Lcd.fillRect(14,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,117,32,32,ORANGE);
        }
        M5.lcd.setCursor(18,121);
        M5.Lcd.print("D");
        if(line_sensor_pwm[5] > 10){
            M5.Lcd.fillRect(53,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,117,32,32,ORANGE);
        }
        if(line_sensor_digital[6] > 11){
            M5.Lcd.fillRect(92,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,117,32,32,GREEN);
        }
        if(line_sensor_pwm[6] > 10){
            M5.Lcd.fillRect(131,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,117,32,32,ORANGE);
        }
        if(line_sensor_pwm[7] > 10){
            M5.Lcd.fillRect(170,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,117,32,32,ORANGE);
        }
        M5.lcd.setCursor(174,121);
        M5.Lcd.print("B");

        //fourth
        if(line_sensor_digital[7] > 11){
            M5.Lcd.fillRect(14,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,156,32,32,GREEN);
        }
        if(line_sensor_digital[8] > 11){
            M5.Lcd.fillRect(53,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,156,32,32,GREEN);
        }
        if(line_sensor_pwm[8] > 10){
            M5.Lcd.fillRect(92,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,156,32,32,ORANGE);
        }
        if(line_sensor_digital[9] > 11){
            M5.Lcd.fillRect(131,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,156,32,32,GREEN);
        }
        if(line_sensor_digital[10] > 11){
            M5.Lcd.fillRect(170,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,156,32,32,GREEN);
        }

        //fifth
        if(line_sensor_pwm[9] > 10){
            M5.Lcd.fillRect(14,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,195,32,32,ORANGE);
        }
        if(line_sensor_digital[11] > 11){
            M5.Lcd.fillRect(53,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,195,32,32,GREEN);
        }
        if(line_sensor_pwm[10] > 10){
            M5.Lcd.fillRect(92,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,195,32,32,ORANGE);
        }
        M5.lcd.setCursor(96,199);
        M5.Lcd.print("C");
        if(line_sensor_digital[12] > 11){
            M5.Lcd.fillRect(131,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,195,32,32,GREEN);
        }
        if(line_sensor_pwm[11] > 10){
            M5.Lcd.fillRect(170,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,195,32,32,ORANGE);
        }

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,35);
        M5.Lcd.print("sensor A");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(215,55);
        M5.Lcd.printf("A : %d",line_sensor_pwm[0]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,74);
        M5.Lcd.print("sensor B");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(215,94);
        M5.Lcd.printf("B : %d",line_sensor_pwm[2]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,113);
        M5.Lcd.print("sensor C");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(215,133);
        M5.Lcd.printf("C : %d",line_sensor_pwm[11]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,152);
        M5.Lcd.print("sensor D");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(215,172);
        M5.Lcd.printf("D : %d",line_sensor_pwm[9]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.IMU.getTempData(&temp);
        M5.lcd.setCursor(215,191);
        M5.Lcd.print("Temp");
        M5.lcd.setCursor(215,211);
        M5.Lcd.printf("%.2f C",temp);

        delay(100);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int line_sensor_GUI_fourth(){
    int line_sensor_digital[13];
    int line_sensor_pwm[12];
    float temp = 0.0F;

    for(;;){
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("Line sensor data 4");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.Lcd.drawFastHLine(10,35,196,BLUE);
        M5.Lcd.drawFastHLine(10,74,196,BLUE);
        M5.Lcd.drawFastHLine(10,113,196,BLUE);
        M5.Lcd.drawFastHLine(10,152,196,BLUE);
        M5.Lcd.drawFastHLine(10,191,196,BLUE);
        M5.Lcd.drawFastHLine(10,230,196,BLUE);
        M5.Lcd.drawFastVLine(10,35,196,BLUE);
        M5.Lcd.drawFastVLine(49,35,196,BLUE);
        M5.Lcd.drawFastVLine(88,35,196,BLUE);
        M5.Lcd.drawFastVLine(127,35,196,BLUE);
        M5.Lcd.drawFastVLine(166,35,196,BLUE);
        M5.Lcd.drawFastVLine(205,35,196,BLUE);

        for(int i = 0;i < 13;i++){
            line_sensor_digital[i] = random(0,99);
        }
        for(int i = 0;i < 12;i++){
            line_sensor_pwm[i] = random(0,99);
        }

        M5.Lcd.setTextColor(BLUE);

        //first
        if(line_sensor_pwm[0] > 10){
            M5.Lcd.fillRect(14,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,39,32,32,ORANGE);
        }
        if(line_sensor_digital[0] > 11){
            M5.Lcd.fillRect(53,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,39,32,32,GREEN);
        }
        if(line_sensor_pwm[1] > 10){
            M5.Lcd.fillRect(92,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,39,32,32,ORANGE);
        }
        if(line_sensor_digital[1] > 11){
            M5.Lcd.fillRect(131,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,39,32,32,GREEN);
        }
        if(line_sensor_pwm[2] > 10){
            M5.Lcd.fillRect(170,39,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,39,32,32,ORANGE);
        }

        //second
        if(line_sensor_digital[2] > 11){
            M5.Lcd.fillRect(14,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,78,32,32,GREEN);
        }
        if(line_sensor_digital[3] > 11){
            M5.Lcd.fillRect(53,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,78,32,32,GREEN);
        }
        if(line_sensor_pwm[3] > 10){
            M5.Lcd.fillRect(92,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,78,32,32,ORANGE);
        }
        M5.lcd.setCursor(96,82);
        M5.Lcd.print("A");
        if(line_sensor_digital[4] > 11){
            M5.Lcd.fillRect(131,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,78,32,32,GREEN);
        }
        if(line_sensor_digital[5] > 11){
            M5.Lcd.fillRect(170,78,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,78,32,32,GREEN);
        }

        //third
        if(line_sensor_pwm[4] > 10){
            M5.Lcd.fillRect(14,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,117,32,32,ORANGE);
        }
        if(line_sensor_pwm[5] > 10){
            M5.Lcd.fillRect(53,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,117,32,32,ORANGE);
        }
        M5.lcd.setCursor(57,121);
        M5.Lcd.print("D");
        if(line_sensor_digital[6] > 11){
            M5.Lcd.fillRect(92,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,117,32,32,GREEN);
        }
        if(line_sensor_pwm[6] > 10){
            M5.Lcd.fillRect(131,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,117,32,32,ORANGE);
        }
        M5.lcd.setCursor(135,121);
        M5.Lcd.print("B");
        if(line_sensor_pwm[7] > 10){
            M5.Lcd.fillRect(170,117,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,117,32,32,ORANGE);
        }

        //fourth
        if(line_sensor_digital[7] > 11){
            M5.Lcd.fillRect(14,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,156,32,32,GREEN);
        }
        if(line_sensor_digital[8] > 11){
            M5.Lcd.fillRect(53,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,156,32,32,GREEN);
        }
        if(line_sensor_pwm[8] > 10){
            M5.Lcd.fillRect(92,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,156,32,32,ORANGE);
        }
        M5.lcd.setCursor(96,160);
        M5.Lcd.print("C");
        if(line_sensor_digital[9] > 11){
            M5.Lcd.fillRect(131,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,156,32,32,GREEN);
        }
        if(line_sensor_digital[10] > 11){
            M5.Lcd.fillRect(170,156,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,156,32,32,GREEN);
        }

        //fifth
        if(line_sensor_pwm[9] > 10){
            M5.Lcd.fillRect(14,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(14,195,32,32,ORANGE);
        }
        if(line_sensor_digital[11] > 11){
            M5.Lcd.fillRect(53,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(53,195,32,32,GREEN);
        }
        if(line_sensor_pwm[10] > 10){
            M5.Lcd.fillRect(92,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(92,195,32,32,ORANGE);
        }
        if(line_sensor_digital[12] > 11){
            M5.Lcd.fillRect(131,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(131,195,32,32,GREEN);
        }
        if(line_sensor_pwm[11] > 10){
            M5.Lcd.fillRect(170,195,32,32,LIGHTGREY);
        } else {
            M5.Lcd.fillRect(170,195,32,32,ORANGE);
        }

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,35);
        M5.Lcd.print("sensor A");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(215,55);
        M5.Lcd.printf("A : %d",line_sensor_pwm[0]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,74);
        M5.Lcd.print("sensor B");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(215,94);
        M5.Lcd.printf("B : %d",line_sensor_pwm[2]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,113);
        M5.Lcd.print("sensor C");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(215,133);
        M5.Lcd.printf("C : %d",line_sensor_pwm[11]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(215,152);
        M5.Lcd.print("sensor D");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(215,172);
        M5.Lcd.printf("D : %d",line_sensor_pwm[9]);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.IMU.getTempData(&temp);
        M5.lcd.setCursor(215,191);
        M5.Lcd.print("Temp");
        M5.lcd.setCursor(215,211);
        M5.Lcd.printf("%.2f C",temp);

        delay(100);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_gyro_mode_GUI(){
    for(;;){
        float gyroX = 0.0F;
        float gyroY = 0.0F;
        float gyroZ = 0.0F;

        int interval = 10;
        float gyro_data,sin_data,cos_data;

        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("MPU6886 Gyro data");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);
        M5.Lcd.drawFastVLine(107,36,194,BLUE);
        M5.Lcd.drawFastVLine(214,36,194,BLUE);

        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("gyro X");
        M5.Lcd.fillRect(5,56,100,14,BLACK);
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(5,56);
        M5.Lcd.printf("X%6.2f",gyroX);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(5,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(35,116);
        M5.Lcd.print("O/S");

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(112,36);
        M5.Lcd.print("gyro Y");
        M5.Lcd.fillRect(112,56,100,14,BLACK);
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(112,56);
        M5.Lcd.printf("Y%6.2f",gyroY);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(112,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(112,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(142,116);
        M5.Lcd.print("O/S");

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(219,36);
        M5.Lcd.print("gyro Z");
        M5.Lcd.fillRect(219,56,100,14,BLACK);
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(219,56);
        M5.Lcd.printf("Z%6.2f",gyroZ);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(219,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(219,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(249,116);
        M5.Lcd.print("O/S");

        M5.Lcd.fillCircle(53,180,47,BLACK);
        M5.Lcd.drawCircle(53,180,48,BLUE);
        M5.Lcd.drawFastVLine(53,134,45,BLUE);

        M5.Lcd.fillCircle(160,180,47,BLACK);
        M5.Lcd.drawCircle(160,180,48,BLUE);
        M5.Lcd.drawFastVLine(160,134,45,BLUE);

        M5.Lcd.fillCircle(267,180,47,BLACK);
        M5.Lcd.drawCircle(267,180,48,BLUE);
        M5.Lcd.drawFastVLine(267,134,45,BLUE);

        gyro_data = (180 * gyroX) / 500;
        if(gyro_data < 0) gyro_data * -1;
        gyro_data *= 3.14 / 90;
        sin_data = 53 + 47 * sin(gyro_data);
        cos_data = 180 - 47 * cos(gyro_data);
        M5.Lcd.drawLine(53,180,sin_data,cos_data,CYAN);

        gyro_data = (180 * gyroY) / 500;
        if(gyro_data < 0) gyro_data * -1;
        gyro_data *= 3.14 / 90;
        sin_data = 160 + 47 * sin(gyro_data);
        cos_data = 180 - 47 * cos(gyro_data);
        M5.Lcd.drawLine(160,180,sin_data,cos_data,MAGENTA);

        gyro_data = (180 * gyroZ) / 500;
        if(gyro_data < 0) gyro_data * -1;
        gyro_data *= 3.14 / 90;
        sin_data = 267 + 47 * sin(gyro_data);
        cos_data = 180 - 47 * cos(gyro_data);
        M5.Lcd.drawLine(267,180,sin_data,cos_data,YELLOW);

        delay(interval);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_acc_mode_GUI(){
    for(;;){
        float accX = 0.0F;
        float accY = 0.0F;
        float accZ = 0.0F;

        int interval = 1;
        float acc_data,sin_data,cos_data;

        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("MPU6886 Acc data");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);
        M5.Lcd.drawFastVLine(107,36,194,BLUE);
        M5.Lcd.drawFastVLine(214,36,194,BLUE);

        M5.IMU.getAccelData(&accX,&accY,&accZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("acc X");
        M5.Lcd.fillRect(5,56,100,14,BLACK);
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(5,56);
        M5.Lcd.printf("X%6.2f",accX);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(5,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(45,116);
        M5.Lcd.print("G");

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(112,36);
        M5.Lcd.print("acc Y");
        M5.Lcd.fillRect(112,56,100,14,BLACK);
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(112,56);
        M5.Lcd.printf("Y%6.2f",accY);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(112,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(112,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(152,116);
        M5.Lcd.print("G");

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(219,36);
        M5.Lcd.print("acc Z");
        M5.Lcd.fillRect(219,56,100,14,BLACK);
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(219,56);
        M5.Lcd.printf("Z%6.2f",accZ);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(219,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(219,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(259,116);
        M5.Lcd.print("G");

        M5.Lcd.fillCircle(53,180,47,BLACK);
        M5.Lcd.drawCircle(53,180,48,BLUE);
        M5.Lcd.drawFastVLine(53,134,45,BLUE);

        M5.Lcd.fillCircle(160,180,47,BLACK);
        M5.Lcd.drawCircle(160,180,48,BLUE);
        M5.Lcd.drawFastVLine(160,134,45,BLUE);

        M5.Lcd.fillCircle(267,180,47,BLACK);
        M5.Lcd.drawCircle(267,180,48,BLUE);
        M5.Lcd.drawFastVLine(267,134,45,BLUE);

        acc_data = (180 * accX) / 3;
        if(acc_data < 0) acc_data * -1;
        acc_data *= 3.14 / 90;
        sin_data = 53 + 47 * sin(acc_data);
        cos_data = 180 - 47 * cos(acc_data);
        M5.Lcd.drawLine(53,180,sin_data,cos_data,CYAN);

        acc_data = (180 * accY) / 3;
        if(acc_data < 0) acc_data * -1;
        acc_data *= 3.14 / 90;
        sin_data = 160 + 47 * sin(acc_data);
        cos_data = 180 - 47 * cos(acc_data);
        M5.Lcd.drawLine(160,180,sin_data,cos_data,MAGENTA);

        acc_data = (180 * accZ) / 3;
        if(acc_data < 0) acc_data * -1;
        acc_data *= 3.14 / 90;
        sin_data = 267 + 47 * sin(acc_data);
        cos_data = 180 - 47 * cos(acc_data);
        M5.Lcd.drawLine(267,180,sin_data,cos_data,YELLOW);

        delay(interval);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_gmt_mode_GUI(){
    for(;;){
        float pitch = 0.0F;
        float roll  = 0.0F;
        float yaw   = 0.0F;

        int interval = 1;
        float gmt_data,sin_data,cos_data;

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(55,5);
        M5.Lcd.print("MPU6886 GMT data");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);
        M5.Lcd.drawFastVLine(107,36,194,BLUE);
        M5.Lcd.drawFastVLine(214,36,194,BLUE);

        M5.IMU.getAhrsData(&pitch,&roll,&yaw);

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("pitch");
        M5.Lcd.fillRect(5,56,100,14,BLACK);
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(5,56);
        M5.Lcd.printf("X%6.2f",pitch);
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(5,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(5,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(15,116);
        M5.Lcd.print("degree");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(112,36);
        M5.Lcd.print("roll");
        M5.Lcd.fillRect(112,56,100,14,BLACK);
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(112,56);
        M5.Lcd.printf("Y%6.2f",roll);
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(112,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(112,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(122,116);
        M5.Lcd.print("degree");

        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(219,36);
        M5.Lcd.print("yaw");
        M5.Lcd.fillRect(219,56,100,14,BLACK);
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(219,56);
        M5.Lcd.printf("Z%6.2f",yaw);
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(219,76);
        M5.Lcd.print("interval");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(219,96);
        M5.Lcd.printf("%d ms",interval);
        M5.Lcd.setTextColor(GREEN,BLACK);
        M5.lcd.setCursor(229,116);
        M5.Lcd.print("degree");

        M5.Lcd.fillCircle(53,180,47,BLACK);
        M5.Lcd.drawCircle(53,180,48,GREEN);
        M5.Lcd.drawFastVLine(53,134,45,GREEN);

        M5.Lcd.fillCircle(160,180,47,BLACK);
        M5.Lcd.drawCircle(160,180,48,GREEN);
        M5.Lcd.drawFastVLine(160,134,45,GREEN);

        M5.Lcd.fillCircle(267,180,47,BLACK);
        M5.Lcd.drawCircle(267,180,48,GREEN);
        M5.Lcd.drawFastVLine(267,134,45,GREEN);

        gmt_data = (180 * pitch) / 360;
        if(gmt_data < 0) gmt_data * -1;
        gmt_data *= 3.14 / 90;
        sin_data = 53 + 47 * sin(gmt_data);
        cos_data = 180 - 47 * cos(gmt_data);
        M5.Lcd.drawLine(53,180,sin_data,cos_data,YELLOW);

        gmt_data = (180 * roll) / 360;
        if(gmt_data < 0) gmt_data * -1;
        gmt_data *= 3.14 / 90;
        sin_data = 160 + 47 * sin(gmt_data);
        cos_data = 180 - 47 * cos(gmt_data);
        M5.Lcd.drawLine(160,180,sin_data,cos_data,YELLOW);

        gmt_data = (180 * yaw) / 360;
        if(gmt_data < 0) gmt_data * -1;
        gmt_data *= 3.14 / 90;
        sin_data = 267 + 47 * sin(gmt_data);
        cos_data = 180 - 47 * cos(gmt_data);
        M5.Lcd.drawLine(267,180,sin_data,cos_data,YELLOW);

        delay(interval);

        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_gyro_graph_GUI_gyroX(){
    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;

    int Acquisition_status = 0,val;
    float data[285];
    for(int i = 0;i < 285;i++){
        data[i] = 157;
    }

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(20,5);
        M5.Lcd.print("MPU6886 Gyro data graph");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("gyro X :");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(105,36);
        M5.Lcd.printf("%6.2f",gyroX);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,56);
        val = map(Acquisition_status,0,285,0,100);
        M5.Lcd.printf("Acquisition status : %d%%",val);

        M5.Lcd.drawFastVLine(30,80,155,BLUE);
        M5.Lcd.drawFastHLine(11,157,305,BLUE);
        M5.lcd.setTextSize(1);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,80);
        M5.Lcd.print("500");
        M5.lcd.setCursor(5,225);
        M5.Lcd.print("-500");

        M5.Lcd.fillRect(31,80,320,77,BLACK);
        M5.Lcd.fillRect(31,158,320,77,BLACK);

        for(int i = 0;i < 284;i++){
            data[i] = data[i + 1];
        }
        data[284] = 157 - map(gyroX,-500,500,-77,77);
        for(int i = 0;i < 285;i++){
            data[i] = constrain(data[i],80,234);
        }
        for(int i = 0;i < 284;i++){
            M5.Lcd.drawLine(31 + i,data[i],32 + i,data[i + 1],CYAN);
        }
        if(Acquisition_status < 286) Acquisition_status += 1; 

        delay(50);
    
        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_gyro_graph_GUI_gyroY(){
    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;

    int Acquisition_status = 0,val;
    float data[285];
    for(int i = 0;i < 285;i++){
        data[i] = 157;
    }

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(20,5);
        M5.Lcd.print("MPU6886 Gyro data graph");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("gyro Y :");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(105,36);
        M5.Lcd.printf("%6.2f",gyroY);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,56);
        val = map(Acquisition_status,0,285,0,100);
        M5.Lcd.printf("Acquisition status : %d%%",val);

        M5.Lcd.drawFastVLine(30,80,155,BLUE);
        M5.Lcd.drawFastHLine(11,157,305,BLUE);
        M5.lcd.setTextSize(1);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,80);
        M5.Lcd.print("500");
        M5.lcd.setCursor(5,225);
        M5.Lcd.print("-500");

        M5.Lcd.fillRect(31,80,320,77,BLACK);
        M5.Lcd.fillRect(31,158,320,77,BLACK);

        for(int i = 0;i < 284;i++){
            data[i] = data[i + 1];
        }
        data[284] = 157 - map(gyroY,-500,500,-77,77);
        for(int i = 0;i < 285;i++){
            data[i] = constrain(data[i],80,234);
        }
        for(int i = 0;i < 284;i++){
            M5.Lcd.drawLine(31 + i,data[i],32 + i,data[i + 1],MAGENTA);
        }
        if(Acquisition_status < 286) Acquisition_status += 1; 

        delay(50);
    
        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_gyro_graph_GUI_gyroZ(){
    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;

    int Acquisition_status = 0,val;
    float data[285];
    for(int i = 0;i < 285;i++){
        data[i] = 157;
    }

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(20,5);
        M5.Lcd.print("MPU6886 Gyro data graph");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("gyro Z :");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(105,36);
        M5.Lcd.printf("%6.2f",gyroZ);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,56);
        val = map(Acquisition_status,0,285,0,100);
        M5.Lcd.printf("Acquisition status : %d%%",val);

        M5.Lcd.drawFastVLine(30,80,155,BLUE);
        M5.Lcd.drawFastHLine(11,157,305,BLUE);
        M5.lcd.setTextSize(1);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,80);
        M5.Lcd.print("500");
        M5.lcd.setCursor(5,225);
        M5.Lcd.print("-500");

        M5.Lcd.fillRect(31,80,320,77,BLACK);
        M5.Lcd.fillRect(31,158,320,77,BLACK);

        for(int i = 0;i < 284;i++){
            data[i] = data[i + 1];
        }
        data[284] = 157 - map(gyroZ,-500,500,-77,77);
        for(int i = 0;i < 285;i++){
            data[i] = constrain(data[i],80,234);
        }
        for(int i = 0;i < 284;i++){
            M5.Lcd.drawLine(31 + i,data[i],32 + i,data[i + 1],YELLOW);
        }
        if(Acquisition_status < 286) Acquisition_status += 1; 

        delay(50);
    
        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_acc_graph_GUI_accX(){
    float accX = 0.0F;
    float accY = 0.0F;
    float accZ = 0.0F;

    int Acquisition_status = 0,val;
    float data[285];
    for(int i = 0;i < 285;i++){
        data[i] = 157;
    }

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(20,5);
        M5.Lcd.print("MPU6886 Acc data graph");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.IMU.getAccelData(&accX,&accY,&accZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("acc X :");
        M5.Lcd.setTextColor(CYAN,BLACK);
        M5.lcd.setCursor(105,36);
        M5.Lcd.printf("%6.2f",accX);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,56);
        val = map(Acquisition_status,0,285,0,100);
        M5.Lcd.printf("Acquisition status : %d%%",val);

        M5.Lcd.drawFastVLine(30,80,155,BLUE);
        M5.Lcd.drawFastHLine(11,157,305,BLUE);
        M5.lcd.setTextSize(1);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,80);
        M5.Lcd.print("3");
        M5.lcd.setCursor(5,225);
        M5.Lcd.print("-3");

        M5.Lcd.fillRect(31,80,320,77,BLACK);
        M5.Lcd.fillRect(31,158,320,77,BLACK);

        for(int i = 0;i < 284;i++){
            data[i] = data[i + 1];
        }
        data[284] = 157 - map(accX,-2,2,-77,77);
        for(int i = 0;i < 285;i++){
            data[i] = constrain(data[i],80,234);
        }
        for(int i = 0;i < 284;i++){
            M5.Lcd.drawLine(31 + i,data[i],32 + i,data[i + 1],CYAN);
        }
        if(Acquisition_status < 286) Acquisition_status += 1; 

        delay(50);
    
        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_acc_graph_GUI_accY(){
    float accX = 0.0F;
    float accY = 0.0F;
    float accZ = 0.0F;

    int Acquisition_status = 0,val;
    float data[285];
    for(int i = 0;i < 285;i++){
        data[i] = 157;
    }

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(20,5);
        M5.Lcd.print("MPU6886 Acc data graph");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.IMU.getAccelData(&accX,&accY,&accZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("acc Y :");
        M5.Lcd.setTextColor(MAGENTA,BLACK);
        M5.lcd.setCursor(105,36);
        M5.Lcd.printf("%6.2f",accY);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,56);
        val = map(Acquisition_status,0,285,0,100);
        M5.Lcd.printf("Acquisition status : %d%%",val);

        M5.Lcd.drawFastVLine(30,80,155,BLUE);
        M5.Lcd.drawFastHLine(11,157,305,BLUE);
        M5.lcd.setTextSize(1);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,80);
        M5.Lcd.print("3");
        M5.lcd.setCursor(5,225);
        M5.Lcd.print("-3");

        M5.Lcd.fillRect(31,80,320,77,BLACK);
        M5.Lcd.fillRect(31,158,320,77,BLACK);

        for(int i = 0;i < 284;i++){
            data[i] = data[i + 1];
        }
        data[284] = 157 - map(accY,-2,2,-77,77);
        for(int i = 0;i < 285;i++){
            data[i] = constrain(data[i],80,234);
        }
        for(int i = 0;i < 284;i++){
            M5.Lcd.drawLine(31 + i,data[i],32 + i,data[i + 1],MAGENTA);
        }
        if(Acquisition_status < 286) Acquisition_status += 1; 

        delay(50);
    
        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}

int IMU_sensor_acc_graph_GUI_accZ(){
    float accX = 0.0F;
    float accY = 0.0F;
    float accZ = 0.0F;

    int Acquisition_status = 0,val;
    float data[285];
    for(int i = 0;i < 285;i++){
        data[i] = 157;
    }

    for(;;){
        M5.Lcd.setTextColor(WHITE,BLACK);
        M5.lcd.setTextSize(2);
        M5.lcd.setCursor(20,5);
        M5.Lcd.print("MPU6886 Acc data graph");
        M5.Lcd.drawFastHLine(0,25,320,BLUE);

        M5.IMU.getAccelData(&accX,&accY,&accZ);

        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,36);
        M5.Lcd.print("acc Z :");
        M5.Lcd.setTextColor(YELLOW,BLACK);
        M5.lcd.setCursor(105,36);
        M5.Lcd.printf("%6.2f",accZ);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,56);
        val = map(Acquisition_status,0,285,0,100);
        M5.Lcd.printf("Acquisition status : %d%%",val);

        M5.Lcd.drawFastVLine(30,80,155,BLUE);
        M5.Lcd.drawFastHLine(11,157,305,BLUE);
        M5.lcd.setTextSize(1);
        M5.Lcd.setTextColor(LIGHTGREY,BLACK);
        M5.lcd.setCursor(5,80);
        M5.Lcd.print("3");
        M5.lcd.setCursor(5,225);
        M5.Lcd.print("-3");

        M5.Lcd.fillRect(31,80,320,77,BLACK);
        M5.Lcd.fillRect(31,158,320,77,BLACK);

        for(int i = 0;i < 284;i++){
            data[i] = data[i + 1];
        }
        data[284] = 157 - map(accZ,-2,2,-77,77);
        for(int i = 0;i < 285;i++){
            data[i] = constrain(data[i],80,234);
        }
        for(int i = 0;i < 284;i++){
            M5.Lcd.drawLine(31 + i,data[i],32 + i,data[i + 1],YELLOW);
        }
        if(Acquisition_status < 286) Acquisition_status += 1; 

        delay(50);
    
        if(M5.BtnA.read() == 1){
            return -1;
            break;
        }
        if(M5.BtnB.read() == 1){
            reset = 1;
            break;
        }
        if(M5.BtnC.read() == 1){
            return 1;
            break;
        }
    }
}




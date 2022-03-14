#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <conio.h>
#include <windows.h>
#include "setup.h"
#include "set_color.h"

double main_speed = 50;

typedef struct {
    int color[4];
    int Digital_reflect[5][5];
    int Analog_reflect[12];
    int distance[8];
} Sensor_Value_n;
Sensor_Value_n Sensor_Value;

void Line_Sensor_serial_print();
void Line_Sensor_value();

int main(void) 
{   
    Line_Sensor_serial_print();

    int all_white = 0;
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            all_white += Sensor_Value.Digital_reflect[x][y];
        }
    }
    if (all_white != 0) {
        //Lin_sensor_get_value(DIGITAL,1);
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
        if (Covariance != 0 && Variance_x != 0) {
            //get Inclination
            double Inclination = 0;
            Inclination = Covariance / (Variance_x * Variance_x);
            //get Segment
            double Segment = 0;
            Segment = Average_value_y[2] - Inclination * Average_value_x[2];
            //memory relese
            free(Deviation_x);
            free(Deviation_y);
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
            //Main_motor_control_system(main_speed,running_angle,0,running_time,stop);
            //Main_motor_control_system(main_speed,0,spin_angle,running_time,stop);
            printf("Segment          : %lf\n", Segment);
            printf("Inclination      : %lf\n", Inclination);
            printf("running_distance : %lf\n", running_distance);
            printf("running_angle    : %lf\n", running_angle);
            printf("running_time     : %lf\n", running_time);
            printf("spin_angle       : %lf\n", spin_angle);
        }
        else {
            double Point_AA[4] = { 0,0,0,0 };
            Point_AA[0] = Average_value_x[2];
            Point_AA[1] = sqrt(9 - pow(Point_AA[0], 2.0));
            Point_AA[2] = Point_AA[1] / Point_AA[0];
            Point_AA[3] = atan2(Point_AA[1], Point_AA[0]) * 180 / M_PI;
            double x_running_distance = 0, x_running_angle = 0, x_running_time = 0;
            x_running_distance = sqrt(pow(Point_AA[0], 2.0) + pow(Point_AA[1], 2.0)) * 10;
            x_running_time = x_running_distance / (60 * M_PI / 360) / RR_Spead_195 / main_speed;
            if (Point_AA[3] > 0) x_running_angle = (Point_AA[3] - 90);
            if (Point_AA[3] < 0) x_running_angle = (Point_AA[3] + 90);
            if (Point_AA[3] == 90) x_running_angle = 0;
            //Main_motor_control_system(main_speed,running_angle,0,running_time,stop);
            printf("running_distance : %lf\n", x_running_distance);
            printf("running_angle    : %lf\n", x_running_angle);
            printf("running_time     : %lf\n", x_running_time);
        }
    }
    else if (all_white == 0) {
        double running_time = 0, running_distance;
        //Main_motor_control_system(main_speed,0,0,running_time,stop);
    }

    printf("\n");
    printf("The coordinate is array number of the sensor\n");
}

void Line_Sensor_serial_print() {
    Line_Sensor_value();
    printf("Digital \n");
    printf("\n");
    printf("y\n");
    printf("\n");
    HANDLE hStdoutHandle = GetStdHandle(STD_OUTPUT_HANDLE);
    for (int i = 4; i >= 0; i--) {
        for (int p = 0; p < 5; p++) {
            if (p == 4) {
                //printf("%d\n",Sensor_Value.Digital_reflect[p][i]);
                if (Sensor_Value.Digital_reflect[p][i] == 0) {
                    SET(COLOR, BACKGROUND_GREEN | T_BLACK);
                }
                else {
                    SET(COLOR, BACKGROUND_RED | T_BLACK);
                }
                printf(" \n");
                SET(COLOR, T_BLACK | T_WHITE);
            }
            else if (p == 0) {
                printf("%d  ", i);
                if (Sensor_Value.Digital_reflect[p][i] == 0) {
                    SET(COLOR, BACKGROUND_GREEN | T_BLACK);
                }
                else {
                    SET(COLOR, BACKGROUND_RED | T_BLACK);
                }
                printf(" ");
                SET(COLOR, T_BLACK | T_WHITE);
                printf("   ");
            }
            else {
                //printf("%d",Sensor_Value.Digital_reflect[p][i]);
                //printf("   ");
                if (Sensor_Value.Digital_reflect[p][i] == 0) {
                    SET(COLOR, BACKGROUND_GREEN | T_BLACK);
                }
                else {
                    SET(COLOR, BACKGROUND_RED | T_BLACK);
                }
                printf(" ");
                SET(COLOR, T_BLACK | T_WHITE);
                printf("   ");
            }
        }
        printf("\n");
    }
    printf("   0   1   2   3   4   x\n");
    printf("\n");
}

void Line_Sensor_value() {
    Sensor_Value.Digital_reflect[0][0] = 0;
    Sensor_Value.Digital_reflect[0][1] = 0;
    Sensor_Value.Digital_reflect[0][2] = 0;
    Sensor_Value.Digital_reflect[0][3] = 0;
    Sensor_Value.Digital_reflect[0][4] = 0;

    Sensor_Value.Digital_reflect[1][0] = 0;
    Sensor_Value.Digital_reflect[1][1] = 0;
    Sensor_Value.Digital_reflect[1][2] = 0;
    Sensor_Value.Digital_reflect[1][3] = 0;
    Sensor_Value.Digital_reflect[1][4] = 0;

    Sensor_Value.Digital_reflect[2][0] = 0;
    Sensor_Value.Digital_reflect[2][1] = 1;
    Sensor_Value.Digital_reflect[2][2] = 1;
    Sensor_Value.Digital_reflect[2][3] = 0;
    Sensor_Value.Digital_reflect[2][4] = 0;

    Sensor_Value.Digital_reflect[3][0] = 1;
    Sensor_Value.Digital_reflect[3][1] = 1;
    Sensor_Value.Digital_reflect[3][2] = 1;
    Sensor_Value.Digital_reflect[3][3] = 1;
    Sensor_Value.Digital_reflect[3][4] = 0;

    Sensor_Value.Digital_reflect[4][0] = 0;
    Sensor_Value.Digital_reflect[4][1] = 0;
    Sensor_Value.Digital_reflect[4][2] = 1;
    Sensor_Value.Digital_reflect[4][3] = 1;
    Sensor_Value.Digital_reflect[4][4] = 1;
}





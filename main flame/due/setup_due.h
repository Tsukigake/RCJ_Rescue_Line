 //main system
#define to_rad *0.01745

//MDM control system
#define no_motion   0
#define stop        1
#define start       2
#define motor_break     2
#define RR_Spead_195    0.002119053 //Reference rotation speed 195:1 angle / 1duty ratio * 1ms
#define RR_Spead_100    0.001770711 //Reference rotation speed 100:1 angle / 1duty ratio * 1ms
#define RR_Spead_180    0.003095506 //Reference rotation speed 172:1 angle / 1duty ratio * 1ms

//LED
#define ON      1
#define OFF     0

//button
#define GREEN   0
#define RED     1

//VL53L0X
#define DEFAULT_mode                0
#define LONG_RANGE              1
#define HIGH_SPEED              2
#define HIGH_ACCURACY           3
#define distance_mode           0
#define coordinate_setup_mode   1
#define coordinate_mode         2


//Line sensor
#define ANALOG  0
#define DIGITAL 1
#define ANALOG_THRESHOLD    100

//I2C
#define Arduino_maga    0x11
#define Arduino_nano    0x12	
#define M5stack_core2   0x13
#define BSCS            0x14


//line sensor
//  15_A13_53   25_D43_12   35_A08_62   45_D33_31   55_A04_81
//
//  14_D51_43   24_D45_93   34_A09_52   44_D35_21   54_D29_71
//
//  13_A14_33   23_A12_83   33_D41_42   43_A07_92   53_A05_61
//
//  12_D53_23   22_D47_73   32_A10_32   42_D37_82   52_D31_51
//
//  11_A15_13   21_D49_63   31_A11_22   41_D39_72   51_A06_41

//VL53L0X
//      1                   5   6
//

//  4       2           7           8
//
//      3

//git add xxxx.html
//git commit -m "[Add] xxxx"
//git push origin main

//git remote add origin https://github.com/kamiokannde/poseidon.git














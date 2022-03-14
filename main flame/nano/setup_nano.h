/*
The MIT License (MIT)
Copyright (c) 2022 Mitsyoshi Sugaya.
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

//MDM control system
#define no_motion 0
#define stop 1
#define brake 2
#define RR_Spead_195    0.002119053 //Reference rotation speed 195:1 angle / 1duty ratio * 1ms
#define RR_Spead_100    0.001770711 //Reference rotation speed 100:1 angle / 1duty ratio * 1ms
#define RR_Spead_180    0.003095506 //Reference rotation speed 172:1 angle / 1duty ratio * 1ms

//LED
#define on 1
#define off 0

//operation_command
#define system_comand_structure_mode_1  1
#define system_comand_structure_mode_2  2

//button
#define GREEN 0
#define RED 1

//I2C
#define first_motor_x 1
#define first_motor_y 2
#define first_motor 3
#define second_motor_x 4
#define second_motor_y 5
#define second_motor 6
#define third_motor 8
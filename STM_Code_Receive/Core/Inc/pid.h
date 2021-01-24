#ifndef PID_H__
#define PID_H__


#include <math.h>
#include "systick.h"
#include <stdbool.h>


//Set up robot parameters

#define MASS 			0.44
#define INERTIA 	0.001
#define LENGTH 		0.2
#define W_RADIUS 	0.024
#define COM_DIST 0.02
#define PI 3.14
#define FREQ 0.1047
//Set up controller parameters
#define KPHI_OUTER 10
#define KV_OUTER 5
#define KP_INNER 0.0002
#define KD_INNER 0.000001

#define MOTOR_MAX_SPEED	72.88					//rad/s = 1000 RPM
#define MAX_DUTY_CYCLE	50.0

//Choose 1 of the two

/****DIVIDE TIME CLOCK BASED ON INNER LOOP PERIOD****/
/*
 *	Inner loop: 10ms -> DIVIDE_TIME_CLK = 100.0
 *	Inner loop: 100ms -> DIVIDE_TIME_CLK = 10.0
 */

//Inner loop 10ms
//#define INNER_LOOP_PERIOD 0.01
//#define DIVIDE_TIME_CLK	100.0	

//Inner loop 100ms
#define INNER_LOOP_PERIOD 0.1
#define DIVIDE_TIME_CLK 10.0
/****************************************************/
	
/************ Function Prototype *****************/

void PID_KinematicControl(float x_fb, float y_fb, float phi_fb, float* v_out, float* w_out);
float PID_MeasureVelocity(float left, float right);
float PID_MeasureRotation(float left, float right);
void PID_DynamicInverse(float v_ref, float w_ref, float v_measure, float w_measure, float* left_torque, float* right_torque);
void PID_ReferenceInnerLoop(float v_ref, float w_ref, float* w_right_ref, float* w_left_ref);
void PID_Compensator(float w_left_ref, float w_right_ref,float* w_left_error, float* w_right_error, float measured_w_left, float measured_w_right, float* torque_left, float* torque_right);
void PID_DynamicModel(float right_torque, float left_torque, float v_measure, float w_measure, float* left_pwm_duty, float* right_pwm_duty);

#endif

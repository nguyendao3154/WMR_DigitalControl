#ifndef PID_H__
#define PID_H__


#include <math.h>
#include "systick.h"
#include <stdbool.h>


//Set up robot parameters

#define MASS 			0.75
#define INERTIA 	0.001
#define LENGTH 		0.075
#define W_RADIUS 	0.024
#define COM_DIST 0.01
#define PI 3.14
#define FREQ 0.1047
//Set up controller parameters
#define KPHI_OUTER 10
#define KV_OUTER 5
#define KP_INNER 0.0005
#define KD_INNER 0.00002

#define MOTOR_MAX_SPEED	104.7					//rad/s = 1000 RPM

/************ Function Prototype *****************/

void PID_KinematicControl(float x_fb, float y_fb, float phi_fb, float* v_out, float* w_out);
float PID_MeasureVelocity(float left, float right);
float PID_MeasureRotation(float left, float right);
void PID_DynamicInverse(float v_ref, float w_ref, float v_measure, float w_measure, float* left_torque, float* right_torque);
void PID_Compensator(float v_ref, float w_ref, float v_measure, float w_measure, float* v_error, float* w_error, float* delta_r_torque, float* delta_l_torque);
void PID_OutputDynamicControl(float r_torque_iv, float l_torque_iv, float delta_r, float delta_l, float* right_torque, float* left_torque);
void PID_DynamicModel(float right_torque, float left_torque, float v_measure, float w_measure, uint8_t* left_pwm_duty, uint8_t* right_pwm_duty);

#endif

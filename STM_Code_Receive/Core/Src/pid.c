#include "pid.h"

extern uint32_t g_systick;


//static uint32_t outer_timer;							//use to mark start of outer loop
float r_torque, l_torque;

float PID_MeasureVelocity(float left, float right) {
	return (W_RADIUS*(left + right)/2);
}
float PID_MeasureRotation(float left, float right) {
	return (W_RADIUS*(left - right)/LENGTH);
}


/*
@brief:		First block of model => Kinematic controller
@param:		float x_fb, y_fb, phi_fb: value of feedback position
					float* v_out, *w_out: pointer to output of kinematic controller
																these variables are inputs of inverse dynamic model
*/
void PID_KinematicControl(float x_fb, float y_fb, float phi_fb, float* v_out, float* w_out) {
	/*******************************/
	/*
	Note: g_systick (* 10ms) => divide by 100 to get sec
	*/
	float time = g_systick*INNER_LOOP_PERIOD;
	/************** Position tracking - van co the la hinh tron   ****************/
	float xRef = 0.5;
	float yRef = 0.4;
	/**************Trajectory number 8 - thuc chat van la hinh tron***************/
	
/*
	float xRef = 0.5 + 0.4*sin(FREQ*time);
	//float dxRef = FREQ*0.4*cos(FREQ*time);
  //float ddxRef = (-1)*FREQ*FREQ*0.4*sin(FREQ*time);
  float yRef = 0.5 + 0.4*sin(2*FREQ * time);
  //float dyRef = 2*FREQ*0.4*cos(2*FREQ*time);
  //float ddyRef = -4*FREQ*FREQ*0.4*sin(2*FREQ*time);
  //float vRef = sqrt(dxRef*dxRef + dyRef*dyRef);
 // float wRef = (dxRef*ddyRef - dyRef*ddxRef) / (dxRef*dxRef + dyRef*dyRef);
*/
	
	/************************************************/
	
	/*************Trajectory Straight Line - thuc chat la hinh tron****************/
	/*
	float xRef = 0.4 + time/60;
	float yRef = 0.4 + 0.5*time/60;
	*/
//  float dxRef = 1.0/60;
//	float dyRef = 0.5/60;
	
//	float ddxRef = 0;
//	float ddyRef = 0;
	
 // float vRef = sqrt(dxRef*dxRef + dyRef*dyRef);
 // float wRef = (dxRef*ddyRef - dyRef*ddxRef) / (dxRef*dxRef + dyRef*dyRef);
	
	/*****************************************************/
	
	float ex = xRef - x_fb;
	float ey = yRef - y_fb;
	
	float phiRef = atan2((double)ey, (double)ex);
	
	float ePhi = phiRef - phi_fb;
	
	*v_out = (float)sqrt(ex*ex + ey*ey)*KV_OUTER;
	*v_out = (*v_out)*cos(ePhi)/fabs(cos(ePhi));
	//changing sign
	if(cos(ePhi) < 0) *v_out = 0.0 -*v_out;
	ePhi = atan(tan(ePhi));													//mapping ePhi to [-pi/2;pi/2]
	float alpha = ePhi * KPHI_OUTER;
	
	//Constraint: if rotation angle is too large => reduce
	if(alpha > PI/4) alpha = PI/4;
	else if(alpha < -PI/4) alpha = -PI/4;
	
	//set max velocity for robot
	if(*v_out > 0.2) *v_out = 0.2;
	else if(*v_out < -0.2) *v_out = 0.2;
	
	*w_out = (float) (*v_out)*tan(alpha)/COM_DIST;
}

/*
@brief:		Second block of model => Inverse Dynamic Controller (IDC). This block is used parallel with the Compensator
@param:		float v_ref, w_ref: 	value of set point (output of kinematic controller)
					float v_measure, w_measure: actual measurement of v and w
					float* right_torque, *left_torque: pointer to output of inverse dynamic controller
																these variables are used in combination with compensator output to return final torque
@note:		This block is used following the Kinematic Controller, with the same frequency
*/

void PID_DynamicInverse(float v_ref, float w_ref, float v_measure, float w_measure, float* left_torque, float* right_torque) {
	float dvRef = (v_ref - v_measure)*DIVIDE_TIME_CLK;
	float dwRef = (w_ref - w_measure)*DIVIDE_TIME_CLK;
	//divide by 0.01s
	*right_torque = (W_RADIUS*(dvRef*MASS-COM_DIST*w_ref*MASS*w_ref))/2+(W_RADIUS*(dwRef*(MASS*COM_DIST*COM_DIST+INERTIA)+COM_DIST*w_ref*MASS*v_ref))/LENGTH;
	*left_torque = (W_RADIUS*(dvRef*MASS-COM_DIST*w_ref*MASS*w_ref))/2-(W_RADIUS*(dwRef*(MASS*COM_DIST*COM_DIST+INERTIA)+COM_DIST*w_ref*MASS*v_ref))/LENGTH;
}

/*
@brief:		This function calculate reference velocity and rotational speed for inner loop
@param:		float *w_left_ref, w_right_ref: ref rotational speed of 2 wheels for inner loop
					float v_ref, w_ref: reference velocity and rotational speed for inner loop
*/
void PID_ReferenceInnerLoop(float v_ref, float w_ref, float* w_right_ref, float* w_left_ref) {
	*w_right_ref = (v_ref/W_RADIUS) + (LENGTH*w_ref)/(2*W_RADIUS);
	*w_left_ref = (v_ref/W_RADIUS) - (LENGTH*w_ref)/(2*W_RADIUS);
}

/*
@brief:		Third blocks of model: Compensator. This block is set in parallel with the Inverse Dynamic Controller
@param:		float w_left_ref, w_right_ref: set point of left & right speed. These param and those from the inverse dynamic controller are similar
					float *w_left_cal, *w_right_cal: calculation of v and w
					float *w_left_error, *w_right_error: These pointers point to param of errors of previous measurement
																		On first run, these param should be 0
					float* delta_r_torque, *delta_l_torque: pointer to output of compensator block. These param are then should be added to output of IDC

@note: 		This block runs every 10ms when called in the main program
*/
void PID_Compensator(float w_left_ref, float w_right_ref,float* w_left_error, float* w_right_error, float measured_w_left, float measured_w_right, float* torque_left, float* torque_right) {
	float new_error_w_right = w_right_ref - measured_w_right;
	float new_error_w_left = w_left_ref - measured_w_left;
	
	float de_w_right = new_error_w_right - *w_right_error;
	float de_w_left = new_error_w_left - *w_left_error;
	
	*w_right_error = new_error_w_right;
	*w_left_error = new_error_w_left;
	
	float delta_right = - KP_INNER * new_error_w_right - KD_INNER*de_w_right;
	float delta_left = - KP_INNER * new_error_w_left - KD_INNER*de_w_left;
	
	*torque_right += delta_right;
	*torque_left += delta_left;
}

/*
@brief:	Dynamic model of robot. This is used to calculate control signal of motor
@param:	float: right_torque, left_torque: output of Dynamic Controller
				float v_measure, w_measure: actual measurement of v and w
				uint8_t* left_pwm_duty, *right_pwm_duty:	duty to control pwm channel => motor
*/
void PID_DynamicModel(float right_torque, float left_torque, float v_measure, float w_measure, float* left_pwm_duty, float* right_pwm_duty) {
	float dv = COM_DIST*w_measure*w_measure + (right_torque+left_torque)/(MASS*W_RADIUS);
	float dw = -COM_DIST*v_measure*w_measure*MASS/(MASS*COM_DIST*COM_DIST+INERTIA)+(right_torque-left_torque)*LENGTH/(2*W_RADIUS*(MASS*COM_DIST*COM_DIST+INERTIA));
	
	//Update measure
	v_measure += dv*INNER_LOOP_PERIOD;
	w_measure += dw*INNER_LOOP_PERIOD;
	
	float w_left = v_measure/W_RADIUS - (LENGTH*w_measure)/(2*W_RADIUS);
	float w_right = v_measure/W_RADIUS + (LENGTH*w_measure)/(2*W_RADIUS);
	
	/********change to PWM => recalculate**********/
	//*left_pwm_duty = w_left/MOTOR_MAX_SPEED*MAX_DUTY_CYCLE;				//SET to range 0 - 100
	//*right_pwm_duty = w_right/MOTOR_MAX_SPEED*MAX_DUTY_CYCLE;

	*left_pwm_duty = (fabs(w_left)*0.962 + 29.9)*w_left/fabs(w_left);
	*right_pwm_duty = (fabs(w_right)*0.962 + 29.9)*w_right/fabs(w_right);
}

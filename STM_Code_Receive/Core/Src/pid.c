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
	Note: g_systick (* 100ms) => divide by 10 to get sec
	*/
	float time = (float) (UserSysTick_GetTick()/10.0);				
	/**************Trajectory number 8***************/
	
	/*
	float xRef = 0.5 + 0.4*sin(FREQ*time);
	float dxRef = FREQ*0.4*cos(FREQ*time);
  float ddxRef = (-1)*FREQ*FREQ*0.4*sin(FREQ*time);
  float yRef = 0.5 + 0.4*sin(2*FREQ * time);
  float dyRef = 2*FREQ*0.4*cos(2*FREQ*time);
  float ddyRef = -4*FREQ*FREQ*0.4*sin(2*FREQ*time);
  float vRef = sqrt(dxRef*dxRef + dyRef*dyRef);
  float wRef = (dxRef*ddyRef - dyRef*ddxRef) / (dxRef*dxRef + dyRef*dyRef);
	*/
	
	/************************************************/
	
	/*************Trajectory Straight Line****************/
	
	float xRef = 0.4 + time/60;
	float yRef = 0.4 + 0.5*time/60;
	
	float dxRef = 1.0/60;
	float dyRef = 0.5/60;
	
	float ddxRef = 0;
	float ddyRef = 0;
	
  float vRef = sqrt(dxRef*dxRef + dyRef*dyRef);
  float wRef = (dxRef*ddyRef - dyRef*ddxRef) / (dxRef*dxRef + dyRef*dyRef);
	
	/*****************************************************/
	
	float ex = xRef - x_fb;
	float ey = yRef - y_fb;
	
	float phiRef = atan2((double)ey, (double)ex);
	
	float ePhi = phiRef - phi_fb;
	
	*v_out = (float)sqrt(ex*ex + ey*ey)*KV_OUTER;
	*v_out = (*v_out)*cos(ePhi)/fabs(cos(ePhi));										//changing sign
	ePhi = atan(tan(ePhi));
	float alpha = ePhi * KPHI_OUTER;
	
	if(fabs(alpha) > PI/4) {
		alpha = PI/4*alpha/fabs(alpha);
	}
	
	if(fabs(*v_out) > 0.5) {
		*v_out = (*v_out)*0.5/fabs(*v_out);
	}
	*w_out = (float) (*v_out)/COM_DIST*tan(alpha);
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
	float dvRef = v_ref - v_measure;
	float dwRef = w_ref - w_measure;
	
	*right_torque = (W_RADIUS*(dvRef*MASS-COM_DIST*w_ref*MASS*w_ref))/2+(W_RADIUS*(dwRef*(MASS*COM_DIST*COM_DIST+INERTIA)+COM_DIST*w_ref*MASS*v_ref))/LENGTH;
	*left_torque = (W_RADIUS*(dvRef*MASS-COM_DIST*w_ref*MASS*w_ref))/2-(W_RADIUS*(dwRef*(MASS*COM_DIST*COM_DIST+INERTIA)+COM_DIST*w_ref*MASS*v_ref))/LENGTH;
}

/*
@brief:		Third blocks of model: Compensator. This block is set in parallel with the Inverse Dynamic Controller
@param:		float v_ref, w_ref: set point of v and w. These param and those from the inverse dynamic controller are similar
					float v_measure, w_measure: actual measurement of v and w
					float *v_error, *w_error: These pointers point to param of errors of previous measurement
																		On first run, these param should be 0
					float* delta_r_torque, *delta_l_torque: pointer to output of compensator block. These param are then should be added to output of IDC
@note: 		This block runs every 10ms when called in the main program
*/
void PID_Compensator(float v_ref, float w_ref, float v_measure, float w_measure, float* v_error, float* w_error, float* delta_r_torque, float* delta_l_torque) {
	float new_error_v = v_ref - v_measure;
	float new_error_w = w_ref - w_measure;
	
	float d_ev = (new_error_v - *v_error)*100;		//divided by time interval
	float d_ew = (new_error_w - *w_error)*100;
	
	*v_error = new_error_v;
	*w_error = new_error_w;
	
	*delta_r_torque = - KP_INNER*new_error_v - KD_INNER*d_ev;
	*delta_l_torque = - KP_INNER*new_error_w - KD_INNER*d_ew;
}

/*
@brief: final function of dynamic controller
@param:	float r_torque_iv, l_torque_iv: output torque of dynamic inverse controller
				delta_r, delta_l: output of compensator
				*right_torque, *left_torque: output of dynamic controller
*/
void PID_OutputDynamicControl(float r_torque_iv, float l_torque_iv, float delta_r, float delta_l, float* right_torque, float* left_torque) {
	*right_torque = r_torque_iv + delta_r;
	*left_torque = l_torque_iv + delta_l;
}

/*
@brief:	Dynamic model of robot. This is used to calculate control signal of motor
@param:	float: right_torque, left_torque: output of Dynamic Controller
				float v_measure, w_measure: actual measurement of v and w
				uint8_t* left_pwm_duty, *right_pwm_duty:	duty to control pwm channel => motor
*/
void PID_DynamicModel(float right_torque, float left_torque, float v_measure, float w_measure, uint8_t* left_pwm_duty, uint8_t* right_pwm_duty) {
	float dv = COM_DIST*w_measure*w_measure + (right_torque+left_torque)/(MASS*W_RADIUS);
	float dw = -COM_DIST*v_measure*w_measure*MASS/(MASS*COM_DIST*COM_DIST+INERTIA)+(right_torque-left_torque)*LENGTH/(2*W_RADIUS*(MASS*COM_DIST*COM_DIST+INERTIA));
	
	//Update measure
	v_measure += dv*0.01;
	w_measure += dw*0.01;
	
	float w_left = v_measure/W_RADIUS - (LENGTH*w_measure)/(2*W_RADIUS);
	float w_right = v_measure/W_RADIUS + (LENGTH*w_measure)/(2*W_RADIUS);
	
	*left_pwm_duty = (uint8_t)w_left/MOTOR_MAX_SPEED*50;				//SET to range 0 - 100
	*right_pwm_duty = (uint8_t)w_right/MOTOR_MAX_SPEED*50;
}

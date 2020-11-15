#include "pid.h"

extern uint32_t g_systick;

//static float xRef, dxRef, ddxRef;
//static float yRef, dyRef, ddyRef;
//static float phiRef;
//static float vRef, wRef;

//static float dv_Ref, dw_Ref;
extern float left_speed, right_speed;
float r_torque, l_torque;

void PID_Calculate_Torque(float x_fb, float y_fb, float phi_fb) {
	uint32_t time = g_systick;
	float xRef = 1.1 + 0.7*sin(FREQ*time);
	float dxRef = FREQ*0.7*cos(FREQ*time);
  float ddxRef = (-1)*FREQ*FREQ*0.7*sin(FREQ*time);
  float yRef = 0.9 + 0.7*sin(2*FREQ * time);
  float dyRef = 2*FREQ*0.7*cos(2*FREQ*time);
  float ddyRef = -4*FREQ*FREQ*0.7*sin(2*FREQ*time);
  float vRef = sqrt(dxRef*dxRef + dyRef*dyRef);
  float wRef = (dxRef*ddyRef - dyRef*ddxRef) / (dxRef*dxRef + dyRef*dyRef);
	
	float ex = xRef - x_fb;
	float ey = yRef - y_fb;
	
	float phiRef = atan2((double)ey, (double)ex);
	
	float ePhi = phiRef - phi_fb;
	
	float v = sqrt(ex*ex + ey*ey)*KV_OUTER;
	v = v*cos(ePhi)/fabs(cos(ePhi));
	ePhi = atan(tan(ePhi));
	float alpha = ePhi * KPHI_OUTER;
	
	if(fabs(alpha) > PI/4) {
		alpha = PI/4*alpha/fabs(alpha);
	}
	
	if(fabs(v) > 0.5) {
		v = v*0.5/fabs(v);
	}
	float w = v/COM_CENT_DIST*tan(alpha);
	
	//to be continued
}


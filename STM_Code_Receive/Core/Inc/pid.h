#ifndef PID_H__
#define PID_H__


#include <math.h>
#include "systick.h"


//Set up robot parameters

#define MASS 			0.75
#define INERTIA 	0.001
#define WIDTH 		0.075
#define W_RADIUS 	0.024
#define COM_CENT_DIST 0.01
#define PI 3.14
#define FREQ 0.1047
//Set up controller parameters
#define KPHI_OUTER 10
#define KV_OUTER 5
#define KP_INNER 0.0002
#define KD_INNER 0.00001

#define X_COORDINATE 0
#define Y_COORDINATE 1

/************ Function Prototype *****************/

void PID_Calculate_Torque(float x_fb, float y_fb, float phi_fb);

#endif

#include "util.h"
#include <math.h>
#include "configuration.h"
#define	BALL_MAX    127
#ifdef SIMULATION 
int _cnt=0;

int get_byte(int p){
    return _cnt;
}
#define STEP 2
//void put_byte(int v){
//    static int skip=0;
//    if(skip == 0){
//        if(v >=0)_cnt+=STEP;
//        else _cnt-=STEP;
//        if(_cnt>127) _cnt =127;
//        else if(_cnt <-127) _cnt =-127;
//    }
//
//}

void put_byte(int angle_int){
        float angle_f;
        static float time = 0;
        const int gravity = 1000;
        int debug=1;
        static double velo = 0, dt = 0.01 , dv = 0, position_f = BALL_MAX, position_0 = 0;

		/* Ball reaches the tips of the beam */
		if (position_f > BALL_MAX) {
			position_f = BALL_MAX;
			position_0 = BALL_MAX;
			velo = 0;
			dv = 0;
		}
		else if (position_f < -BALL_MAX) {
			position_f = -BALL_MAX;
			position_0 = -BALL_MAX;
			velo = 0;
			dv = 0;
		}
		/* Ball gravity effect */
        angle_f = mapf(angle_int, -16, 16, -45, 45); // from tics to degrees w/o gearbox
		dv = gravity * sin((double)angle_f* 6.28 / 360) * dt;
		velo = velo + dv;
		position_f = position_f + velo * dt;
        _cnt = (int)position_f;
        if(_cnt <0){
            debug=0;
        }

}
#endif
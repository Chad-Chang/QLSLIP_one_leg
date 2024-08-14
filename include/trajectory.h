#ifndef __TRAJECTORY_H__
#define	__TRAJECTORY_H__

#include "globVariable.h"
#include <iostream>
using namespace std;
class trajectory
{
    private:

        double squat_T_pause;
        double freq_squat;
        double squat_r0;
        double squat_rc;

        double K_thrust;
        double zeta_thrust;
        double K_land;
        double zeta_land;

        double jump_r0;
        double jump_rc;
        double jump_rt;

        double jump_T_stand;
        double jump_T_crouch;
        double jump_T_pause;
        double jump_T_land;
        double jump_T_recover;
        double jump_qd_max;

        double mass;
        double r_td;
        double r_to;
        double B_m = 2000;
        double th_td = pi/2;
        double th_to= pi/2;
        double th_r; // steddy state일때의 swept 각도
        double wd;
        double r0 = 0.4;
        double du;
        double T_stand=0.1; 
        double time_now;
        bool time_reset = true; 
        double t_norm;
        int count ; 
        double time_buff;
        double th_buf1;
        double th_buf2;


        Vector2d vel_trunk_des;
        Vector2d pos_trunk_des;



    public:
        trajectory();
        ~trajectory();
        void Squat(double t,StateModel_* state_model);
        void Jumping(double t, StateModel_* state_model, int mode_admitt);
        void Hold(StateModel_* state_model);
        void trajectory_walking(double t, StateModel_* stat_model,double vx);
        Vector2d cubic_trajectory(double T_f, double T_curr, Vector2d P0, Vector2d Pf);

};



#endif // !__TRAJECTORY_H__

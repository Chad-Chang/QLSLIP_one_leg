#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include "globVariable.h"

/***************** Data Logging *****************/
FILE* fid;
FILE* fid2;
FILE* fid3;
int loop_index = 0;
const int data_frequency = 1; // frequency at which data is written to a file

char datapath[] = "../data/data.csv"; // tracking  
char datapath2[] = "../data/touchtime.csv"; // touchdown time 
char datapath3[] = "../data/QLSLIP.csv"; // touchdown time 
char filename[] = "../model/mcl_oneleg/scene.xml";


/***************** Logging 변수 선언 *****************/
Vector2d posRW_curr;  Vector2d posRW_ref; // 현재 위치와 현재 위치 레퍼런스
Vector2d velRW_curr;  Vector2d velRW_ref; // 현재 속도와 현재 속도 레퍼런스 

double body_vel; extern double body_vel_ref;
extern Vector2d r_i_FL; extern Vector2d th_i_FL; // flight phase상태에서의 trajectory
extern bool FL_phase;


void init_save_data()
{   
    fprintf(fid, "t, ");

    fprintf(fid, "posRW_r_ref, posRW_r_curr, velRW_r_ref, velRW_r_curr, "); 
    fprintf(fid, "posRW_th_ref, posRW_th_curr, velRW_th_ref, velRW_th_curr, "); 
    
    fprintf(fid, "body_vel_ref, body_vel"); // body velocity

    // // flight
    // fprintf(fid3, "r_i_pos_ref, r_i_pos_curr, r_i_vel_ref, r_i_vel_curr, "); // flight r - phase trajectory tracking 
    // fprintf(fid3, "th_i_pos_ref, th_i_pos_curr, th_i_vel_ref, th_i_vel_curr, "); //  flight th - phase trajectory tracking 
    // // stance 
    // fprintf(fid2, "touchsensor_raw, position_touch_raw, tc_time_ts, tc_time_ps, "); // stance th - phase trajectory tracking 
    // fprintf(fid2, "r_force, th_force, corri_force, gravity_force, "); // stance th - phase trajectory tracking 
    // fprintf(fid, "\n");
}

void save_data(const mjModel* m, mjData* d, StateModel_* state_model)
{
    posRW_curr = state_model->posRW;
    velRW_curr = state_model->velRW;
    posRW_ref = state_model->posRW_ref;
    velRW_ref = state_model->velRW_ref;
    body_vel = d->sensordata[3]; // x방향 속도

    fprintf(fid, "%f, ", d->time);
    
    // 기본 position tracking
    fprintf(fid, "%f, %f, %f, %f, ",posRW_ref[0] ,posRW_curr[0] ,velRW_ref[0] ,velRW_curr[0]); // r방향 로그
    fprintf(fid, "%f, %f, %f, %f, ",posRW_ref[1] ,posRW_curr[1] ,velRW_ref[1] ,velRW_curr[1]); // th방향 로그 
    
    // body vel
    fprintf(fid, "%f, %f",body_vel, body_vel_ref); 
    
    //flight phase trajectory tracking
    // if(FL_phase)
    // {
    //     fprintf(fid3, "%f, %f, %f, %f, ",r_i_FL[0] ,posRW_curr[0] ,r_i_FL[1] ,velRW_curr[0]); // r방향 trajetory
    //     fprintf(fid3, "%f, %f, %f, %f, ",th_i_FL[0] ,posRW_curr[1] ,th_i_FL[1] ,velRW_curr[1]); // th방향 trajetory
    // }
    // else
    // {
    //     //stance phase trajectory tracking
    //     fprintf(fid2, "%f, %f, %f, %f, ",posRW_ref[0] ,posRW_curr[0] ,velRW_ref[0] ,velRW_ref[0]); // r방향 로그
    //     fprintf(fid2, "%f, %f, %f, %f, ",posRW_ref[1] ,posRW_curr[1] ,velRW_ref[1] ,velRW_ref[1]); // th방향 로그 
        
    // }
    // // Don't remove the newline
    fprintf(fid, "\n");


}

#endif // DATALOGGING_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> //for bool
//#include<unistd.h> //for usl eep
#include <math.h>
//#include <resource.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "controller.h"
#include "dataLogging.h"
#include "animation.h"
#include "kinematics.h"
#include "trajectory.h"
#include <iostream>
// #include <casadi/casadi.hpp>
// #include "IpIpoptApplication.hpp"


using namespace std;

mjvFigure figPosRW;         // RW position tracking plot
mjvFigure figFOB;           // RWFOB GRF estimation plot
mjvFigure figTrunkState;    // Trunk state plot


double simEndtime = 5;	// Simulation End Time
// state parameters

/*================================기본=======================================*/
StateModel_ state_Model_FL;
const int leg_FL_no = 0;
controller ctrl_FL; // other class is in main loop
kinematics kin_FL;
trajectory tra_FL;
Matrix2d jnt2bi; Matrix2d bi2jnt; // joint to biarticular

int flag_DOB = 1;           // flag for switching ON/OFF RWDOB
int flag_admitt = 0; 


/*================================QLSLIP=======================================*/
// feedback control gain 값들
double Bm = 3500;
double K_spring = 2000;

// take off, touch down  angle
double r0  = 0.38; double th0  = pi/2; // 초기 각도

// reference 속도
double body_vel_ref = 0.5; double wd; // 각속도

// 실제 속도
double trunk_vel; // 실제 trunck vel

// trajectory값들
Vector2d r_i_FL; Vector2d th_i_FL;

//trajectory optimization constraint
Vector2d r_to_FL; // takoff r값: 현재값
Vector2d r_td_FL; // touch down r값 : 계산값.
Vector2d r_top;  // touch down r값 : 정해진 값.
Vector2d th_to_FL; // take off th값: 현재 값
Vector2d th_td_FL; // touch down th값 : 계산값.
bool FL_phase = 1; // 0 : stance, 1: flight 

/***************** Main Controller *****************/
void mycontroller(const mjModel* m, mjData* d)
{   
    
    double time_run = d->time;

    
    //Admittance Control
    ctrl_FL.admittanceCtrl(&state_Model_FL,5,2,4000, flag_admitt); //parameter(omega_n,zeta,k)

    // PID Control
    ctrl_FL.pid_gain_pos(30000,1000, 150); //(kp,kd,freq)

    ctrl_FL.pid_gain_vel(100,0,0, 150); //(kp,kd,freq)
    
    // 기본 PID + DOB
    state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans * (ctrl_FL.PID_pos(&state_Model_FL)+ctrl_FL.PID_vel(&state_Model_FL));
    // state_Model_FL.tau_bi += ctrl_FL.DOBRW(&state_Model_FL, 150, flag_DOB); 

            /*stance 제어*/
    // pos_vel_RW_force_FL = ctrl_FL.PID_pos(&state_Model_FL);
    // state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans *(pos_vel_RW_force_FL)//+ ctrl_FL.PID_vel(&state_Model_FL));
    //                         + state_Model_FL.jacbRW_trans * ctrl_FL.feedback_bi_control(&state_Model_FL, M_front, Bm,wd,K_spring)
    //                         + state_Model_FL.jacbRW_trans * ctrl_FL.Trunk_gravity_compensation(&state_Model_FL, 0)
    //                         + ctrl_FL.nonlinear_compensation_torque(&state_Model_FL)
    //                         + ctrl_FL.inertia_modulation_torque(&state_Model_FL, M_front);
            

    // Force Observer
    ctrl_FL.FOBRW(&state_Model_FL, 100); // Rotating Workspace Force Observer (RWFOB)

    d->ctrl[0] = state_Model_FL.tau_bi[0] + state_Model_FL.tau_bi[1]; 
    d->ctrl[1] = state_Model_FL.tau_bi[1];


    

    if (loop_index % data_frequency == 0) {  
        save_data(m, d, &state_Model_FL);
    }
    loop_index += 1;
}


/***************** Main Function *****************/
int main(int argc, const char** argv)
{   
    r_top << 0.37, 0;

    jnt2bi << 1,0,
        1,1;
    bi2jnt << 1,0,
        -1,1;


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);


    // make data
    d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {-88.95, -17.5, 1.8, 0,d->qpos[0], 0.27};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
    
    fid = fopen(datapath, "w");
    fid2 = fopen(datapath2, "w");
    fid3 = fopen(datapath3, "w");
    init_save_data();

    
    // Initialization
    d->qpos[1] = 0.5; //  z direction
    d->qpos[2] = pi/4; //FLHIP         //d->ctrl[0] FLHAA
    d->qpos[3] = pi/2; //FLKNEE         //d->ctrl[0] FLHAA
    
    
    state_Model_FL.posRW_ref[0] = 0.3536;
    state_Model_FL.posRW_des[0] = state_Model_FL.posRW_ref[0];
    state_Model_FL.posRW_ref[1] = pi /2;
    
    kin_FL.model_param_cal(m, d, &state_Model_FL); // state init is before. Caution Error.
 
    kin_FL.state_init(m,d, &state_Model_FL);
 
    
    // custom controller
    mjcb_control = mycontroller;

    /***************** Simulation Loop *****************/
    // use the first while condition if you want to simulate for a period.
    int i = 0;
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        // Otherwise add a cpu timer and exit this loop when it is time to render.

        mjtNum simstart = d->time;
        state_Model_FL.time = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {   
            /* Trajectory Generation */
            int cmd_motion_type = 0;
            int mode_admitt = 1;
            
            if (cmd_motion_type == 0)   // Squat
            {
                tra_FL.Squat(d->time, &state_Model_FL);
            }
            else if(cmd_motion_type == 1)
            {
                tra_FL.trajectory_walking(d->time, &state_Model_FL, body_vel_ref);
            }
            else
            {  
                tra_FL.Hold(&state_Model_FL);  // Hold stance
            }
            
            kin_FL.sensor_measure(m, d, &state_Model_FL, leg_FL_no); // get joint sensor data & calculate biarticular angles
            kin_FL.model_param_cal(m, d,&state_Model_FL); // calculate model parameters
            kin_FL.jacobianRW(&state_Model_FL);
            kin_FL.fwdKinematics_cal(&state_Model_FL);     // calculate RW Kinematics
            
            mj_step(m, d);
            // 카메라 따라다니기 
            // cam.lookat[0] = d->qpos[0];
            // cam.lookat[1] = d->qpos[1];
           
            kin_FL.state_update(&state_Model_FL);
            ctrl_FL.ctrl_update();
        }

        if (d->time >= simEndtime) {
            fclose(fid);
            fclose(fid2);
            fclose(fid3);

            break;
        }
        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        
        // update scene and render
        //opt.frame = mjFRAME_WORLD;
        //cam.lookat[0] = d->qpos[0];
        //cam.lookat[1] = 0;
        //cam.lookat[2] = 0;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);
        
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}

#include "trajectory.h"

Trajectory::Trajectory() {
 std::cout<<"Class 'Trajectory' Called"<<std::endl;
 setParam();
}


Trajectory::~Trajectory() {
}

void Trajectory::setParam() {
    RL.joint.torque.ref<<0,0,0;
    RR.joint.torque.ref<<0,0,0;
    FL.joint.torque.ref<<0,0,0;
    FR.joint.torque.ref<<0,0,0;

    RL.joint.pos.ref<<0,0,0;
    RR.joint.pos.ref<<0,0,0;
    FR.joint.pos.ref<<0,0,0;
    FR.joint.pos.ref<<0,0,0;

    RL.joint.vel.ref<<0,0,0;
    RR.joint.vel.ref<<0,0,0;
    FR.joint.vel.ref<<0,0,0;
    FR.joint.vel.ref<<0,0,0;

    CoM.pos.ref<<0.0, 0.0, 0.45;
    CoM.vel.ref<<0.0, 0.0, 0.0;
    CoM.acc.ref<<0.0, 0.0, 0.0;
    CoM.pos.goal<<0.0, 0.0, 0.45;

    CoM.ori.euler.pos.ref<<0.,0.,0.;
    CoM.ori.euler.pos.now<<0.,0.,0.;
    CoM.ori.euler.vel.now<<0.,0.,0.;

    Traj.walk.bezier.foot_height=0.15;

    RL.contact.ref=1;
    RR.contact.ref=1;
    FL.contact.ref=1;
    FR.contact.ref=1;

    Traj.moveState.done=false;
}

void Trajectory::setPreviewParam(void){
    Traj.walk.zmp.AA=MatrixNd::Zero(3,3);
    Traj.walk.zmp.BB=VectorNd::Zero(3);
    Traj.walk.zmp.CC=VectorNd::Zero(3);

    Traj.walk.zmp.Gp=VectorNd::Zero(3000);
    Traj.walk.zmp.Gx=VectorNd::Zero(3);
    Traj.walk.zmp.Gi=0;

    Traj.walk.zmp.ref=VectorNd::Zero(2);
    Traj.walk.zmp.ref_old=VectorNd::Zero(2);
    Traj.walk.zmp.ref_array=MatrixNd::Zero(2,3000);

    Traj.walk.zmp.err=VectorNd::Zero(2);
    Traj.walk.zmp.sum_e=VectorNd::Zero(2);
    Traj.walk.zmp.sum_p=VectorNd::Zero(2);

    Traj.walk.zmp.X_new=MatrixNd::Zero(3,2);
    getPreviewParam();
}

void Trajectory::getPreviewParam(void) {
    int nCount = 0;
    double temp_Gp_gain, temp_Gx_gain, temp_Gi_gain;

    Traj.walk.zmp.AA<< 1, dt, dt * dt / 2.0f\
    ,0, 1, dt\
    ,0, 0, 1;

    Traj.walk.zmp.BB << dt * dt * dt / 6.0f, dt * dt / 2.0f, dt;

    Traj.walk.zmp.CC << 1, 0, -CoM.pos.goal(2) / GRAVITY;

    FILE *fp1;
    FILE *fp2;
    FILE *fp3;

    fp1 = fopen(filepath.PREVIEW_GAIN_Gp.c_str(), "r");
    if (fp1 == NULL)printf("CAN NOT OPEN Gp TEXT FILE \n");
    while (fscanf(fp1, "%lf", &temp_Gp_gain) == 1) {
        Traj.walk.zmp.Gp[nCount] = temp_Gp_gain;
        nCount++;
    }
    fclose(fp1);
    nCount = 0;

    fp2 = fopen(filepath.PREVIEW_GAIN_Gx.c_str(), "r");
    if (fp2 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
    while (fscanf(fp2, "%lf", &temp_Gx_gain) == 1) {
        Traj.walk.zmp.Gx[nCount] = temp_Gx_gain;
        nCount++;
    }
    fclose(fp2);
    nCount = 0;

    fp3 = fopen(filepath.PREVIEW_GAIN_Gi.c_str(), "r");
    if (fp3 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
    while (fscanf(fp3, "%lf", &temp_Gi_gain) == 1) {
        Traj.walk.zmp.Gi = temp_Gi_gain;
    }
    fclose(fp3);
}

void Trajectory::Bezier_Traj(VectorNd init_EP_pos_3d, VectorNd goal_EP_pos_3d) {
 VectorNd Dist_2d_global(2);
 VectorNd now_vel_global(2);
 double tmp_foot_height;
 double alpha=1.2;
 double beta=1.2;

 now_vel_global = CoM.ori.R.yaw.ref.block(0,0,2,2) * Traj.walk.vel.now.segment(0, 2);
 Dist_2d_global = now_vel_global * Traj.walk.time.ref;

 if((RL.contact.ref+RR.contact.ref+FL.contact.ref+FR.contact.ref)==3){
     Traj.walk.bezier.enable=true;
     if(Traj.walk.vel.now(2)>0){
         if(RL.contact.ref==1 || FL.contact.ref==1){
             tmp_foot_height=Traj.walk.bezier.foot_height/beta;
         }
         if(RR.contact.ref==1 || FR.contact.ref==1){
             tmp_foot_height=Traj.walk.bezier.foot_height+0.05;
         }
     }else if(Traj.walk.vel.now(2)<0){
         if(RL.contact.ref==1 || FL.contact.ref==1){
             tmp_foot_height=Traj.walk.bezier.foot_height+0.05;
         }
         if(RR.contact.ref==1 || FR.contact.ref==1){
             tmp_foot_height=Traj.walk.bezier.foot_height/beta;
         }
     }else{
         tmp_foot_height=Traj.walk.bezier.foot_height;
     }
 }else{
     Traj.walk.bezier.enable=false;
 }

 if(Traj.walk.bezier.enable==true){
     Traj.walk.bezier.time.now=Traj.walk.bezier.cnt.now*dt;
     if(Traj.walk.bezier.cnt.now==0){
         Traj.walk.bezier.P0 << init_EP_pos_3d(0), init_EP_pos_3d(1), init_EP_pos_3d(2);
         Traj.walk.bezier.P1 << init_EP_pos_3d(0), init_EP_pos_3d(1), init_EP_pos_3d(2);
         Traj.walk.bezier.P2 << init_EP_pos_3d(0) - Dist_2d_global(0) / 2.0, init_EP_pos_3d(1) - Dist_2d_global(1) / 2.0, init_EP_pos_3d(2) + tmp_foot_height*alpha;
         Traj.walk.bezier.P3 << (init_EP_pos_3d(0) + (goal_EP_pos_3d(0) - init_EP_pos_3d(0))*2.0 / 4.0), (init_EP_pos_3d(1) + (goal_EP_pos_3d(1) - init_EP_pos_3d(1))*2.0 / 4.0), init_EP_pos_3d(2) + tmp_foot_height;
         Traj.walk.bezier.P4 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) + tmp_foot_height*alpha;
     // Traj.walk.bezier.P5 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) - 2 * Dist_2d_global(0) * tan(abs(target_base_ori_local_HS(1)));
     // Traj.walk.bezier.P6 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) - 2 * Dist_2d_global(0) * tan(abs(target_base_ori_local_HS(1)));  
         Traj.walk.bezier.P5 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) - 2 * Dist_2d_global(0) * tan(abs(0.0));
         Traj.walk.bezier.P6 << goal_EP_pos_3d(0), goal_EP_pos_3d(1), init_EP_pos_3d(2) - 2 * Dist_2d_global(0) * tan(abs(0.0));

         Traj.walk.bezier.pos = Traj.walk.bezier.P0;
         Traj.walk.bezier.vel << 0., 0., 0.;
         Traj.walk.bezier.acc << 0., 0., 0.;
         Traj.walk.bezier.cnt.now++;
     }else if (Traj.walk.bezier.cnt.now<Traj.walk.fly.cnt.ref){
         Traj.walk.bezier.b0 = pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 6);
         Traj.walk.bezier.b1 = 6 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 5) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1);
         Traj.walk.bezier.b2 = 15 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2);
         Traj.walk.bezier.b3 = 20 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3);
         Traj.walk.bezier.b4 = 15 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4);
         Traj.walk.bezier.b5 = 6 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 5);  
         Traj.walk.bezier.b6 = pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 6);

         Traj.walk.bezier.b0_dot = 6 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 5)*(-1 / Traj.walk.fly.time.ref);
         Traj.walk.bezier.b1_dot = 30 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4)*(-1 / Traj.walk.fly.time.ref) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1) + 6 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 5)*(1 / Traj.walk.fly.time.ref);
         Traj.walk.bezier.b2_dot = 60 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)*(-1 / Traj.walk.fly.time.ref) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2) + 30 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1)*(1 / Traj.walk.fly.time.ref);
         Traj.walk.bezier.b3_dot = 60 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2)*(-1 / Traj.walk.fly.time.ref) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3) + 60 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2)*(1 / Traj.walk.fly.time.ref);
         Traj.walk.bezier.b4_dot = 30 * pow((1. - Traj.walk.bezier.time.now/ Traj.walk.fly.time.ref), 1)*(-1 / Traj.walk.fly.time.ref) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4) + 60 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)*(1 / Traj.walk.fly.time.ref);
         Traj.walk.bezier.b5_dot = 6 * (-1 / (Traj.walk.fly.time.ref)) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 5) + 30 * pow((1 - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4)*(1 / Traj.walk.fly.time.ref);
         Traj.walk.bezier.b6_dot = 6 * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 5)*(1 / Traj.walk.fly.time.ref);

         Traj.walk.bezier.b0_ddot = (30 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4))*pow((1 / Traj.walk.fly.time.ref),2);
         Traj.walk.bezier.b1_ddot = (120 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1)-60*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4))*pow((1 / Traj.walk.fly.time.ref),2);
         Traj.walk.bezier.b2_ddot = (180 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2)-240*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1)+30*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1))*pow((1 / Traj.walk.fly.time.ref),2);
         Traj.walk.bezier.b3_ddot = (120 * pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1) * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)-360*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2)+120*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1))*pow((1 / Traj.walk.fly.time.ref),2);
         Traj.walk.bezier.b4_ddot = (30 * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4)-240*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)+180*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 2)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3))*pow((1 / Traj.walk.fly.time.ref),2);
         Traj.walk.bezier.b5_ddot = -30 * pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4)*pow(((1/Traj.walk.fly.time.ref)),2)-30*pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 4)*(1/Traj.walk.fly.time.ref)+120*pow((1. - Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 1)* pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref), 3)*(1/Traj.walk.fly.time.ref);
         Traj.walk.bezier.b6_ddot = 30*pow((Traj.walk.bezier.time.now / Traj.walk.fly.time.ref),4)*pow((1/Traj.walk.fly.time.ref),2);

         Traj.walk.bezier.pos = Traj.walk.bezier.P0 * Traj.walk.bezier.b0 + Traj.walk.bezier.P1 * Traj.walk.bezier.b1 + Traj.walk.bezier.P2 * Traj.walk.bezier.b2 + Traj.walk.bezier.P3 * Traj.walk.bezier.b3 + Traj.walk.bezier.P4 * Traj.walk.bezier.b4 + Traj.walk.bezier.P5 * Traj.walk.bezier.b5 + Traj.walk.bezier.P6*Traj.walk.bezier.b6;
         Traj.walk.bezier.vel = Traj.walk.bezier.P0 * Traj.walk.bezier.b0_dot + Traj.walk.bezier.P1 * Traj.walk.bezier.b1_dot + Traj.walk.bezier.P2 * Traj.walk.bezier.b2_dot + Traj.walk.bezier.P3 * Traj.walk.bezier.b3_dot + Traj.walk.bezier.P4 * Traj.walk.bezier.b4_dot + Traj.walk.bezier.P5 * Traj.walk.bezier.b5_dot + Traj.walk.bezier.P6*Traj.walk.bezier.b6_dot;
         Traj.walk.bezier.acc = Traj.walk.bezier.P0 * Traj.walk.bezier.b0_ddot + Traj.walk.bezier.P1 * Traj.walk.bezier.b1_ddot + Traj.walk.bezier.P2 * Traj.walk.bezier.b2_ddot + Traj.walk.bezier.P3 * Traj.walk.bezier.b3_ddot + Traj.walk.bezier.P4 * Traj.walk.bezier.b4_ddot + Traj.walk.bezier.P5 * Traj.walk.bezier.b5_ddot + Traj.walk.bezier.P6*Traj.walk.bezier.b6_ddot;
         Traj.walk.bezier.cnt.now++;
     }

     if(Traj.walk.bezier.cnt.now==Traj.walk.fly.cnt.ref){
         Traj.walk.bezier.cnt.now=0;
         Traj.walk.bezier.pos=Traj.walk.bezier.P6;
         Traj.walk.bezier.vel<<0.,0.,0.;
         Traj.walk.bezier.acc<<0.,0.,0.;
         Traj.walk.bezier.enable=false;
     }
 }

 if(Traj.walk.bezier.enable==false){
     Traj.walk.bezier.cnt.now=0;
     Traj.walk.bezier.pos = init_EP_pos_3d;
     Traj.walk.bezier.vel << 0., 0., 0.;
     Traj.walk.bezier.acc << 0., 0., 0.;
 }
}

void Trajectory::FootStepPlanning(void){

 Traj.walk.footstep.increment(0) = Traj.walk.time.ref * Traj.walk.vel.ref(0);
 Traj.walk.footstep.increment(1) = Traj.walk.time.ref * Traj.walk.vel.ref(1);
 Traj.walk.footstep.increment(2) = Traj.walk.time.ref * Traj.walk.vel.ref(2);

 CoM.ori.euler.pos.goal(2)=CoM.ori.euler.pos.init(2)+Traj.walk.footstep.increment(2);

 Traj.walk.footstep.R_incre<< cos(Traj.walk.footstep.increment(2)), -sin(Traj.walk.footstep.increment(2))\
 , sin(Traj.walk.footstep.increment(2)), cos(Traj.walk.footstep.increment(2));

 Traj.walk.footstep.R_init<< cos(CoM.ori.euler.pos.init(2)), -sin(CoM.ori.euler.pos.init(2))\
 , sin(CoM.ori.euler.pos.init(2)), cos(CoM.ori.euler.pos.init(2));

 Traj.walk.footstep.R_goal<< cos(CoM.ori.euler.pos.goal(2)), -sin(CoM.ori.euler.pos.goal(2))\
 , sin(CoM.ori.euler.pos.goal(2)), cos(CoM.ori.euler.pos.goal(2));

 Traj.walk.footstep.tmp_RL_pos = Traj.walk.footstep.R_incre * (RL.ep.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);
 Traj.walk.footstep.tmp_RR_pos = Traj.walk.footstep.R_incre * (RR.ep.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);
 Traj.walk.footstep.tmp_FL_pos = Traj.walk.footstep.R_incre * (FL.ep.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);
 Traj.walk.footstep.tmp_FR_pos = Traj.walk.footstep.R_incre * (FR.ep.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);

 Traj.walk.footstep.delta_x_local<<Traj.walk.footstep.increment(0), 0.0;
 Traj.walk.footstep.delta_y_local<<0.0, Traj.walk.footstep.increment(1);

 Traj.walk.footstep.nx_local<<1,0;
 Traj.walk.footstep.ny_local<<0,1;

 Traj.walk.footstep.nx_global<<Traj.walk.footstep.R_goal*Traj.walk.footstep.nx_local;
 Traj.walk.footstep.ny_global<<Traj.walk.footstep.R_goal*Traj.walk.footstep.ny_local;

 Traj.walk.footstep.tmp_RL_pos2 = Traj.walk.footstep.tmp_RL_pos + Traj.walk.footstep.R_goal*Traj.walk.footstep.delta_x_local;
 Traj.walk.footstep.tmp_FR_pos2 = Traj.walk.footstep.tmp_FR_pos + Traj.walk.footstep.R_goal*Traj.walk.footstep.delta_x_local;
 Traj.walk.footstep.tmp_RR_pos2 = Traj.walk.footstep.tmp_RR_pos + ((Traj.walk.footstep.tmp_RL_pos - Traj.walk.footstep.tmp_RR_pos).transpose() * Traj.walk.footstep.nx_global)*Traj.walk.footstep.nx_global + Traj.walk.footstep.R_goal * Traj.walk.footstep.delta_x_local / 2.0;
 Traj.walk.footstep.tmp_FL_pos2 = Traj.walk.footstep.tmp_FL_pos + ((Traj.walk.footstep.tmp_FR_pos - Traj.walk.footstep.tmp_FL_pos).transpose() * Traj.walk.footstep.nx_global)*Traj.walk.footstep.nx_global + Traj.walk.footstep.R_goal * Traj.walk.footstep.delta_x_local / 2.0;
 
 FL.ep.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_FL_pos2 + Traj.walk.footstep.R_goal*Traj.walk.footstep.delta_y_local;
 FR.ep.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_FR_pos2 + Traj.walk.footstep.R_goal*Traj.walk.footstep.delta_y_local;
 RL.ep.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_RL_pos2 +((Traj.walk.footstep.tmp_FL_pos2 - Traj.walk.footstep.tmp_RL_pos2).transpose() * Traj.walk.footstep.ny_global)*Traj.walk.footstep.ny_global + Traj.walk.footstep.R_goal * Traj.walk.footstep.delta_y_local / 2.0;
 RR.ep.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_RR_pos2 +((Traj.walk.footstep.tmp_FR_pos2 - Traj.walk.footstep.tmp_RR_pos2).transpose() * Traj.walk.footstep.ny_global)*Traj.walk.footstep.ny_global + Traj.walk.footstep.R_goal * Traj.walk.footstep.delta_y_local / 2.0;

 CoM.pos.goal(0) = (RL.ep.global.pos.goal(0) + RR.ep.global.pos.goal(0) + FL.ep.global.pos.goal(0) + FR.ep.global.pos.goal(0)) / 4.0;
 CoM.pos.goal(1) = (RL.ep.global.pos.goal(1) + RR.ep.global.pos.goal(1) + FL.ep.global.pos.goal(1) + FR.ep.global.pos.goal(1)) / 4.0;
}

void Trajectory::ZMP_PreviewControl(void){
 Traj.walk.zmp.ref_old=Traj.walk.zmp.CC.transpose()*Traj.walk.zmp.X_new;
 Traj.walk.zmp.err=Traj.walk.zmp.ref_old-Traj.walk.zmp.ref_array.block(0,0,2,1);
 Traj.walk.zmp.sum_e=Traj.walk.zmp.sum_e+Traj.walk.zmp.err;

 Traj.walk.zmp.sum_p<<0.,0.;
 Traj.walk.zmp.sum_p=Traj.walk.zmp.ref_array*Traj.walk.zmp.Gp;

 Traj.walk.zmp.u = -Traj.walk.zmp.Gi*Traj.walk.zmp.sum_e-Traj.walk.zmp.X_new.transpose()*Traj.walk.zmp.Gx-Traj.walk.zmp.sum_p;
 Traj.walk.zmp.X_new=Traj.walk.zmp.AA*Traj.walk.zmp.X_new+Traj.walk.zmp.BB*Traj.walk.zmp.u.transpose();
}

void Trajectory::BodyMove_Traj(void){
 // double tmp_target_yaw;
 MatrixNd tmp_R(2,2);

 if(Traj.walk.zmp.cnt.now==0){
     Traj.walk.zmp.swing.local_x<<0.05,0.;
     Traj.walk.zmp.swing.local_y<<0.,0.08;
 }
 
 tmp_target_yaw = CoM.ori.euler.pos.init(2) + (CoM.ori.euler.pos.goal(2) - CoM.ori.euler.pos.init(2)) / 2.0 * (1. - cos(PI / Traj.walk.time.ref * Traj.walk.zmp.cnt.now*dt));

 tmp_R<<cos(tmp_target_yaw),-sin(tmp_target_yaw)\
 ,sin(tmp_target_yaw),cos(tmp_target_yaw);
 
 Traj.walk.zmp.swing.global_x=tmp_R*Traj.walk.zmp.swing.local_x;
 Traj.walk.zmp.swing.global_y=tmp_R*Traj.walk.zmp.swing.local_y;

 if(Traj.walk.zmp.cnt.now<=(Traj.walk.fly.cnt.ref)){
     Traj.walk.zmp.ref=(RL.ep.global.pos.init.segment(0,2)+FR.ep.global.pos.init.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref/4)){
     Traj.walk.zmp.ref=(RR.ep.global.pos.goal.segment(0,2)+FL.ep.global.pos.init.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref/4+Traj.walk.fly.cnt.ref)){
     Traj.walk.zmp.ref=(RR.ep.global.pos.goal.segment(0,2)+FL.ep.global.pos.init.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*2/4)){
     Traj.walk.zmp.ref=(RL.ep.global.pos.goal.segment(0,2)+FR.ep.global.pos.init.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;    

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*2/4+Traj.walk.fly.cnt.ref)){
     Traj.walk.zmp.ref=(RL.ep.global.pos.goal.segment(0,2)+FR.ep.global.pos.init.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;    

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*3/4)){
     Traj.walk.zmp.ref=(RR.ep.global.pos.goal.segment(0,2)+FL.ep.global.pos.goal.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;    

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*3/4+Traj.walk.fly.cnt.ref)){
     Traj.walk.zmp.ref=(RR.ep.global.pos.goal.segment(0,2)+FL.ep.global.pos.goal.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;    

 }else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*4/4)){
     Traj.walk.zmp.ref=(RL.ep.global.pos.goal.segment(0,2)+FR.ep.global.pos.goal.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;    

 }else{
     Traj.walk.zmp.ref=(RL.ep.global.pos.goal.segment(0,2)+FR.ep.global.pos.goal.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;    
 }


 Traj.walk.zmp.ref_array.block(0,0,2,2999)=Traj.walk.zmp.ref_array.block(0,1,2,2999);
 Traj.walk.zmp.ref_array.block(0,2999,2,1)=Traj.walk.zmp.ref;
 
 ZMP_PreviewControl();

 CoM.pos.ref(0)=Traj.walk.zmp.X_new(0,0);
 CoM.pos.ref(1)=Traj.walk.zmp.X_new(0,1);

 CoM.vel.ref(0)=Traj.walk.zmp.X_new(1,0);
 CoM.vel.ref(1)=Traj.walk.zmp.X_new(1,1);

 CoM.acc.ref(0)=Traj.walk.zmp.X_new(2,0);
 CoM.acc.ref(1)=Traj.walk.zmp.X_new(2,1);

 Traj.walk.zmp.cnt.now++;

 if(Traj.walk.zmp.cnt.now==(Traj.walk.cnt.ref)){
     Traj.walk.zmp.cnt.now=0;
 }

}

void Trajectory::FootMove_Traj(unsigned int _i){
 if(Traj.moveState.stop==false){
     if(_i<Traj.walk.fly.cnt.ref){
         if(_i==0){
             RR.contact.ref=0;
             RL.ep.global.pos.pre_init(2)=RL.ep.global.pos.ref(2);
             RR.ep.global.pos.pre_init(2)=RR.ep.global.pos.ref(2);
             FL.ep.global.pos.pre_init(2)=FL.ep.global.pos.ref(2);
             FR.ep.global.pos.pre_init(2)=FR.ep.global.pos.ref(2);

             RL.ep.global.pos.init(2)=RL.ep.global.pos.pre_init(2);
             RR.ep.global.pos.init(2)=RR.ep.global.pos.pre_init(2);
             FL.ep.global.pos.init(2)=FL.ep.global.pos.pre_init(2);
             FR.ep.global.pos.init(2)=FR.ep.global.pos.pre_init(2);
         }

         RL.ep.global.pos.ref=RL.ep.global.pos.pre_init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;

         Bezier_Traj(RR.ep.global.pos.pre_init,RR.ep.global.pos.init);
         RR.ep.global.pos.ref=Traj.walk.bezier.pos;
         RR.ep.global.vel.ref=Traj.walk.bezier.vel;
         RR.ep.global.acc.ref=Traj.walk.bezier.acc;

         FL.ep.global.pos.ref=FL.ep.global.pos.pre_init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;

         FR.ep.global.pos.ref=FR.ep.global.pos.pre_init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
     }else if(_i<(Traj.walk.fly.cnt.ref+Traj.walk.stance.cnt.ref)){
         RL.ep.global.pos.ref=RL.ep.global.pos.pre_init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;
         RL.contact.ref=1;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;
         RR.contact.ref=1;

         FL.ep.global.pos.ref=FL.ep.global.pos.pre_init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;
         FL.contact.ref=1;

         FR.ep.global.pos.ref=FR.ep.global.pos.pre_init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
         FR.contact.ref=1;
     }else if(_i<(Traj.walk.cnt.ref/4+Traj.walk.fly.cnt.ref)){
         if(_i==Traj.walk.cnt.ref/4){
             RL.contact.ref=0;
             RL.ep.global.pos.pre_init(2)=RL.ep.global.pos.ref(2);
             RR.ep.global.pos.pre_init(2)=RR.ep.global.pos.ref(2);
             FL.ep.global.pos.pre_init(2)=FL.ep.global.pos.ref(2);
             FR.ep.global.pos.pre_init(2)=FR.ep.global.pos.ref(2);

             RL.ep.global.pos.init(2)=RL.ep.global.pos.pre_init(2);
             RR.ep.global.pos.init(2)=RR.ep.global.pos.pre_init(2);
             FL.ep.global.pos.init(2)=FL.ep.global.pos.pre_init(2);
             FR.ep.global.pos.init(2)=FR.ep.global.pos.pre_init(2);
         }

         Bezier_Traj(RL.ep.global.pos.pre_init,RL.ep.global.pos.init);
         RL.ep.global.pos.ref=Traj.walk.bezier.pos;
         RL.ep.global.vel.ref=Traj.walk.bezier.vel;
         RL.ep.global.acc.ref=Traj.walk.bezier.acc;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;

         FL.ep.global.pos.ref=FL.ep.global.pos.pre_init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;

         FR.ep.global.pos.ref=FR.ep.global.pos.pre_init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
     }else if(_i<Traj.walk.cnt.ref*2/4){
         RL.ep.global.pos.ref=RL.ep.global.pos.init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;
         RL.contact.ref=1;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;
         RR.contact.ref=1;

         FL.ep.global.pos.ref=FL.ep.global.pos.pre_init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;
         FL.contact.ref=1;

         FR.ep.global.pos.ref=FR.ep.global.pos.pre_init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
         FR.contact.ref=1;

     }else if(_i<(Traj.walk.cnt.ref*2/4+Traj.walk.fly.cnt.ref)){
         if(_i==Traj.walk.cnt.ref*2/4){
             FL.contact.ref=0;
             RL.ep.global.pos.pre_init(2)=RL.ep.global.pos.ref(2);
             RR.ep.global.pos.pre_init(2)=RR.ep.global.pos.ref(2);
             FL.ep.global.pos.pre_init(2)=FL.ep.global.pos.ref(2);
             FR.ep.global.pos.pre_init(2)=FR.ep.global.pos.ref(2);

             RL.ep.global.pos.init(2)=RL.ep.global.pos.pre_init(2);
             RR.ep.global.pos.init(2)=RR.ep.global.pos.pre_init(2);
             FL.ep.global.pos.init(2)=FL.ep.global.pos.pre_init(2);
             FR.ep.global.pos.init(2)=FR.ep.global.pos.pre_init(2);
         }

         RL.ep.global.pos.ref=RL.ep.global.pos.init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;

         Bezier_Traj(FL.ep.global.pos.pre_init,FL.ep.global.pos.init);
         FL.ep.global.pos.ref=Traj.walk.bezier.pos;
         FL.ep.global.vel.ref=Traj.walk.bezier.vel;
         FL.ep.global.acc.ref=Traj.walk.bezier.acc;
         
         FR.ep.global.pos.ref=FR.ep.global.pos.pre_init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
     }else if(_i<Traj.walk.cnt.ref*3/4){
         RL.ep.global.pos.ref=RL.ep.global.pos.init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;
         RL.contact.ref=1;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;
         RR.contact.ref=1;

         FL.ep.global.pos.ref=FL.ep.global.pos.init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;
         FL.contact.ref=1;

         FR.ep.global.pos.ref=FR.ep.global.pos.pre_init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
         FR.contact.ref=1;
     }else if(_i<(Traj.walk.cnt.ref*3/4+Traj.walk.fly.cnt.ref)){
         if(_i==Traj.walk.cnt.ref*3/4){
             FR.contact.ref=0;
             RL.ep.global.pos.pre_init(2)=RL.ep.global.pos.ref(2);
             RR.ep.global.pos.pre_init(2)=RR.ep.global.pos.ref(2);
             FL.ep.global.pos.pre_init(2)=FL.ep.global.pos.ref(2);
             FR.ep.global.pos.pre_init(2)=FR.ep.global.pos.ref(2);

             RL.ep.global.pos.init(2)=RL.ep.global.pos.pre_init(2);
             RR.ep.global.pos.init(2)=RR.ep.global.pos.pre_init(2);
             FL.ep.global.pos.init(2)=FL.ep.global.pos.pre_init(2);
             FR.ep.global.pos.init(2)=FR.ep.global.pos.pre_init(2);
         }

         RL.ep.global.pos.ref=RL.ep.global.pos.init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;

         FL.ep.global.pos.ref=FL.ep.global.pos.init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;

         Bezier_Traj(FR.ep.global.pos.pre_init,FR.ep.global.pos.init);
         FR.ep.global.pos.ref=Traj.walk.bezier.pos;
         FR.ep.global.vel.ref=Traj.walk.bezier.vel;
         FR.ep.global.acc.ref=Traj.walk.bezier.acc;
         // FR.global.pos.ref=FR.global.pos.ref;
         // FR.global.vel.ref<<0.,0.,0.;
     }else{
         RL.ep.global.pos.ref=RL.ep.global.pos.init;
         RL.ep.global.pos.ref(2)=RL.ep.global.pos.ref(2);
         RL.ep.global.vel.ref<<0.,0.,0.;
         RL.ep.global.acc.ref<<0.,0.,0.;
         RL.contact.ref=1;

         RR.ep.global.pos.ref=RR.ep.global.pos.init;
         RR.ep.global.pos.ref(2)=RR.ep.global.pos.ref(2);
         RR.ep.global.vel.ref<<0.,0.,0.;
         RR.ep.global.acc.ref<<0.,0.,0.;
         RR.contact.ref=1;

         FL.ep.global.pos.ref=FL.ep.global.pos.init;
         FL.ep.global.pos.ref(2)=FL.ep.global.pos.ref(2);
         FL.ep.global.vel.ref<<0.,0.,0.;
         FL.ep.global.acc.ref<<0.,0.,0.;
         FL.contact.ref=1;

         FR.ep.global.pos.ref=FR.ep.global.pos.init;
         FR.ep.global.pos.ref(2)=FR.ep.global.pos.ref(2);
         FR.ep.global.vel.ref<<0.,0.,0.;
         FR.ep.global.acc.ref<<0.,0.,0.;
         FR.contact.ref=1;
     }
 }
}


void Trajectory::Init_Pose_Traj(void){
    Traj.init.cnt.ref=2000;
    Traj.init.time.ref=Traj.init.cnt.ref*dt;

    if(Traj.cnt==0){
        RL.joint.pos.init=RL.joint.pos.now;
        RL.joint.pos.ref=RL.joint.pos.init;
        RL.joint.vel.ref<<0.,0.,0.;
        RL.joint.pos.goal<<0.*D2R,45*D2R,-90*D2R;
        RR.joint.pos.init=RR.joint.pos.now;
        RR.joint.pos.ref=RR.joint.pos.init;
        RR.joint.vel.ref<<0.,0.,0.;
        RR.joint.pos.goal<<0.*D2R,45*D2R,-90*D2R;
        FL.joint.pos.init=FL.joint.pos.now;
        FL.joint.pos.ref=FL.joint.pos.init;
        FL.joint.vel.ref<<0.,0.,0.;
        FL.joint.pos.goal<<0.*D2R,45*D2R,-90*D2R;
        FR.joint.pos.init=FR.joint.pos.now;
        FR.joint.pos.ref=FR.joint.pos.init;
        FR.joint.vel.ref<<0.,0.,0.;
        FR.joint.pos.goal<<0.*D2R,45*D2R,-90*D2R;         
        Traj.cnt++;
    }else if(Traj.cnt<Traj.init.cnt.ref){
        RL.joint.pos.ref=RL.joint.pos.init+(RL.joint.pos.goal-RL.joint.pos.init)/2.0*(1.0-cos(PI/Traj.init.time.ref*Traj.cnt*dt));
        RL.joint.vel.ref=PI/Traj.init.time.ref*(RL.joint.pos.goal-RL.joint.pos.init)/2.0*sin(PI/Traj.init.time.ref*Traj.cnt*dt);
        RR.joint.pos.ref=RR.joint.pos.init+(RR.joint.pos.goal-RR.joint.pos.init)/2.0*(1.0-cos(PI/Traj.init.time.ref*Traj.cnt*dt));
        RR.joint.vel.ref=PI/Traj.init.time.ref*(RR.joint.pos.goal-RR.joint.pos.init)/2.0*sin(PI/Traj.init.time.ref*Traj.cnt*dt);
        FL.joint.pos.ref=FL.joint.pos.init+(FL.joint.pos.goal-FL.joint.pos.init)/2.0*(1.0-cos(PI/Traj.init.time.ref*Traj.cnt*dt));
        FL.joint.vel.ref=PI/Traj.init.time.ref*(FL.joint.pos.goal-FL.joint.pos.init)/2.0*sin(PI/Traj.init.time.ref*Traj.cnt*dt);
        FR.joint.pos.ref=FR.joint.pos.init+(FR.joint.pos.goal-FR.joint.pos.init)/2.0*(1.0-cos(PI/Traj.init.time.ref*Traj.cnt*dt));
        FR.joint.vel.ref=PI/Traj.init.time.ref*(FR.joint.pos.goal-FR.joint.pos.init)/2.0*sin(PI/Traj.init.time.ref*Traj.cnt*dt);
        Traj.cnt++;
    }else{
        RL.joint.pos.ref=RL.joint.pos.goal;
        RL.joint.vel.ref<<0.,0.,0.;
        RR.joint.pos.ref=RR.joint.pos.goal;
        RR.joint.vel.ref<<0.,0.,0.;
        FL.joint.pos.ref=FL.joint.pos.goal;
        FL.joint.vel.ref<<0.,0.,0.;
        FR.joint.pos.ref=FR.joint.pos.goal;
        FR.joint.vel.ref<<0.,0.,0.;
        Traj.moveState.done=true;
    }
}

void Trajectory::Ready_Pose_Traj(void){
    Traj.ready.cnt.ref=2000;
    Traj.ready.time.ref=Traj.ready.cnt.ref*dt;

    if(Traj.cnt==0){
        CoM.pos.init=CoM.pos.now;
        CoM.pos.goal<<0.0,0.0,0.45;
        CoM.pos.ref=CoM.pos.init;
        
        Base.pos.init=GetBasePos(CoM.pos.init, CoM.ori.R.now);
        CoM.vel.ref<<0.,0.,0.;

        //*******************************************************************//
        RL.ep.global.pos.init=Base.pos.init+CoM.ori.R.now*RL.ep.local.pos.now;
        RR.ep.global.pos.init=Base.pos.init+CoM.ori.R.now*RR.ep.local.pos.now;
        FL.ep.global.pos.init=Base.pos.init+CoM.ori.R.now*FL.ep.local.pos.now;
        FR.ep.global.pos.init=Base.pos.init+CoM.ori.R.now*FR.ep.local.pos.now;

        RL.ep.global.pos.goal<<-0.35,0.22,0.0;
        RR.ep.global.pos.goal<<-0.35,-0.22,0.0;
        FL.ep.global.pos.goal<<0.35,0.22,0.0;
        FR.ep.global.pos.goal<<0.35,-0.22,0.0;

        RL.ep.global.pos.ref=RL.ep.global.pos.init;
        RR.ep.global.pos.ref=RR.ep.global.pos.init;
        FL.ep.global.pos.ref=FL.ep.global.pos.init;
        FR.ep.global.pos.ref=FR.ep.global.pos.init;

        RL.ep.global.vel.ref<<0.,0.,0.;
        RR.ep.global.vel.ref<<0.,0.,0.;
        FL.ep.global.vel.ref<<0.,0.,0.;
        FR.ep.global.vel.ref<<0.,0.,0.;

        Traj.cnt++;
    }else if(Traj.cnt<Traj.ready.cnt.ref){
        CoM.pos.ref=CoM.pos.init+(CoM.pos.goal-CoM.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
        CoM.vel.ref=PI/Traj.ready.time.ref*(CoM.pos.goal-CoM.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));

        RL.ep.global.pos.ref=RL.ep.global.pos.init+(RL.ep.global.pos.goal-RL.ep.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
        RR.ep.global.pos.ref=RR.ep.global.pos.init+(RR.ep.global.pos.goal-RR.ep.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
        FL.ep.global.pos.ref=FL.ep.global.pos.init+(FL.ep.global.pos.goal-FL.ep.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
        FR.ep.global.pos.ref=FR.ep.global.pos.init+(FR.ep.global.pos.goal-FR.ep.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));

        RL.ep.global.vel.ref=PI/Traj.ready.time.ref*(RL.ep.global.pos.goal-RL.ep.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
        RR.ep.global.vel.ref=PI/Traj.ready.time.ref*(RR.ep.global.pos.goal-RR.ep.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
        FL.ep.global.vel.ref=PI/Traj.ready.time.ref*(FL.ep.global.pos.goal-FL.ep.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
        FR.ep.global.vel.ref=PI/Traj.ready.time.ref*(FR.ep.global.pos.goal-FR.ep.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
        Traj.cnt++;
    }else{
        if(Traj.moveState.done==false){
            CoM.pos.ref=CoM.pos.goal;
            CoM.vel.ref<<0.0,0.0,0.0;

            RL.ep.global.pos.ref=RL.ep.global.pos.goal;
            RR.ep.global.pos.ref=RR.ep.global.pos.goal;
            FL.ep.global.pos.ref=FL.ep.global.pos.goal;
            FR.ep.global.pos.ref=FR.ep.global.pos.goal;
            
            RL.ep.global.vel.ref<<0.,0.,0.;
            RR.ep.global.vel.ref<<0.,0.,0.;
            FL.ep.global.vel.ref<<0.,0.,0.;
            FR.ep.global.vel.ref<<0.,0.,0.;
        }
        Traj.moveState.done=true;
    }
    //DataSave();
    Base.pos.ref=GetBasePos(CoM.pos.ref, CoM.ori.R.ref);
    Base.vel.ref=CoM.vel.ref;

    // if(Traj.cnt<=Traj.ready.time/dt){
    //  cout<<"CoM:"<<CoM.pos.ref.transpose()<<endl;
    //  cout<<"RL:"<<RL.global.pos.ref.transpose()<<endl;
    //  cout<<"RR:"<<RR.global.pos.ref.transpose()<<endl;
    //  cout<<"FL:"<<FL.global.pos.ref.transpose()<<endl;
    //  cout<<"FR:"<<FR.global.pos.ref.transpose()<<endl;
    //  cout<<"-----"<<endl;    
    // }
}


void Trajectory::Slow_Walk_Traj(void){
 Traj.walk.fly.cnt.ref=700;
 Traj.walk.fly.time.ref=Traj.walk.fly.cnt.ref*dt;
 
 Traj.walk.stance.cnt.ref=50;
 Traj.walk.stance.time.ref=Traj.walk.stance.cnt.ref*dt;

 Traj.walk.cnt.ref=(Traj.walk.fly.cnt.ref+Traj.walk.stance.cnt.ref)*4;
 Traj.walk.time.ref=Traj.walk.cnt.ref*dt;
 
 if(Traj.cnt<Traj.walk.cnt.ref){
     // Traj.walk.vel.ref<<0.1, 0.0, 0.0;
     Traj.walk.vel.ref<<0.05, 0.0, 0.0;
     Init_Slow_walk_traj(Traj.cnt);
     Traj.cnt++;

 }else if(Traj.cnt<(Traj.walk.cnt.ref*2)){
     if(Traj.cnt==Traj.walk.cnt.ref){
         if(Traj.moveState.stop==true){
             if(Traj.walk.stop.body==true){
                 Traj.walk.stop.all=true;
             }
             if(Traj.walk.stop.foot==true){
                 Traj.walk.stop.body=true;
             }
             Traj.walk.stop.foot=true;
             joy.vel_x=0.0;
             joy.vel_y=0.0;
             joy.vel_yaw=0.0;
         }
         else if(Traj.moveState.stop==false){
             if(Traj.walk.stop.body==false){
                 Traj.walk.stop.all=false;
             }
             Traj.walk.stop.body=false;
             Traj.walk.stop.foot=false;
         }
         
         // Traj.walk.vel.ref<<joy.vel_x, joy.vel_y, joy.vel_yaw;
         FootStepPlanning();
     }

     CoM.ori.euler.pos.ref(2)=CoM.ori.euler.pos.pre_init(2)+(CoM.ori.euler.pos.init(2)-CoM.ori.euler.pos.pre_init(2))/2.0*(1-cos(PI/(Traj.walk.cnt.ref)*(Traj.cnt-Traj.walk.cnt.ref)));
     // cout<<CoM.ori.euler.pos.ref(2)<<endl;
     // cout<<"----"<<endl;
     FootMove_Traj(Traj.cnt-Traj.walk.cnt.ref);
     
     //CoM.pos.ref(2)=CoM.pos.init(2)+(-0.08)*(1-cos(PI/(Traj.walk.cnt.ref/8.0)*(Traj.cnt-Traj.walk.cnt.ref)));

     if (Traj.cnt == (Traj.walk.cnt.ref*2-1)) {
         Traj.walk.vel.now=Traj.walk.vel.ref;
         CoM.pos.init(0)=CoM.pos.goal(0);
         CoM.pos.init(1)=CoM.pos.goal(1);

         CoM.ori.euler.pos.pre_init(2) = CoM.ori.euler.pos.init(2);
         CoM.ori.euler.pos.init(2)=CoM.ori.euler.pos.goal(2);

         RL.ep.global.pos.pre_init=RL.ep.global.pos.init;
         RR.ep.global.pos.pre_init=RR.ep.global.pos.init;
         FL.ep.global.pos.pre_init=FL.ep.global.pos.init;
         FR.ep.global.pos.pre_init=FR.ep.global.pos.init;

         RL.ep.global.pos.init=RL.ep.global.pos.goal;
         RR.ep.global.pos.init=RR.ep.global.pos.goal;
         FL.ep.global.pos.init=FL.ep.global.pos.goal;
         FR.ep.global.pos.init=FR.ep.global.pos.goal;

         Traj.cnt=Traj.walk.cnt.ref-1;
     }
     Traj.cnt++;
 }
 BodyMove_Traj();
}

void Trajectory::Init_Slow_walk_traj(unsigned int _i){
 if(_i==0){
     CoM.pos.init=CoM.pos.goal;

     CoM.ori.euler.pos.pre_init(2)=CoM.ori.euler.pos.ref(2);
     CoM.ori.euler.pos.init(2)=CoM.ori.euler.pos.pre_init(2);

     RL.ep.global.pos.pre_init=RL.ep.global.pos.ref;
     RR.ep.global.pos.pre_init=RR.ep.global.pos.ref;
     FL.ep.global.pos.pre_init=FL.ep.global.pos.ref;
     FR.ep.global.pos.pre_init=FR.ep.global.pos.ref;
     RL.ep.global.pos.init=RL.ep.global.pos.ref;
     RR.ep.global.pos.init=RR.ep.global.pos.ref;
     FL.ep.global.pos.init=FL.ep.global.pos.ref;
     FR.ep.global.pos.init=FR.ep.global.pos.ref;

     Traj.walk.vel.now<<0.,0.,0.;
     Traj.walk.vel.ref<<0.,0.,0.;
     FootStepPlanning();
 }

}

void Trajectory::resetTrajPram(void){
    Traj.cnt=0;
    Traj.walk.bezier.cnt.now=0;
    Traj.walk.zmp.cnt.now=0;
    Traj.moveState.stop=false;
    Traj.walk.stop.body=false;
    Traj.walk.stop.foot=false;
    Traj.walk.stop.all=false;
}
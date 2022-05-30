#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "common.h"

struct COUNT{
    unsigned int now;
    unsigned int ref;
};

struct TIME{
    double now;
    double ref;
};

struct BEZIER{
    double b0, b1, b2, b3, b4, b5, b6;
    double b0_dot, b1_dot, b2_dot, b3_dot, b4_dot, b5_dot, b6_dot;
    double b0_ddot, b1_ddot, b2_ddot, b3_ddot, b4_ddot, b5_ddot, b6_ddot;

    Eigen::Vector3d P0;
    Eigen::Vector3d P1;
    Eigen::Vector3d P2;
    Eigen::Vector3d P3;
    Eigen::Vector3d P4;
    Eigen::Vector3d P5;
    Eigen::Vector3d P6;
    
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    struct COUNT cnt;
    struct TIME time;
    bool enable;
    double foot_height;
};

struct PHASE{
    struct COUNT cnt;
    struct TIME time;
};

struct FLY{
    struct COUNT cnt;
    struct TIME time;
};

struct BODYSWING{
    Eigen::Vector2d local_x;
    Eigen::Vector2d local_y;
    Eigen::Vector2d global_x;
    Eigen::Vector2d global_y;
};

struct ZMP_PREVIEW{
    struct COUNT cnt;
    struct TIME time;
    struct BODYSWING swing;
    
    Eigen::Vector2d ref;
    Eigen::Vector2d ref_old;

    MatrixNd X_new; // 3 by 2
    MatrixNd ref_array; //2 by 3000

    VectorNd sum_p; //2 by 1
    VectorNd sum_e; //2 by 1
    VectorNd err; //2 by 1

    VectorNd u; //2 by 1 

    MatrixNd AA; //3 by 3
    VectorNd BB; //3 by 1
    VectorNd CC; // 3 by 1

    VectorNd Gp; //3000 by 1
    VectorNd Gx; // 3 by 1
    double Gi; // 1
};

struct STOP_FLAG{
    bool foot;
    bool body;
    bool all;
};

struct MOVE_STATE{
    bool done;
    bool stop;
};

struct FOOTSTEP{
    Eigen::Vector3d increment;

    Eigen::Matrix2d R_incre;
    Eigen::Matrix2d R_init;
    Eigen::Matrix2d R_goal;

    Eigen::Vector2d nx_local;
    Eigen::Vector2d ny_local;
    Eigen::Vector2d nx_global;
    Eigen::Vector2d ny_global;
    Eigen::Vector2d delta_x_local;
    Eigen::Vector2d delta_y_local;

    Eigen::Vector2d tmp_RL_pos;
    Eigen::Vector2d tmp_RR_pos;
    Eigen::Vector2d tmp_FL_pos;
    Eigen::Vector2d tmp_FR_pos;

    Eigen::Vector2d tmp_RL_pos2;
    Eigen::Vector2d tmp_RR_pos2;
    Eigen::Vector2d tmp_FL_pos2;
    Eigen::Vector2d tmp_FR_pos2;
};


struct WALK{
    struct COUNT cnt;
    struct TIME time;
    struct PHASE stance;
    struct PHASE fly;

    struct BEZIER bezier;
    struct ZMP_PREVIEW zmp;
    struct VELOCITY vel;
    struct STOP_FLAG stop;
    struct FOOTSTEP footstep;
};



struct INIT{        
    struct COUNT cnt;
    struct TIME time;
};

struct READY{
    struct COUNT cnt;
    struct TIME time;
};

struct TRAJ{
    unsigned int cnt;
    struct INIT init;
    struct READY ready;
    struct WALK walk;

    struct MOVE_STATE moveState;
    bool move_done_flag;
    bool move_stop_flag;
};


class Trajectory : virtual public Common {
public:
    Trajectory();
    ~Trajectory();
public:
    TRAJ Traj;
    void setParam();
    
    void setPreviewParam(void);
    void getPreviewParam(void);
    void Bezier_Traj(VectorNd, VectorNd);
    void FootStepPlanning(void);
    void ZMP_PreviewControl(void);
    void BodyMove_Traj(void);
    void FootMove_Traj(unsigned int);

    void Init_Pose_Traj(void);
    void Ready_Pose_Traj(void);
    void Slow_Walk_Traj(void);
    void Init_Slow_walk_traj(unsigned int);

    void resetTrajPram(void);
    double tmp_target_yaw=0.0;
private:
};

#endif

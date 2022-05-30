#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <stdio.h>
#include "Eigen/Dense"
#include <unistd.h>
#include <string>

#include <math.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
// using Eigen::MatrixXd;
// using Eigen::VectorXd;
// using Eigen::Vector3d;


#define nDOF 12
#define dt 0.001
#define PI 3.141592
#define R2D 180/PI
#define D2R PI/180
#define GRAVITY 9.81
#define Robot_mass 50
#define Leg_mass 5.5922

struct FILEPATH{
    string URDF;
    string PREVIEW_GAIN_Gp;
    string PREVIEW_GAIN_Gx;
    string PREVIEW_GAIN_Gi;
    string FILE_SAVE;
};

struct RBDL{
    RigidBodyDynamics::Model* pModel;
    VectorNd RobotState;
    VectorNd RobotStateDot;
    VectorNd RobotState2Dot;
    RigidBodyDynamics::Math::VectorNd BaseState;
    RigidBodyDynamics::Math::VectorNd BaseStateDot;
    RigidBodyDynamics::Math::VectorNd BaseState2Dot;
    RigidBodyDynamics::Math::VectorNd JointState;
    RigidBodyDynamics::Math::VectorNd JointStateDot;
    RigidBodyDynamics::Math::VectorNd JointState2Dot;
    RigidBodyDynamics::Math::VectorNd EPState;
    RigidBodyDynamics::Math::VectorNd EPStateDot;
    RigidBodyDynamics::Math::VectorNd EPState2Dot;

    RigidBodyDynamics::Math::VectorNd JointState_ref;
    RigidBodyDynamics::Math::VectorNd JointStateDot_ref;
    RigidBodyDynamics::Math::VectorNd JointState2Dot_ref;
    RigidBodyDynamics::Math::VectorNd EPState_ref;
    RigidBodyDynamics::Math::VectorNd EPStateDot_ref;
    RigidBodyDynamics::Math::VectorNd EPState2Dot_ref;
};

struct TORQUE{
   Eigen::Vector3d now;
   Eigen::Vector3d ref;
};

struct POSITION{
   Eigen::Vector3d prev;
   Eigen::Vector3d now;

   Eigen::Vector3d init;
   Eigen::Vector3d ref;
   Eigen::Vector3d goal;

   Eigen::Vector3d pre_init;
   Eigen::Vector3d init_rot;
   Eigen::Vector3d ref_from_com;
   Eigen::Vector3d now_from_com;
};

struct VELOCITY{
   Eigen::Vector3d prev;
   Eigen::Vector3d now;
   Eigen::Vector3d init;
   Eigen::Vector3d ref;
   Eigen::Vector3d goal;
};

struct ACCELERATION{
   Eigen::Vector3d prev;
   Eigen::Vector3d now;

   Eigen::Vector3d init;
   Eigen::Vector3d ref;
   Eigen::Vector3d goal;
};

struct QUAT_POSITION{
    Quaternion prev;
    Quaternion now;

    Quaternion init;
    Quaternion ref;
    Quaternion goal;
};

struct QUAT_VELOCITY{
    Quaternion prev;
    Quaternion now;

    Quaternion init;
    Quaternion ref;
    Quaternion goal;
};

struct QUAT_ACCELERATION{
    Quaternion prev;
    Quaternion now;

    Quaternion init;
    Quaternion ref;
    Quaternion goal;
};


struct JOINT{
    struct TORQUE torque;
    struct POSITION pos;
    struct VELOCITY vel;
    struct ACCELERATION acc;
    Eigen::Matrix3d kp;
    Eigen::Matrix3d kd;
};

struct SEMI{
    struct POSITION pos;
    struct VELOCITY vel;
    struct ACCELERATION acc;
};

struct LOCAL{
    struct POSITION pos;
    struct VELOCITY vel;
    struct ACCELERATION acc;
    struct SEMI semi;
};

struct GLOBAL{
    struct POSITION pos;
    struct VELOCITY vel;
    struct ACCELERATION acc;
    struct SEMI semi;
};

struct CONTACT{
    unsigned int ref;
    unsigned int now;
};

struct ENDPOINT{
    struct LOCAL local;
    struct GLOBAL global;
    Eigen::Matrix3d kp;
    Eigen::Matrix3d kd;
};

struct LEG{
    int ID;
    struct JOINT joint;
    struct ENDPOINT ep;
    Eigen::Matrix3d Jac;
    struct CONTACT contact;
};

struct EULER{
    struct POSITION pos;
    struct VELOCITY vel;
    struct ACCELERATION acc;
};

struct ROLL{
    Eigen::Matrix3d ref;
    Eigen::Matrix3d now;
};

struct PITCH{
    Eigen::Matrix3d ref;
    Eigen::Matrix3d now;
};

struct YAW{
    Eigen::Matrix3d ref;
    Eigen::Matrix3d now;
    Eigen::Matrix3d init;
    Eigen::Matrix3d goal;
};

struct ROTATION_MATRIX{
    Eigen::Matrix3d ref;
    Eigen::Matrix3d now;
    struct ROLL roll;
    struct PITCH pitch;
    struct YAW yaw;
};

struct QUATERNION{
    struct QUAT_POSITION pos;
    struct QUAT_VELOCITY vel;
    struct QUAT_ACCELERATION acc;
    //struct ROTATION_MATRIX R;
};

struct ORIENTATION{
    struct EULER euler;
    struct QUATERNION quat;
    struct ROTATION_MATRIX R;
};

struct COM{
    int ID;
    struct POSITION pos;
    struct VELOCITY vel;
    struct ACCELERATION acc;
    struct ORIENTATION ori;
    struct SEMI semi;
    struct LOCAL local;
};

struct BASE{
 int ID;
 struct POSITION pos;
 struct VELOCITY vel;
 struct ACCELERATION acc;
 // struct ORIENTATION ori;
};

struct KALMAN{
    double R;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::MatrixXd Q;
    Eigen::VectorXd H;
    Eigen::Vector2d x_hat;
    Eigen::Vector2d x_bar;
    Eigen::Matrix2d p_hat;
    Eigen::Matrix2d p_bar;
    Eigen::Vector2d gain;

    Eigen::Vector3d acc;
};

struct DATASAVE{
    double **save_array;
    int SAVE_LENGTH;    //The number of data
    int SAVE_TIME;
    int SAVE_COUNT;
    //double save_array[SAVE_COUNT][SAVE_LENGTH];
    unsigned int save_cnt;
};

struct JOY{
    double vel_x=0.0;
    double vel_y=0.0;
    double vel_yaw=0.0;
};

enum CONTROL_MODE {
    CTRLMODE_NONE = 0,
    CTRLMODE_INIT_POSE,
    CTRLMODE_WALK_READY,
    CTRLMODE_SLOW_WALK
};

enum COMMAND_FLAG {
    NO_ACT = 0,
    GOTO_INIT_POSE,
    GOTO_READY_POSE,
    GOTO_SLOW_WALK
};

class Common 
{
    public :
    Common();
    LEG RL,RR,FL,FR;
    BASE Base;
    COM CoM;
    RBDL rbdl;
    KALMAN kalman;
    FILEPATH filepath;
    JOY joy;
    DATASAVE datasave;
    enum CONTROL_MODE ControlMode;
    enum COMMAND_FLAG CommandFlag;

public:
    VectorNd GetComPos(VectorNd,MatrixNd);
    VectorNd GetBasePos(VectorNd,MatrixNd);
    VectorNd FK(VectorNd);
    MatrixNd skewMat(VectorNd);

    Vector3d offset_B2C=Vector3d::Zero();
    MatrixNd J_A=MatrixNd::Zero(12,12);

    void KalmanFilter(double);
    void setKalmanParam(void);
    
    void getRotationMatrix(void);
    void setPath(void);
    void setDataSave(void);
    void DataSave(void);


};

#endif


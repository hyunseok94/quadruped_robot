#ifndef QROBOT_H
#define QROBOT_H

#include "Eigen/Dense"
#include <iostream>
#include <unistd.h>
#include <string>
//#include <boost/filesystem.hpp>
//#include <filesystem>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "QuadProgpp/Array.hh"
#include "QuadProgpp/QuadProg++.hh"

#define nDOF 12
#define dt 0.001
#define PI 3.141592
#define R2D 180/PI
#define D2R PI/180
#define GRAVITY 9.81

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

enum CONTROL_MODE {
	CTRLMODE_NONE = 0,
	CTRLMODE_INIT_POSE,
	CTRLMODE_WALK_READY,
	CTRLMODE_SLOW_WALK
};

enum COMMAND_FLAG {
	NO_ACT = 0,
	GOTO_INIT_POSE,
	GOTO_WALK_READY,
	GOTO_SLOW_WALK
};

////////////////
struct POSITION{
 double prev;
 double now;

 double init;
 double ref;
 double goal;
};

struct POSITION3D{
 Eigen::Vector3d prev;
 Eigen::Vector3d now;
 Eigen::Vector3d now_from_com;

 Eigen::Vector3d pre_init;
 Eigen::Vector3d init;
 Eigen::Vector3d init_rot;
 Eigen::Vector3d ref;
 Eigen::Vector3d ref_from_com;
 Eigen::Vector3d goal;
};

struct VELOCITY{
 double prev;
 double now;

 double init;
 double ref;
 double goal;
};

struct VELOCITY3D{
 Eigen::Vector3d prev;
 Eigen::Vector3d now;

 Eigen::Vector3d init;
 Eigen::Vector3d ref;
 Eigen::Vector3d goal;
};

struct ACCELERATION{
 double prev;
 double now;

 double init;
 double ref;
 double goal;
};

struct ACCELERATION3D{
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

struct SEMI{
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
};

struct TORQUE{
 double now;
 double ref;
};

struct FORCE{
 double now;
 double ref;
};

struct LOCAL{
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
	struct SEMI semi;
};

struct GLOBAL{
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
};

struct EULER{
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
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

struct JOINT{
    struct TORQUE torque;
	struct POSITION pos;
	struct VELOCITY vel;
	struct ACCELERATION acc;
	double kp;
 	double kd;
};

struct BASE{
	int ID;
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
	// struct ORIENTATION ori;
};

struct COM{
	int ID;
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
	struct ORIENTATION ori;
	struct SEMI semi;
	struct LOCAL local;
};

struct QP{
	int n;
	int m;
	int p;

	quadprogpp::Matrix<double> G;
	quadprogpp::Matrix<double> CE;
	quadprogpp::Matrix<double> CI;
	quadprogpp::Vector<double> g0;
	quadprogpp::Vector<double> ce0;
	quadprogpp::Vector<double> ci0;
	quadprogpp::Vector<double> x;
	Eigen::VectorXd x_saved;

	RigidBodyDynamics::Math::MatrixNd A_qp;
	RigidBodyDynamics::Math::VectorNd b_qp;
	RigidBodyDynamics::Math::MatrixNd W;

	Eigen::MatrixXd tmp_G;
	Eigen::VectorXd tmp_g0;
	Eigen::MatrixXd tmp_CI;

	Eigen::VectorXd kp;
	Eigen::VectorXd kd;
	Eigen::MatrixXd kp_mat;
	Eigen::MatrixXd kd_mat;
	Eigen::VectorXd StateErr;
	Eigen::VectorXd StateDotErr;
	Eigen::VectorXd Inertia_term;

	Eigen::Vector3d b_local;
	Eigen::Vector3d t_local;
	Eigen::Vector3d n_local;

	Eigen::Vector3d b_semi_global;
	Eigen::Vector3d t_semi_global;
	Eigen::Vector3d n_semi_global;

	double mu;
};

struct GROUND_FORCE{
	struct QP qp;
	RigidBodyDynamics::Math::VectorNd value;
};

struct TASK_PD{
	RigidBodyDynamics::Math::VectorNd value;
};

struct JOINT_PD{
	RigidBodyDynamics::Math::VectorNd value;
};

struct CONTROLLER{
	struct GROUND_FORCE grf;
	struct TASK_PD task;
	struct JOINT_PD joint;
};

struct ENDPOINT{
	int ID;
	struct LOCAL local;
	struct GLOBAL global;
	Eigen::Matrix3d kp;
 	Eigen::Matrix3d kd;
 	Eigen::Matrix3d Jac;
 	unsigned int Contact;
};

struct COUNT{
	unsigned int now;
	unsigned int ref;
};

struct TIME{
	double now;
	double ref;
};

struct INIT{		
	struct COUNT cnt;
	struct TIME time;
};

struct READY{
	struct COUNT cnt;
	struct TIME time;
};

struct BEZIER{
	double b0, b1, b2, b3, b4, b5, b6;
	
	double b0_dot, b1_dot, b2_dot, b3_dot, b4_dot, b5_dot, b6_dot;
	Eigen::Vector3d P0;
	Eigen::Vector3d P1;
	Eigen::Vector3d P2;
	Eigen::Vector3d P3;
	Eigen::Vector3d P4;
	Eigen::Vector3d P5;
	Eigen::Vector3d P6;
	
	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
	struct COUNT cnt;
	struct TIME time;
	bool enable;
	double foot_height;
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

	RigidBodyDynamics::Math::MatrixNd X_new; // 3 by 2
	RigidBodyDynamics::Math::MatrixNd ref_array; //2 by 3000

	RigidBodyDynamics::Math::VectorNd sum_p; //2 by 1
	RigidBodyDynamics::Math::VectorNd sum_e; //2 by 1
	RigidBodyDynamics::Math::VectorNd err; //2 by 1

	RigidBodyDynamics::Math::VectorNd u; //2 by 1 

	RigidBodyDynamics::Math::MatrixNd AA; //3 by 3
	RigidBodyDynamics::Math::VectorNd BB; //3 by 1
	RigidBodyDynamics::Math::VectorNd CC; // 3 by 1

	RigidBodyDynamics::Math::VectorNd Gp; //3000 by 1
	RigidBodyDynamics::Math::VectorNd Gx; // 3 by 1
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

struct STANCE{
	struct COUNT cnt;
	struct TIME time;
};

struct FLY{
	struct COUNT cnt;
	struct TIME time;
};

struct WALK{
	struct COUNT cnt;
	struct TIME time;
	struct STANCE stance;
	struct STANCE fly;

	struct BEZIER bezier;
	struct ZMP_PREVIEW zmp;
	struct VELOCITY3D vel;
	struct STOP_FLAG stop;
	struct FOOTSTEP footstep;
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

struct RBDL{
	RigidBodyDynamics::Model* pModel;
	RigidBodyDynamics::Math::VectorNd RobotState;
    RigidBodyDynamics::Math::VectorNd RobotStateDot;
    RigidBodyDynamics::Math::VectorNd RobotState2Dot;
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

struct FILEPATH{
	string URDF;
	string PREVIEW_GAIN_Gp;
	string PREVIEW_GAIN_Gx;
	string PREVIEW_GAIN_Gi;
};

struct JOY{
	double vel_x=0.0;
	double vel_y=0.0;
	double vel_yaw=0.0;
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

class QRobot{
public:
	
	JOINT* joint;
	TRAJ Traj;
	RBDL rbdl;
	BASE Base;
	COM CoM;
	ENDPOINT RL,RR,FL,FR;
	FILEPATH filepath;
	JOY joy;
	KALMAN kalman;
	CONTROLLER controller;
	enum CONTROL_MODE ControlMode;
	enum COMMAND_FLAG CommandFlag;
	
	QRobot();
	~QRobot();
	void setParam(void);
	void setPath(void);
	void setPreviewParam(void);
	void resetTrajPram(void);
	void setRobotModel(Model* getModel);
	void setQPParam(void);
	void setKalmanParam(void);

	void ComputeTorqueControl(void);
	VectorNd Joint_PD_Controller(void);
	VectorNd Task_PD_Controller(void);
	// VectorNd QuadPP_Controller(void);
	void QuadPP_Controller(void);
	
	VectorNd FK(VectorNd);
	MatrixNd skewMat(VectorNd);
	void KalmanFilter(double);
	void StateUpdate(void);
	void getRotationMatrix(void);

	void Init_Pose_Traj(void);
	void WalkReady_Pose_Traj(void);
	void Slow_Walk_Traj(void);
	void Init_Slow_walk_traj(unsigned int);
	void FootStepPlanning(void);
	void Bezier_Traj(VectorNd, VectorNd);
	void getPreviewParam(void);
	void ZMP_PreviewControl(void);

	VectorNd getCOMpos(VectorNd,MatrixNd);
	VectorNd getBASEpos(VectorNd,MatrixNd);

	void BodyMove_Traj(void);
	void FootMove_Traj(unsigned int);

	VectorXd joint_control_value=VectorXd::Zero(nDOF);
	VectorXd Task_control_value=VectorXd::Zero(nDOF);
	VectorXd CTC_Torque=VectorXd::Zero(nDOF);
	MatrixNd J_A=MatrixNd::Zero(12,12);
	//MatrixNd M_term = MatrixNd::Zero(18, 18);
    VectorNd G_term = VectorNd::Zero(12);
    VectorNd C_term = VectorNd::Zero(12);
    Eigen::Vector3d offset_B2C=Eigen::Vector3d::Zero();
    double Robot_mass=50;
    double tmp_target_yaw=0.0;
private:
};
#endif 


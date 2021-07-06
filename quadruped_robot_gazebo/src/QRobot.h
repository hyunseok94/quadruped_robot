#ifndef QROBOT_H
#define QROBOT_H

#include "Eigen/Dense"
#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#define nDOF 12
#define dt 0.001
#define PI 3.141592
#define R2D 180/PI
#define D2R PI/180

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


enum CONTROL_MODE {
	CTRLMODE_NONE = 0,
	CTRLMODE_INIT_POSE,
	CTRLMODE_WALK_READY
};

enum COMMAND_FLAG {
	NO_ACT = 0,
	GOTO_INIT_POSE,
	GOTO_WALK_READY
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

 Eigen::Vector3d init;
 Eigen::Vector3d ref;
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
	struct ROTATION_MATRIX R;
};

struct QUATERNION{
	struct QUAT_POSITION pos;
	struct QUAT_VELOCITY vel;
	struct QUAT_ACCELERATION acc;
	struct ROTATION_MATRIX R;
};

struct ORIENTATION{
	struct EULER euler;
	struct QUATERNION quat;
};


struct TORQUE{
 double now;
 double ref;
};

struct FORCCE{
 double now;
 double ref;
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
	struct ORIENTATION ori;
};

struct COM{
	int ID;
	struct POSITION3D pos;
	struct VELOCITY3D vel;
	struct ACCELERATION3D acc;
	struct ORIENTATION ori;
};

struct EP{
	int ID;
	struct LOCAL local;
	struct GLOBAL global;
	Eigen::Matrix3d kp;
 	Eigen::Matrix3d kd;
 	Eigen::Matrix3d Jac;
};

struct INIT{		
	double cycle;
};

struct READY{
	double cycle;
};

struct TRAJ{
	unsigned int cnt;
	struct INIT init;
	struct READY ready;
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


class QRobot{
public:
	
	JOINT* joint;
	TRAJ Traj;
	RBDL rbdl;
	BASE Base;
	COM CoM;
	EP RL,RR,FL,FR;

	enum CONTROL_MODE ControlMode;
	enum COMMAND_FLAG CommandFlag;
	
	QRobot();
	~QRobot();
	void paramReset(void);
	void ComputeTorqueControl(void);
	VectorNd Joint_PD_Controller(void);
	VectorNd Task_PD_Controller(void);
	VectorNd FK(VectorNd);
	void ResetTraj(void);

	void setRobotModel(Model* getModel);
	void StateUpdate(void);
	void getRotationMatrix(void);

	void Init_Pose_Traj(void);
	void WalkReady_Pose_Traj(void);

	VectorNd getCOMpos(VectorNd,MatrixNd);
	VectorNd getBASEpos(VectorNd,MatrixNd);

	VectorXd joint_control_value=VectorXd::Zero(nDOF);
	VectorXd Task_control_value=VectorXd::Zero(nDOF);
	VectorXd CTC_Torque=VectorXd::Zero(nDOF);
	MatrixNd J_A=MatrixNd::Zero(12,12);
	//MatrixNd M_term = MatrixNd::Zero(18, 18);
    VectorNd G_term = VectorNd::Zero(12);
    VectorNd C_term = VectorNd::Zero(12);
    Eigen::Vector3d offset_B2C=Eigen::Vector3d::Zero();
private:
};
#endif 
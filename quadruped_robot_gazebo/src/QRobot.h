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
 double kp;
 double kd;
};

struct ROLL{
	double now;
	double ref;
	double goal;
};

struct PITCH{
	double now;
	double ref;
	double init;
	double goal;
};

struct YAW{
	double now;
	double ref;
	double init;
	double goal;
};

struct ORIENTATION{
	struct ROLL roll;
	struct PITCH pitch;
	struct YAW yaw;
	Quaternion quat_ref;
	Quaternion quat_now;
};

struct VELOCITY{
 double prev;
 double now;

 double init;
 double ref;
 double goal;
 double kp;
 double kd;
};

struct ACCELERATION{
 double prev;
 double now;

 double init;
 double ref;
 double goal;
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
};

struct GAIN{
	double kp;
	double kd;
	double kp_EP;
	double kd_EP;
};

struct BASE{
	int ID;
	struct POSITION pos;
	struct VELOCITY vel;
	struct ACCELERATION acc;
	struct ORIENTATION ori;
};

struct EP{
	int ID;
	struct POSITION pos;
	struct VELOCITY vel;
	struct ACCELERATION acc;
};

struct TRAJ{
	unsigned int cnt;
	double cycle;
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

};

class QRobot{
public:
	
	JOINT* joint;
	GAIN* gain; 
	TRAJ Traj;
	RBDL rbdl;
	BASE Base;
	EP RL,RR,FL,FR;

	enum CONTROL_MODE ControlMode;
	enum COMMAND_FLAG CommandFlag;
	
	QRobot();
	~QRobot();
	void paramReset(void);
	void ComputeTorqueControl(void);
	VectorXd Joint_PD_Controller(void);
	void ResetTraj(void);
	void Init_Pose_Traj(void);
	
	void setRobotModel(Model* getModel);
	void StateUpdate(void);

	VectorXd joint_control_value=VectorXd::Zero(nDOF);
	VectorXd CTC_Torque=VectorXd::Zero(nDOF);
	MatrixNd J_A=MatrixNd::Zero(12,12);
	//MatrixNd M_term = MatrixNd::Zero(18, 18);
    VectorNd G_term = VectorNd::Zero(12);
    VectorNd C_term = VectorNd::Zero(12);
private:
};
#endif 
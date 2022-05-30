#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"
#include "QuadProgpp/Array.hh"
#include "QuadProgpp/QuadProg++.hh"

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

	int n2;
	int m2;
	int p2;

	quadprogpp::Matrix<double> G2;
	quadprogpp::Matrix<double> CE2;
	quadprogpp::Matrix<double> CI2;
	quadprogpp::Vector<double> g02;
	quadprogpp::Vector<double> ce02;
	quadprogpp::Vector<double> ci02;
	quadprogpp::Vector<double> x2;
	Eigen::VectorXd x_saved2;

	RigidBodyDynamics::Math::MatrixNd A_qp2;
	RigidBodyDynamics::Math::VectorNd b_qp2;
	RigidBodyDynamics::Math::MatrixNd W2;

	Eigen::MatrixXd tmp_G2;
	Eigen::VectorXd tmp_g02;
	Eigen::MatrixXd tmp_CI2;

	Eigen::VectorXd kp2;
	Eigen::VectorXd kd2;
	Eigen::MatrixXd kp_mat2;
	Eigen::MatrixXd kd_mat2;
	Eigen::VectorXd StateErr2;
	Eigen::VectorXd StateDotErr2;
	Eigen::VectorXd Inertia_term2;

};

struct GROUND_FORCE{
	struct QP qp;
	RigidBodyDynamics::Math::VectorNd value;
	RigidBodyDynamics::Math::VectorNd value2;
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


class QRobot;

class Controller : virtual public Common {
public:
	Controller();

public:
	CONTROLLER controller;
	
	void setParam();
	void setQPParam(void);

	VectorNd Joint_PD_Controller(void);
	VectorNd Task_PD_Controller(void);
	VectorNd Compensation_Controller(void);
	

	void TorqueControl(void);
	
	void QuadPP_Controller_Stance(void);	
	void QuadPP_Controller_Fly(void);
	VectorNd CTC_Torque=VectorNd::Zero(nDOF);
	VectorNd G_term = VectorNd::Zero(12);
	VectorNd C_term = VectorNd::Zero(12);
};

#endif
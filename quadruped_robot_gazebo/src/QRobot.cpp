#include <stdio.h>
#include <math.h>
#include "QRobot.h"

QRobot::QRobot() {
	joint = new JOINT[nDOF]; 
	gain = new GAIN[nDOF]; 
	
	//ControlMode=CTRLMODE_NONE;
	ControlMode=CTRLMODE_INIT_POSE;
	paramReset();
}

QRobot::~QRobot() {
	delete[] joint;
	delete[] gain;
}

void QRobot::paramReset(void){
	for (int i=0;i<nDOF;i++){
		joint[i].pos.now=0.0;
		joint[i].pos.prev=0.0;
		joint[i].vel.now=0.0;
		joint[i].vel.prev=0.0;
		joint[i].torque.ref=0.0;	
		joint[i].pos.ref=0.0;
		joint[i].vel.ref=0.0;
	}

	for (int i=0; i<4; i++){
		gain[3*i+0].kp=400;
		gain[3*i+1].kp=100;
		gain[3*i+2].kp=100;

		gain[3*i+0].kd=15;
		gain[3*i+1].kd=5;
		gain[3*i+2].kd=5;
	}
}


void QRobot::ComputeTorqueControl(void){
	MatrixNd tmp_J_BASE = MatrixNd::Zero(6, 19);
	MatrixNd tmp_J_RL = MatrixNd::Zero(3, 18);
	MatrixNd tmp_J_RR = MatrixNd::Zero(3, 18);
	MatrixNd tmp_J_FL = MatrixNd::Zero(3, 18);
	MatrixNd tmp_J_FR = MatrixNd::Zero(3, 18);
	VectorNd tmp_hatNonLinearEffects = VectorNd::Zero(18);
    VectorNd tmp_G_term = VectorNd::Zero(18);
    VectorNd tmp_C_term = VectorNd::Zero(18);
	
	CalcPointJacobian6D(*rbdl.pModel, rbdl.RobotState, Base.ID, rbdl.BaseState.segment(0,3), tmp_J_BASE, true);	
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, RL.ID, Eigen::Vector3d::Zero(), tmp_J_RL, true);
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, RR.ID, Eigen::Vector3d::Zero(), tmp_J_RR, true);
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, FL.ID, Eigen::Vector3d::Zero(), tmp_J_FL, true);
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, FR.ID, Eigen::Vector3d::Zero(), tmp_J_FR, true);

    J_A.block(0,0,3,3)=tmp_J_RL.block(0,6,3,3);
    J_A.block(3,3,3,3)=tmp_J_RR.block(0,9,3,3);
    J_A.block(6,6,3,3)=tmp_J_FL.block(0,12,3,3);
    J_A.block(9,9,3,3)=tmp_J_FR.block(0,15,3,3);
    


  	//CompositeRigidBodyAlgorithm(*rbdl.pModel, rbdl.RobotState, M_term, true);
    NonlinearEffects(*rbdl.pModel, rbdl.RobotState, rbdl.RobotStateDot, tmp_hatNonLinearEffects);
    NonlinearEffects(*rbdl.pModel, rbdl.RobotState, VectorNd::Zero(rbdl.pModel->dof_count), tmp_G_term);
    tmp_C_term = tmp_hatNonLinearEffects - tmp_G_term;

    C_term=tmp_C_term.segment(6,12);
    G_term=tmp_G_term.segment(6,12);


	joint_control_value=Joint_PD_Controller();
	CTC_Torque=G_term+C_term+joint_control_value;
	//cout<<"CTC:"<<CTC_Torque.transpose()<<endl;
	//cout<<"---"<<endl;
	for (int i = 0; i < nDOF; i++) {
		//joint[i].torque.ref = CTC_Torque(6 + i);
		joint[i].torque.ref = CTC_Torque(i);
	}
}


VectorXd QRobot::Joint_PD_Controller(void){
	VectorXd _CTC_Torque(12);
	for(int i=0; i<nDOF; i++){
		_CTC_Torque(i)=gain[i].kp*(joint[i].pos.ref-joint[i].pos.now)+gain[i].kd*(joint[i].vel.ref-joint[i].vel.now);	
	}

	return _CTC_Torque;
}

void QRobot::ResetTraj(void){
	Traj.cnt=0;
	Traj.cycle=5.0;
}

void QRobot::Init_Pose_Traj(void){
	if(Traj.cnt==0){
		for(int i=0;i<nDOF;i++){
			joint[i].pos.init=joint[i].pos.now;
			joint[i].pos.ref=joint[i].pos.init;
			joint[i].vel.ref=0.0;
		}
		for (int i=0; i<4; i++){
			joint[3*i+0].pos.goal=0.0*D2R;
			joint[3*i+1].pos.goal=45.0*D2R;
			joint[3*i+2].pos.goal=-90.0*D2R;
		}			
		Traj.cnt++;
	}else if(Traj.cnt<Traj.cycle/dt){
		for (int i=0;i<nDOF;i++){
			joint[i].pos.ref=joint[i].pos.init+(joint[i].pos.goal-joint[i].pos.init)/2.0*(1.0-cos(PI/Traj.cycle*Traj.cnt*dt));
			joint[i].vel.ref=PI/Traj.cycle*(joint[i].pos.goal-joint[i].pos.init)/2.0*sin(PI/Traj.cycle*Traj.cnt*dt);
		}
		Traj.cnt++;
	}else{
		for (int i=0;i<nDOF;i++){
			joint[i].pos.ref=joint[i].pos.goal;
			joint[i].vel.ref=0.0;
		}
	}
	//printf("%d, %d, %d\n",joint[0].pos.ref,joint[1].pos.ref,joint[2].pos.ref);
	// cout<<joint[0].pos.ref<<endl;
	// cout<<joint[1].pos.ref<<endl;
	// cout<<joint[2].pos.ref<<endl;
	// cout<<"---------------"<<endl;
}

void QRobot::setRobotModel(Model* getModel){
	double tmp_nDOF;
	rbdl.pModel = getModel;
	rbdl.pModel->gravity=Eigen::Vector3d(0.0,0.0,-9.81);
	tmp_nDOF=rbdl.pModel->dof_count-6;
	//cout<<tmp_nDOF<<endl;

	Base.ID=rbdl.pModel->GetBodyId("BODY");
	RL.ID=rbdl.pModel->GetBodyId("RL_TIP");
	RR.ID=rbdl.pModel->GetBodyId("RR_TIP");
	FL.ID=rbdl.pModel->GetBodyId("FL_TIP");
	FR.ID=rbdl.pModel->GetBodyId("FR_TIP");

	rbdl.RobotState=VectorNd::Zero(nDOF+7);
	rbdl.RobotStateDot=VectorNd::Zero(nDOF+6);
	rbdl.RobotState2Dot=VectorNd::Zero(nDOF+6);

	rbdl.BaseState=VectorNd::Zero(6);
	rbdl.BaseStateDot=VectorNd::Zero(6);
	rbdl.BaseState2Dot=VectorNd::Zero(6);

	rbdl.JointState=VectorNd::Zero(nDOF);
	rbdl.JointStateDot=VectorNd::Zero(nDOF);
	rbdl.JointState2Dot=VectorNd::Zero(nDOF);

	Base.ori.quat_ref<<0.0,0.0,0.0,1.0;
	rbdl.pModel->SetQuaternion(Base.ID,Base.ori.quat_ref,rbdl.RobotState);

	cout << endl << "Set Robot Model End !!" << endl;
}

void QRobot::StateUpdate(void){

	rbdl.BaseState(0)=0.0;
	rbdl.BaseState(1)=0.0;
	rbdl.BaseState(2)=0.0;
	rbdl.BaseState(3)=0.0;
	rbdl.BaseState(4)=0.0;
	rbdl.BaseState(5)=0.0;

	rbdl.BaseStateDot(0)=0.0;
	rbdl.BaseStateDot(1)=0.0;
	rbdl.BaseStateDot(2)=0.0;
	rbdl.BaseStateDot(3)=0.0;
	rbdl.BaseStateDot(4)=0.0;
	rbdl.BaseStateDot(5)=0.0;

	rbdl.BaseState2Dot(0)=0.0;
	rbdl.BaseState2Dot(1)=0.0;
	rbdl.BaseState2Dot(2)=0.0;
	rbdl.BaseState2Dot(3)=0.0;
	rbdl.BaseState2Dot(4)=0.0;
	rbdl.BaseState2Dot(5)=0.0;
	
	cout<<"Pos:"<<rbdl.BaseState.segment(0,3)<<endl;
	cout<<"Ori:"<<rbdl.BaseState.segment(3,3)<<endl;
	cout<<"----"<<endl;

	for (int i=0;i<12;i++){
		rbdl.JointState(i)=joint[i].pos.now;
		rbdl.JointStateDot(i)=joint[i].vel.now;
		rbdl.JointState2Dot(i)=joint[i].acc.now;
	}

	for(int i=0;i<6;i++){
		rbdl.RobotState(i)=rbdl.BaseState(i);
		rbdl.RobotStateDot(i)=rbdl.BaseStateDot(i);
		rbdl.RobotState2Dot(i)=rbdl.BaseState2Dot(i);
	}
	for(int i=0;i<12;i++){
		rbdl.RobotState(i+6)=rbdl.JointState(i);	
		rbdl.RobotStateDot(i+6)=rbdl.JointStateDot(i);	
		rbdl.RobotState2Dot(i+6)=rbdl.JointState2Dot(i);	
	}
	
	Base.ori.quat_ref<<0.0,0.0,0.0,1.0;
	rbdl.pModel->SetQuaternion(Base.ID,Base.ori.quat_ref,rbdl.RobotState);
}
#include <stdio.h>
#include <math.h>
#include "QRobot.h"

QRobot::QRobot() {
	joint = new JOINT[nDOF]; 
	
	//ControlMode=CTRLMODE_NONE;
	ControlMode=CTRLMODE_INIT_POSE;
	//ControlMode=CTRLMODE_WALK_READY;

	paramReset();
}

QRobot::~QRobot() {
	delete[] joint;
}

void QRobot::paramReset(void){
	
	offset_B2C<<-0.05, 0.0, -0.0;

	for (int i=0;i<nDOF;i++){
		joint[i].pos.now=0.0;
		joint[i].pos.prev=0.0;
		joint[i].vel.now=0.0;
		joint[i].vel.prev=0.0;
		joint[i].torque.ref=0.0;	
		joint[i].pos.ref=0.0;
		joint[i].vel.ref=0.0;
	}

	Base.pos.ref<<0.0, 0.0, 0.45;
	CoM.pos.ref<<0.0, 0.0, 0.45;

	CoM.ori.euler.pos.ref<<0.,0.,0.;
	CoM.ori.euler.pos.now<<0.,0.,0.;
	CoM.ori.euler.vel.now<<0.,0.,0.;

	for (int i=0; i<4; i++){
		joint[3*i+0].kp=400;
		joint[3*i+1].kp=200;
		joint[3*i+2].kp=200;

		joint[3*i+0].kd=15;
		joint[3*i+1].kd=10;
		joint[3*i+2].kd=10;
	}

	RL.kp<<2000,0,0\
		   ,0,2000,0\
		   ,0,0,1000;
	RL.kd<<70,0,0\
		   ,0,70,0\
		   ,0,0,50;

	RR.kp<<2000,0,0\
		   ,0,2000,0\
		   ,0,0,1000;
	RR.kd<<70,0,0\
		   ,0,70,0\
		   ,0,0,50;

	FL.kp<<2000,0,0\
		   ,0,2000,0\
		   ,0,0,1000;
	FL.kd<<70,0,0\
		   ,0,70,0\
		   ,0,0,50;

	FR.kp<<2000,0,0\
		   ,0,2000,0\
		   ,0,0,1000;
	FR.kd<<70,0,0\
		   ,0,70,0\
		   ,0,0,50;
	Traj.move_done_flag=true;
	//Traj.move_stop_flag=false;
}


void QRobot::ComputeTorqueControl(void){
	VectorNd tmp_hatNonLinearEffects = VectorNd::Zero(18);
    VectorNd tmp_G_term = VectorNd::Zero(18);
    VectorNd tmp_C_term = VectorNd::Zero(18);

    NonlinearEffects(*rbdl.pModel, rbdl.RobotState, rbdl.RobotStateDot, tmp_hatNonLinearEffects);
    NonlinearEffects(*rbdl.pModel, rbdl.RobotState, VectorNd::Zero(rbdl.pModel->dof_count), tmp_G_term);
    tmp_C_term = tmp_hatNonLinearEffects - tmp_G_term;

    C_term=tmp_C_term.segment(6,12);
    G_term=tmp_G_term.segment(6,12);

    if(CommandFlag==GOTO_INIT_POSE){
    	joint_control_value=Joint_PD_Controller();	
    	Task_control_value=VectorNd::Zero(12);
    }else{
    	Task_control_value=Task_PD_Controller();	
    	joint_control_value=VectorNd::Zero(12);
    }
    CTC_Torque=G_term+C_term+joint_control_value+J_A.transpose()*(Task_control_value);
	//cout<<"CTC:"<<CTC_Torque.transpose()<<endl;
	//cout<<"---"<<endl;
	for (int i = 0; i < nDOF; i++) {
		joint[i].torque.ref = CTC_Torque(i);
	}
}

void QRobot::ResetTraj(void){
	Traj.cnt=0;
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

	rbdl.EPState=VectorNd::Zero(12);
	rbdl.EPStateDot=VectorNd::Zero(12);
	rbdl.EPState2Dot=VectorNd::Zero(12);

	rbdl.JointState_ref=VectorNd::Zero(nDOF);
	rbdl.JointStateDot_ref=VectorNd::Zero(nDOF);
	rbdl.JointState2Dot_ref=VectorNd::Zero(nDOF);
	rbdl.EPState_ref=VectorNd::Zero(12);
	rbdl.EPStateDot_ref=VectorNd::Zero(12);
	rbdl.EPState2Dot_ref=VectorNd::Zero(12);

	Base.ori.quat.pos.ref<<0.,0.,0.,1.;
	rbdl.pModel->SetQuaternion(Base.ID,Base.ori.quat.pos.ref,rbdl.RobotState);

	cout << endl << "Set Robot Model End !!" << endl;
}

void QRobot::StateUpdate(void){
	MatrixNd tmp_J_BASE = MatrixNd::Zero(6, 19);
	MatrixNd tmp_J_RL = MatrixNd::Zero(3, 18);
	MatrixNd tmp_J_RR = MatrixNd::Zero(3, 18);
	MatrixNd tmp_J_FL = MatrixNd::Zero(3, 18);
	MatrixNd tmp_J_FR = MatrixNd::Zero(3, 18);

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
	
	Base.ori.quat.pos.ref<<0.,0.,0.,1.;
	rbdl.pModel->SetQuaternion(Base.ID,Base.ori.quat.pos.ref,rbdl.RobotState);
	
	CalcPointJacobian6D(*rbdl.pModel, rbdl.RobotState, Base.ID, rbdl.BaseState.segment(0,3), tmp_J_BASE, true);	
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, RL.ID, Eigen::Vector3d::Zero(), tmp_J_RL, true);
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, RR.ID, Eigen::Vector3d::Zero(), tmp_J_RR, true);
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, FL.ID, Eigen::Vector3d::Zero(), tmp_J_FL, true);
    CalcPointJacobian(*rbdl.pModel, rbdl.RobotState, FR.ID, Eigen::Vector3d::Zero(), tmp_J_FR, true);

    RL.Jac=tmp_J_RL.block(0,6,3,3);
    RR.Jac=tmp_J_RR.block(0,9,3,3);
    FL.Jac=tmp_J_FL.block(0,12,3,3);
    FR.Jac=tmp_J_FR.block(0,15,3,3);

    J_A.block(0,0,3,3)=RL.Jac;
    J_A.block(3,3,3,3)=RR.Jac;
    J_A.block(6,6,3,3)=FL.Jac;
    J_A.block(9,9,3,3)=FR.Jac;

	rbdl.EPState=FK(rbdl.JointState);
	rbdl.EPStateDot=J_A*rbdl.JointStateDot;

	RL.local.pos.now=rbdl.EPState.segment(0,3);
	RR.local.pos.now=rbdl.EPState.segment(3,3);
	FL.local.pos.now=rbdl.EPState.segment(6,3);
	FR.local.pos.now=rbdl.EPState.segment(9,3);

	RL.local.vel.now=rbdl.EPStateDot.segment(0,3);
	RR.local.vel.now=rbdl.EPStateDot.segment(3,3);
	FL.local.vel.now=rbdl.EPStateDot.segment(6,3);
	FR.local.vel.now=rbdl.EPStateDot.segment(9,3);	
}

VectorNd QRobot::Joint_PD_Controller(void){
	VectorNd Joint_PD(12);
	for(int i=0; i<nDOF; i++){
		Joint_PD(i)=joint[i].kp*(joint[i].pos.ref-joint[i].pos.now)+joint[i].kd*(joint[i].vel.ref-joint[i].vel.now);	
	}

	return Joint_PD;
}

VectorNd QRobot::Task_PD_Controller(void){
	VectorNd Task_PD=VectorNd::Zero(12);

	Matrix3d I=Matrix3d::Ones();
	VectorNd tmp_RL_F=VectorNd::Zero(3);
	VectorNd tmp_RR_F=VectorNd::Zero(3);
	VectorNd tmp_FL_F=VectorNd::Zero(3);
	VectorNd tmp_FR_F=VectorNd::Zero(3);

	VectorNd tmp_RL_F2=VectorNd::Zero(3);
	VectorNd tmp_RR_F2=VectorNd::Zero(3);
	VectorNd tmp_FL_F2=VectorNd::Zero(3);
	VectorNd tmp_FR_F2=VectorNd::Zero(3);

	Base.pos.ref=getBASEpos(CoM.pos.ref, CoM.ori.euler.R.ref);
	Base.vel.ref=CoM.vel.ref;

	/************************************************************/
	RL.local.semi.pos.now=CoM.ori.euler.R.now*RL.local.pos.now;
	RR.local.semi.pos.now=CoM.ori.euler.R.now*RR.local.pos.now;
	FL.local.semi.pos.now=CoM.ori.euler.R.now*FL.local.pos.now;
	FR.local.semi.pos.now=CoM.ori.euler.R.now*FR.local.pos.now;

	RL.local.semi.vel.now=CoM.ori.euler.R.now*RL.local.vel.now;
	RR.local.semi.vel.now=CoM.ori.euler.R.now*RR.local.vel.now;
	FL.local.semi.vel.now=CoM.ori.euler.R.now*FL.local.vel.now;
	FR.local.semi.vel.now=CoM.ori.euler.R.now*FR.local.vel.now;

	/************************************************************/
	RL.local.semi.pos.ref=RL.global.pos.ref-Base.pos.ref;
	RR.local.semi.pos.ref=RR.global.pos.ref-Base.pos.ref;
	FL.local.semi.pos.ref=FL.global.pos.ref-Base.pos.ref;
	FR.local.semi.pos.ref=FR.global.pos.ref-Base.pos.ref;

	RL.local.semi.vel.ref=RL.global.vel.ref-Base.vel.ref;
	RR.local.semi.vel.ref=RR.global.vel.ref-Base.vel.ref;
	FL.local.semi.vel.ref=FL.global.vel.ref-Base.vel.ref;
	FR.local.semi.vel.ref=FR.global.vel.ref-Base.vel.ref;

	/************************************************************/
	tmp_RL_F=CoM.ori.euler.R.ref.transpose()*(RL.kp*(RL.local.semi.pos.ref-RL.local.semi.pos.now)+RL.kd*(RL.local.semi.vel.ref-RL.local.semi.vel.now));
	tmp_RR_F=CoM.ori.euler.R.ref.transpose()*(RR.kp*(RR.local.semi.pos.ref-RR.local.semi.pos.now)+RR.kd*(RR.local.semi.vel.ref-RR.local.semi.vel.now));
	tmp_FL_F=CoM.ori.euler.R.ref.transpose()*(FL.kp*(FL.local.semi.pos.ref-FL.local.semi.pos.now)+FL.kd*(FL.local.semi.vel.ref-FL.local.semi.vel.now));
	tmp_FR_F=CoM.ori.euler.R.ref.transpose()*(FR.kp*(FR.local.semi.pos.ref-FR.local.semi.pos.now)+FR.kd*(FR.local.semi.vel.ref-FR.local.semi.vel.now));
    // tmp_RL_F=RL.kp*(RL.local.pos.ref-RL.local.pos.now)+RL.kd*(RL.local.vel.ref-RL.local.vel.now);
    // tmp_RR_F=RR.kp*(RR.local.pos.ref-RR.local.pos.now)+RR.kd*(RR.local.vel.ref-RR.local.vel.now);
    // tmp_FL_F=FL.kp*(FL.local.pos.ref-FL.local.pos.now)+FL.kd*(FL.local.vel.ref-FL.local.vel.now);
    // tmp_FR_F=FR.kp*(FR.local.pos.ref-FR.local.pos.now)+FR.kd*(FR.local.vel.ref-FR.local.vel.now);
	

    // cout<<tmp_RL_F.transpose()<<endl;
    // cout<<tmp_RR_F.transpose()<<endl;
    // cout<<tmp_FL_F.transpose()<<endl;
    // cout<<tmp_FR_F.transpose()<<endl;
    // cout<<"----"<<endl;
	Task_PD<<tmp_RL_F,tmp_RR_F,tmp_FL_F,tmp_FR_F;

	return Task_PD;
}

VectorNd QRobot::FK(VectorNd _JointState){
	VectorNd _EP_State(12);
	const double L1=0.1045;
	const double L2=0.305;
	const double L3=0.309;

	double q1=0.0;
	double q2=0.0;
	double q3=0.0;
	Eigen::Vector3d Offset_Base2Hip;
	Offset_Base2Hip <<0.35,0.115,-0.053;

	q1=_JointState[0];
	q2=_JointState[1];
	q3=_JointState[2];

	_EP_State[0] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	_EP_State[1] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	_EP_State[2] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	_EP_State[0]=_EP_State[0]-Offset_Base2Hip(0);
	_EP_State[1]=_EP_State[1]+Offset_Base2Hip(1);
	_EP_State[2]=_EP_State[2]+Offset_Base2Hip(2);

    //RR_EP
	q1 = _JointState[3];
	q2 = _JointState[4];
	q3 = _JointState[5];
	_EP_State[3] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	_EP_State[4] = -L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	_EP_State[5] = -L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	_EP_State[3]=_EP_State[3]-Offset_Base2Hip(0);
	_EP_State[4]=_EP_State[4]-Offset_Base2Hip(1);
	_EP_State[5]=_EP_State[5]+Offset_Base2Hip(2);

    //FL_EP
	q1 = _JointState[6];
	q2 = _JointState[7];
	q3 = _JointState[8];
	_EP_State[6] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	_EP_State[7] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	_EP_State[8] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	_EP_State[6]=_EP_State[6]+Offset_Base2Hip(0);
	_EP_State[7]=_EP_State[7]+Offset_Base2Hip(1);
	_EP_State[8]=_EP_State[8]+Offset_Base2Hip(2);

    //FR_EP
	q1 = _JointState[9];
	q2 = _JointState[10];
	q3 = _JointState[11];
	_EP_State[9] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	_EP_State[10] = -L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	_EP_State[11] = -L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	_EP_State[9]=_EP_State[9]+Offset_Base2Hip(0);
	_EP_State[10]=_EP_State[10]-Offset_Base2Hip(1);
	_EP_State[11]=_EP_State[11]+Offset_Base2Hip(2);

    return _EP_State;
}

void QRobot::Init_Pose_Traj(void){
	Traj.init.cycle=2.0;
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
	}else if(Traj.cnt<Traj.init.cycle/dt){
		for (int i=0;i<nDOF;i++){
			joint[i].pos.ref=joint[i].pos.init+(joint[i].pos.goal-joint[i].pos.init)/2.0*(1.0-cos(PI/Traj.init.cycle*Traj.cnt*dt));
			joint[i].vel.ref=PI/Traj.init.cycle*(joint[i].pos.goal-joint[i].pos.init)/2.0*sin(PI/Traj.init.cycle*Traj.cnt*dt);
		}
		Traj.cnt++;
	}else{
		for (int i=0;i<nDOF;i++){
			joint[i].pos.ref=joint[i].pos.goal;
			joint[i].vel.ref=0.0;
		}
	}
}

// void QRobot::WalkReady_Pose_Traj(void){
// 	Traj.ready.cycle=2.0;
// 	if(Traj.cnt==0){
// 			RL.local.pos.init=RL.local.pos.now;
// 			RR.local.pos.init=RR.local.pos.now;
// 			FL.local.pos.init=FL.local.pos.now;
// 			FR.local.pos.init=FR.local.pos.now;

// 			RL.local.pos.goal<<-0.35,0.22,-Base.pos.ref(2);
// 			RR.local.pos.goal<<-0.35,-0.22,-Base.pos.ref(2);
// 			FL.local.pos.goal<<0.35,0.22,-Base.pos.ref(2);
// 			FR.local.pos.goal<<0.35,-0.22,-Base.pos.ref(2);

// 			RL.local.pos.ref=RL.local.pos.init;
// 			RR.local.pos.ref=RR.local.pos.init;
// 			FL.local.pos.ref=FL.local.pos.init;
// 			FR.local.pos.ref=FR.local.pos.init;
// 		Traj.cnt++;
// 	}else if(Traj.cnt<Traj.ready.cycle/dt){
// 			RL.local.pos.ref=RL.local.pos.init+(RL.local.pos.goal-RL.local.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
// 			RR.local.pos.ref=RR.local.pos.init+(RR.local.pos.goal-RR.local.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
// 			FL.local.pos.ref=FL.local.pos.init+(FL.local.pos.goal-FL.local.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
// 			FR.local.pos.ref=FR.local.pos.init+(FR.local.pos.goal-FR.local.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
			
// 			RL.local.vel.ref=PI/Traj.ready.cycle*(RL.local.pos.goal-RL.local.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
// 			RR.local.vel.ref=PI/Traj.ready.cycle*(RR.local.pos.goal-RR.local.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
// 			FL.local.vel.ref=PI/Traj.ready.cycle*(FL.local.pos.goal-FL.local.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
// 			FR.local.vel.ref=PI/Traj.ready.cycle*(FR.local.pos.goal-FR.local.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));

// 		Traj.cnt++;
// 	}else{
// 			RL.local.pos.ref=RL.local.pos.goal;
// 			RR.local.pos.ref=RR.local.pos.goal;
// 			FL.local.pos.ref=FL.local.pos.goal;
// 			FR.local.pos.ref=FR.local.pos.goal;
			
// 			RL.local.vel.ref<<0.,0.,0.;
// 			RR.local.vel.ref<<0.,0.,0.;
// 			FL.local.vel.ref<<0.,0.,0.;
// 			FR.local.vel.ref<<0.,0.,0.;
// 	}

// 	// if(Traj.cnt<=Traj.ready.cycle/dt){
// 	// 	cout<<"RL:"<<RL.local.pos.ref.transpose()<<endl;
// 	// 	cout<<"RR:"<<RR.local.pos.ref.transpose()<<endl;
// 	// 	cout<<"FL:"<<FL.local.pos.ref.transpose()<<endl;
// 	// 	cout<<"FR:"<<FR.local.pos.ref.transpose()<<endl;
// 	// 	cout<<"-----"<<endl;	
// 	// }
// }


void QRobot::WalkReady_Pose_Traj(void){
	Traj.ready.cycle=2.0;
	if(Traj.cnt==0){
		CoM.pos.init=-(RL.local.pos.now+RR.local.pos.now+FL.local.pos.now+FR.local.pos.now)/4.0;
		CoM.pos.goal<<0.0,0.0,0.45;
		CoM.pos.ref=CoM.pos.init;
		Base.pos.init=getBASEpos(CoM.pos.init, CoM.ori.euler.R.now);
		CoM.vel.ref<<0.0,0.0,0.0;

		//*******************************************************************//
		RL.global.pos.init=Base.pos.init+CoM.ori.euler.R.now*RL.local.pos.now;
		RR.global.pos.init=Base.pos.init+CoM.ori.euler.R.now*RR.local.pos.now;
		FL.global.pos.init=Base.pos.init+CoM.ori.euler.R.now*FL.local.pos.now;
		FR.global.pos.init=Base.pos.init+CoM.ori.euler.R.now*FR.local.pos.now;

		RL.global.pos.goal<<-0.35,0.22,0.0;
		RR.global.pos.goal<<-0.35,-0.22,0.0;
		FL.global.pos.goal<<0.35,0.22,0.0;
		FR.global.pos.goal<<0.35,-0.22,0.0;

		RL.global.pos.ref=RL.global.pos.init;
		RR.global.pos.ref=RR.global.pos.init;
		FL.global.pos.ref=FL.global.pos.init;
		FR.global.pos.ref=FR.global.pos.init;

		RL.global.vel.ref<<0.,0.,0.;
		RR.global.vel.ref<<0.,0.,0.;
		FL.global.vel.ref<<0.,0.,0.;
		FR.global.vel.ref<<0.,0.,0.;

		Traj.cnt++;
	}else if(Traj.cnt<Traj.ready.cycle/dt){
		CoM.pos.ref=CoM.pos.init+(CoM.pos.goal-CoM.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
		CoM.vel.ref=PI/Traj.ready.cycle*(CoM.pos.goal-CoM.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));

		RL.global.pos.ref=RL.global.pos.init+(RL.global.pos.goal-RL.global.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
		RR.global.pos.ref=RR.global.pos.init+(RR.global.pos.goal-RR.global.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
		FL.global.pos.ref=FL.global.pos.init+(FL.global.pos.goal-FL.global.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));
		FR.global.pos.ref=FR.global.pos.init+(FR.global.pos.goal-FR.global.pos.init)/2.0*(1-cos(PI/Traj.ready.cycle*Traj.cnt*dt));

		RL.global.vel.ref=PI/Traj.ready.cycle*(RL.global.pos.goal-RL.global.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
		RR.global.vel.ref=PI/Traj.ready.cycle*(RR.global.pos.goal-RR.global.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
		FL.global.vel.ref=PI/Traj.ready.cycle*(FL.global.pos.goal-FL.global.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
		FR.global.vel.ref=PI/Traj.ready.cycle*(FR.global.pos.goal-FR.global.pos.init)/2.0*(sin(PI/Traj.ready.cycle*Traj.cnt*dt));
		Traj.cnt++;
	}else{
			CoM.pos.ref=CoM.pos.goal;
			CoM.vel.ref<<0.0,0.0,0.0;

			RL.global.pos.ref=RL.global.pos.goal;
			RR.global.pos.ref=RR.global.pos.goal;
			FL.global.pos.ref=FL.global.pos.goal;
			FR.global.pos.ref=FR.global.pos.goal;
			
			RL.global.vel.ref<<0.,0.,0.;
			RR.global.vel.ref<<0.,0.,0.;
			FL.global.vel.ref<<0.,0.,0.;
			FR.global.vel.ref<<0.,0.,0.;

			Traj.move_done_flag=true;
	}

	if(Traj.cnt<=Traj.ready.cycle/dt){
		cout<<"CoM:"<<CoM.pos.ref.transpose()<<endl;
		cout<<"RL:"<<RL.global.pos.ref.transpose()<<endl;
		cout<<"RR:"<<RR.global.pos.ref.transpose()<<endl;
		cout<<"FL:"<<FL.global.pos.ref.transpose()<<endl;
		cout<<"FR:"<<FR.global.pos.ref.transpose()<<endl;
		cout<<"-----"<<endl;	
	}
}


void QRobot::getRotationMatrix(void){
	CoM.ori.euler.R.roll.now<<1,0,0\
    							,0,cos(CoM.ori.euler.pos.now(0)),-sin(CoM.ori.euler.pos.now(0))\
    							,0,sin(CoM.ori.euler.pos.now(0)),cos(CoM.ori.euler.pos.now(0));

	CoM.ori.euler.R.pitch.now<<cos(CoM.ori.euler.pos.now(1)),0,sin(CoM.ori.euler.pos.now(1))\
    							,0,1,0\
    							,-sin(CoM.ori.euler.pos.now(1)),0,cos(CoM.ori.euler.pos.now(1));

   CoM.ori.euler.R.yaw.now<<cos(CoM.ori.euler.pos.ref(2)),-sin(CoM.ori.euler.pos.ref(2)),0\
    							,sin(CoM.ori.euler.pos.ref(2)),cos(CoM.ori.euler.pos.ref(2)),0\
    							,0,0,1;

   CoM.ori.euler.R.now=CoM.ori.euler.R.yaw.now*CoM.ori.euler.R.pitch.now*CoM.ori.euler.R.roll.now;

	/**************************************************************************************************************/
   CoM.ori.euler.R.roll.ref<<1,0,0\
    							,0,cos(CoM.ori.euler.pos.ref(0)),-sin(CoM.ori.euler.pos.ref(0))\
    							,0,sin(CoM.ori.euler.pos.ref(0)),cos(CoM.ori.euler.pos.ref(0));

	CoM.ori.euler.R.pitch.ref<<cos(CoM.ori.euler.pos.ref(1)),0,sin(CoM.ori.euler.pos.ref(1))\
    							,0,1,0\
    							,-sin(CoM.ori.euler.pos.ref(1)),0,cos(CoM.ori.euler.pos.ref(1));

   CoM.ori.euler.R.yaw.ref<<cos(CoM.ori.euler.pos.ref(2)),-sin(CoM.ori.euler.pos.ref(2)),0\
    							,sin(CoM.ori.euler.pos.ref(2)),cos(CoM.ori.euler.pos.ref(2)),0\
    							,0,0,1;

   CoM.ori.euler.R.ref=CoM.ori.euler.R.yaw.ref*CoM.ori.euler.R.pitch.ref*CoM.ori.euler.R.roll.ref;
}

VectorNd QRobot::getCOMpos(VectorNd _Base_Pos, MatrixNd C_WB) {
    VectorNd Com_Pos(3);
    
    Com_Pos = _Base_Pos + C_WB*offset_B2C;
    return Com_Pos;
}

VectorNd QRobot::getBASEpos(VectorNd _Com_Pos, MatrixNd C_WB) {
    VectorNd Base_Pos(3);
    Base_Pos = _Com_Pos - C_WB*offset_B2C;
    return Base_Pos;
}
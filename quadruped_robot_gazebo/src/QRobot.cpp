#include <stdio.h>
#include <math.h>
#include "QRobot.h"

QRobot::QRobot() {
	joint = new JOINT[nDOF]; 
	// GetPath();
	//ControlMode=CTRLMODE_NONE;
	ControlMode=CTRLMODE_INIT_POSE;
	//ControlMode=CTRLMODE_WALK_READY;
	setParam();
	setPath();
	setPreviewParam();
	setQPParam();
	setKalmanParam();
}

QRobot::~QRobot() {
	delete[] joint;
}

void QRobot::setPath(void){
	filepath.URDF="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_description/urdf/quadruped_robot.urdf";
	filepath.PREVIEW_GAIN_Gp="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/gain/Gp.txt";
	filepath.PREVIEW_GAIN_Gx="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/gain/Gx.txt";
	filepath.PREVIEW_GAIN_Gi="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/gain/Gi.txt";
}

void QRobot::setParam(void){
	
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

	CoM.pos.ref<<0.0, 0.0, 0.45;
	CoM.vel.ref<<0.0, 0.0, 0.0;
	CoM.acc.ref<<0.0, 0.0, 0.0;
	CoM.pos.goal<<0.0, 0.0, 0.45;
	Traj.walk.bezier.foot_height=0.10;
	CoM.ori.euler.pos.ref<<0.,0.,0.;
	CoM.ori.euler.pos.now<<0.,0.,0.;
	CoM.ori.euler.vel.now<<0.,0.,0.;


	RL.Contact=1;
	RR.Contact=1;
	FL.Contact=1;
	FR.Contact=1;

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
	
	controller.joint.value=VectorNd::Zero(12);
	controller.task.value=VectorNd::Zero(12);
	controller.grf.value=VectorNd::Zero(12);

	Traj.moveState.done=false;
}

void QRobot::setPreviewParam(void){
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

void QRobot::setQPParam(void){
	controller.grf.qp.kp =VectorNd::Zero(6);
	controller.grf.qp.kd =VectorNd::Zero(6);
	controller.grf.qp.kp_mat=MatrixNd::Zero(6,6);
	controller.grf.qp.kd_mat=MatrixNd::Zero(6,6);
	controller.grf.qp.StateErr=VectorNd::Zero(6);
	controller.grf.qp.StateDotErr=VectorNd::Zero(6);
	controller.grf.qp.Inertia_term=VectorNd::Zero(6);
	controller.grf.qp.kp << 10, 10, 4000, 3000, 3000, 0;
    controller.grf.qp.kd << 0.1, 0.1, 40, 30, 30, 10;
    controller.grf.qp.kp_mat=controller.grf.qp.kp.asDiagonal();
    controller.grf.qp.kd_mat=controller.grf.qp.kd.asDiagonal();

	controller.grf.qp.n=12;
	controller.grf.qp.m=0;
	controller.grf.qp.p=24;

	controller.grf.qp.A_qp=MatrixNd::Zero(6,12);
	controller.grf.qp.W=MatrixNd::Zero(12,12);
	controller.grf.qp.b_qp=VectorNd::Zero(6);
	controller.grf.qp.W = MatrixNd::Identity(12, 12);

	controller.grf.qp.tmp_G=MatrixNd::Zero(controller.grf.qp.n,controller.grf.qp.n);
	controller.grf.qp.tmp_g0=VectorNd::Zero(controller.grf.qp.n);
	controller.grf.qp.tmp_CI=MatrixNd::Zero(controller.grf.qp.p,controller.grf.qp.n);

	controller.grf.qp.G.resize(controller.grf.qp.n,controller.grf.qp.n);
	controller.grf.qp.g0.resize(controller.grf.qp.n);
	controller.grf.qp.CE.resize(controller.grf.qp.n,controller.grf.qp.m);
	controller.grf.qp.ce0.resize(controller.grf.qp.m);
	controller.grf.qp.CI.resize(controller.grf.qp.n,controller.grf.qp.p);
	controller.grf.qp.ci0.resize(controller.grf.qp.p);
	controller.grf.qp.x.resize(controller.grf.qp.n);
	controller.grf.qp.x_saved=VectorNd::Zero(12);

	controller.grf.qp.b_local<<0,0,0;
	controller.grf.qp.t_local<<0,0,0;
	controller.grf.qp.n_local<<0,0,0;

	controller.grf.qp.b_semi_global<<0,0,0;
	controller.grf.qp.t_semi_global<<0,0,0;
	controller.grf.qp.n_semi_global<<0,0,0;

	controller.grf.qp.mu=1.0/sqrt(2);

}

void QRobot::setKalmanParam(void){
	kalman.R=0;
	kalman.A=MatrixNd::Zero(2,2);
	kalman.b=VectorNd::Zero(2);
	kalman.Q=MatrixNd::Zero(2,2);
	kalman.H=VectorNd::Zero(2);
	kalman.x_hat=VectorNd::Zero(2);
	kalman.x_bar=VectorNd::Zero(2);
	kalman.p_hat=MatrixNd::Zero(2,2);
	kalman.p_bar=MatrixNd::Zero(2,2);
	kalman.gain=VectorNd::Zero(2);
	kalman.acc=VectorNd::Zero(3);
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
    	controller.joint.value=Joint_PD_Controller();	
    	controller.task.value=VectorNd::Zero(12);
    }else{
    	controller.task.value=Task_PD_Controller();	
    	controller.joint.value=VectorNd::Zero(12);
    	QuadPP_Controller();
    }
    CTC_Torque=G_term+C_term+controller.joint.value+J_A.transpose()*(controller.task.value)-J_A.transpose()*(controller.grf.value);
	//cout<<"CTC:"<<CTC_Torque.transpose()<<endl;
	//cout<<"---"<<endl;
	for (int i = 0; i < nDOF; i++) {
		joint[i].torque.ref = CTC_Torque(i);
	}
}

void QRobot::resetTrajPram(void){
	Traj.cnt=0;
	Traj.walk.bezier.cnt.now=0;
	Traj.walk.zmp.cnt.now=0;
	Traj.moveState.stop=false;
	Traj.walk.stop.body=false;
	Traj.walk.stop.foot=false;
	Traj.walk.stop.all=false;
}

void QRobot::getPreviewParam(void) {
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

	CoM.ori.quat.pos.ref<<0.,0.,0.,1.;
	rbdl.pModel->SetQuaternion(Base.ID,CoM.ori.quat.pos.ref,rbdl.RobotState);

	cout << endl << "Set Robot Model End !!" << endl;
}

void QRobot::StateUpdate(void){
	double measured_height;
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
	
	CoM.ori.quat.pos.ref<<0.,0.,0.,1.;
	rbdl.pModel->SetQuaternion(Base.ID,CoM.ori.quat.pos.ref,rbdl.RobotState);
	
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

	// Base.pos.now=getBASEpos(CoM.pos.ref, CoM.ori.R.now);
	if(CommandFlag==GOTO_INIT_POSE){
		Base.pos.now=-(RL.local.pos.now+RR.local.pos.now+FL.local.pos.now+FR.local.pos.now)/4.0;
		Base.vel.now=-(RL.local.vel.now+RR.local.vel.now+FL.local.vel.now+FR.local.vel.now)/4.0;
		CoM.pos.ref=getCOMpos(Base.pos.now,CoM.ori.R.now);
	}else{
		Base.pos.now=getBASEpos(CoM.pos.ref, CoM.ori.R.now);
		Base.vel.now=getBASEpos(CoM.vel.ref, CoM.ori.R.now);
		measured_height = -(RL.Contact * RL.local.pos.now(2) + RR.Contact * RR.local.pos.now(2) + FL.Contact * FL.local.pos.now(2) + FR.Contact * FR.local.pos.now(2)) / (RL.Contact + RR.Contact + FL.Contact + FR.Contact) / cos(abs(CoM.ori.euler.pos.now(1)));
		KalmanFilter(measured_height);
		Base.pos.now(2)=kalman.x_hat(0);
		//Base.vel.now(2)=kalman.x_hat(1);
	}
	CoM.pos.now=getCOMpos(Base.pos.now,CoM.ori.R.now);	

	// cout<<"Base:"<<Base.pos.now.transpose()<<endl;
	// cout<<"Com:"<<CoM.pos.now.transpose()<<endl;
	// cout<<"--"<<endl;
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

	VectorNd tmp_RL_F=VectorNd::Zero(3);
	VectorNd tmp_RR_F=VectorNd::Zero(3);
	VectorNd tmp_FL_F=VectorNd::Zero(3);
	VectorNd tmp_FR_F=VectorNd::Zero(3);

	Base.pos.ref=getBASEpos(CoM.pos.ref, CoM.ori.R.ref);
	Base.vel.ref=CoM.vel.ref;

	/************************************************************/
	RL.local.semi.pos.now=CoM.ori.R.now*RL.local.pos.now;
	RR.local.semi.pos.now=CoM.ori.R.now*RR.local.pos.now;
	FL.local.semi.pos.now=CoM.ori.R.now*FL.local.pos.now;
	FR.local.semi.pos.now=CoM.ori.R.now*FR.local.pos.now;

	RL.local.semi.vel.now=CoM.ori.R.now*RL.local.vel.now;
	RR.local.semi.vel.now=CoM.ori.R.now*RR.local.vel.now;
	FL.local.semi.vel.now=CoM.ori.R.now*FL.local.vel.now;
	FR.local.semi.vel.now=CoM.ori.R.now*FR.local.vel.now;

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
	tmp_RL_F=CoM.ori.R.ref.transpose()*(RL.kp*(RL.local.semi.pos.ref-RL.local.semi.pos.now)+RL.kd*(RL.local.semi.vel.ref-RL.local.semi.vel.now));
	tmp_RR_F=CoM.ori.R.ref.transpose()*(RR.kp*(RR.local.semi.pos.ref-RR.local.semi.pos.now)+RR.kd*(RR.local.semi.vel.ref-RR.local.semi.vel.now));
	tmp_FL_F=CoM.ori.R.ref.transpose()*(FL.kp*(FL.local.semi.pos.ref-FL.local.semi.pos.now)+FL.kd*(FL.local.semi.vel.ref-FL.local.semi.vel.now));
	tmp_FR_F=CoM.ori.R.ref.transpose()*(FR.kp*(FR.local.semi.pos.ref-FR.local.semi.pos.now)+FR.kd*(FR.local.semi.vel.ref-FR.local.semi.vel.now));
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
	Traj.init.cnt.ref=2000;
	Traj.init.time.ref=Traj.init.cnt.ref*dt;

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
	}else if(Traj.cnt<Traj.init.cnt.ref){
		for (int i=0;i<nDOF;i++){
			joint[i].pos.ref=joint[i].pos.init+(joint[i].pos.goal-joint[i].pos.init)/2.0*(1.0-cos(PI/Traj.init.time.ref*Traj.cnt*dt));
			joint[i].vel.ref=PI/Traj.init.time.ref*(joint[i].pos.goal-joint[i].pos.init)/2.0*sin(PI/Traj.init.time.ref*Traj.cnt*dt);
		}
		Traj.cnt++;
	}else{
		for (int i=0;i<nDOF;i++){
			joint[i].pos.ref=joint[i].pos.goal;
			joint[i].vel.ref=0.0;
		}
		Traj.moveState.done=true;
	}
}

void QRobot::WalkReady_Pose_Traj(void){
	Traj.ready.cnt.ref=2000;
	Traj.ready.time.ref=Traj.ready.cnt.ref*dt;

	if(Traj.cnt==0){
		//CoM.pos.init=-(RL.local.pos.now+RR.local.pos.now+FL.local.pos.now+FR.local.pos.now)/4.0;
		CoM.pos.init=CoM.pos.now;
		CoM.pos.goal<<0.0,0.0,0.45;
		CoM.pos.ref=CoM.pos.init;
		Base.pos.init=getBASEpos(CoM.pos.init, CoM.ori.R.now);
		CoM.vel.ref<<0.0,0.0,0.0;

		//*******************************************************************//
		RL.global.pos.init=Base.pos.init+CoM.ori.R.now*RL.local.pos.now;
		RR.global.pos.init=Base.pos.init+CoM.ori.R.now*RR.local.pos.now;
		FL.global.pos.init=Base.pos.init+CoM.ori.R.now*FL.local.pos.now;
		FR.global.pos.init=Base.pos.init+CoM.ori.R.now*FR.local.pos.now;

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
	}else if(Traj.cnt<Traj.ready.cnt.ref){
		CoM.pos.ref=CoM.pos.init+(CoM.pos.goal-CoM.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
		CoM.vel.ref=PI/Traj.ready.time.ref*(CoM.pos.goal-CoM.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));

		RL.global.pos.ref=RL.global.pos.init+(RL.global.pos.goal-RL.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
		RR.global.pos.ref=RR.global.pos.init+(RR.global.pos.goal-RR.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
		FL.global.pos.ref=FL.global.pos.init+(FL.global.pos.goal-FL.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));
		FR.global.pos.ref=FR.global.pos.init+(FR.global.pos.goal-FR.global.pos.init)/2.0*(1-cos(PI/Traj.ready.time.ref*Traj.cnt*dt));

		RL.global.vel.ref=PI/Traj.ready.time.ref*(RL.global.pos.goal-RL.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
		RR.global.vel.ref=PI/Traj.ready.time.ref*(RR.global.pos.goal-RR.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
		FL.global.vel.ref=PI/Traj.ready.time.ref*(FL.global.pos.goal-FL.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
		FR.global.vel.ref=PI/Traj.ready.time.ref*(FR.global.pos.goal-FR.global.pos.init)/2.0*(sin(PI/Traj.ready.time.ref*Traj.cnt*dt));
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

			Traj.moveState.done=true;
	}
	
	// if(Traj.cnt<=Traj.ready.time/dt){
	// 	cout<<"CoM:"<<CoM.pos.ref.transpose()<<endl;
	// 	cout<<"RL:"<<RL.global.pos.ref.transpose()<<endl;
	// 	cout<<"RR:"<<RR.global.pos.ref.transpose()<<endl;
	// 	cout<<"FL:"<<FL.global.pos.ref.transpose()<<endl;
	// 	cout<<"FR:"<<FR.global.pos.ref.transpose()<<endl;
	// 	cout<<"-----"<<endl;	
	// }
}

void QRobot::Slow_Walk_Traj(void){
	Traj.walk.fly.cnt.ref=700;
	Traj.walk.fly.time.ref=Traj.walk.fly.cnt.ref*dt;
	
	Traj.walk.stance.cnt.ref=50;
	Traj.walk.stance.time.ref=Traj.walk.stance.cnt.ref*dt;

	Traj.walk.cnt.ref=(Traj.walk.fly.cnt.ref+Traj.walk.stance.cnt.ref)*4;
	Traj.walk.time.ref=Traj.walk.cnt.ref*dt;
	
	if(Traj.cnt<Traj.walk.cnt.ref){
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
			
			Traj.walk.vel.ref<<joy.vel_x, joy.vel_y, joy.vel_yaw;
			FootStepPlanning();
		}

		CoM.ori.euler.pos.ref(2)=CoM.ori.euler.pos.pre_init(2)+(CoM.ori.euler.pos.init(2)-CoM.ori.euler.pos.pre_init(2))/2.0*(1-cos(PI/(Traj.walk.cnt.ref)*(Traj.cnt-Traj.walk.cnt.ref)));
		cout<<CoM.ori.euler.pos.ref(2)<<endl;
		cout<<"----"<<endl;
		FootMove_Traj(Traj.cnt-Traj.walk.cnt.ref);

		if (Traj.cnt == (Traj.walk.cnt.ref*2-1)) {
			Traj.walk.vel.now=Traj.walk.vel.ref;
			CoM.pos.init(0)=CoM.pos.goal(0);
			CoM.pos.init(1)=CoM.pos.goal(1);
			 
			CoM.ori.euler.pos.pre_init(2) = CoM.ori.euler.pos.init(2);
            CoM.ori.euler.pos.init(2)=CoM.ori.euler.pos.goal(2);

			RL.global.pos.pre_init.segment(0,2)=RL.global.pos.init.segment(0,2);
			RR.global.pos.pre_init.segment(0,2)=RR.global.pos.init.segment(0,2);
			FL.global.pos.pre_init.segment(0,2)=FL.global.pos.init.segment(0,2);
			FR.global.pos.pre_init.segment(0,2)=FR.global.pos.init.segment(0,2);

			RL.global.pos.init.segment(0,2)=RL.global.pos.goal.segment(0,2);
			RR.global.pos.init.segment(0,2)=RR.global.pos.goal.segment(0,2);
			FL.global.pos.init.segment(0,2)=FL.global.pos.goal.segment(0,2);
			FR.global.pos.init.segment(0,2)=FR.global.pos.goal.segment(0,2);

			Traj.cnt=Traj.walk.cnt.ref-1;
		}
	 	Traj.cnt++;
	}
	BodyMove_Traj();
}

void QRobot::Init_Slow_walk_traj(unsigned int _i){
	if(_i==0){
		CoM.pos.init=CoM.pos.goal;

		CoM.ori.euler.pos.pre_init(2)=CoM.ori.euler.pos.ref(2);
		CoM.ori.euler.pos.init(2)=CoM.ori.euler.pos.pre_init(2);

		RL.global.pos.pre_init=RL.global.pos.ref;
		RR.global.pos.pre_init=RR.global.pos.ref;
		FL.global.pos.pre_init=FL.global.pos.ref;
		FR.global.pos.pre_init=FR.global.pos.ref;
		RL.global.pos.init=RL.global.pos.ref;
		RR.global.pos.init=RR.global.pos.ref;
		FL.global.pos.init=FL.global.pos.ref;
		FR.global.pos.init=FR.global.pos.ref;

		Traj.walk.vel.now<<0.,0.,0.;
		Traj.walk.vel.ref<<0.,0.,0.;
		FootStepPlanning();

		// cout<<Base.ori.euler.pos.pre_init(2)<<endl;
		// cout<<Base.ori.euler.pos.init(2)<<endl;
		// cout<<"------------------"<<endl;
		// cout<<RL.global.pos.pre_init.transpose()<<endl;
		// cout<<RL.global.pos.init.transpose()<<endl;
		// cout<<"------------------"<<endl;
	}

}

void QRobot::FootStepPlanning(void){

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

	Traj.walk.footstep.tmp_RL_pos = Traj.walk.footstep.R_incre * (RL.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);
	Traj.walk.footstep.tmp_RR_pos = Traj.walk.footstep.R_incre * (RR.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);
	Traj.walk.footstep.tmp_FL_pos = Traj.walk.footstep.R_incre * (FL.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);
	Traj.walk.footstep.tmp_FR_pos = Traj.walk.footstep.R_incre * (FR.global.pos.init.segment(0,2) - CoM.pos.init.segment(0,2)) + CoM.pos.init.segment(0,2);

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
	
	FL.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_FL_pos2 + Traj.walk.footstep.R_goal*Traj.walk.footstep.delta_y_local;
	FR.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_FR_pos2 + Traj.walk.footstep.R_goal*Traj.walk.footstep.delta_y_local;
	RL.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_RL_pos2 +((Traj.walk.footstep.tmp_FL_pos2 - Traj.walk.footstep.tmp_RL_pos2).transpose() * Traj.walk.footstep.ny_global)*Traj.walk.footstep.ny_global + Traj.walk.footstep.R_goal * Traj.walk.footstep.delta_y_local / 2.0;
	RR.global.pos.goal.segment(0,2) = Traj.walk.footstep.tmp_RR_pos2 +((Traj.walk.footstep.tmp_FR_pos2 - Traj.walk.footstep.tmp_RR_pos2).transpose() * Traj.walk.footstep.ny_global)*Traj.walk.footstep.ny_global + Traj.walk.footstep.R_goal * Traj.walk.footstep.delta_y_local / 2.0;

    CoM.pos.goal(0) = (RL.global.pos.goal(0) + RR.global.pos.goal(0) + FL.global.pos.goal(0) + FR.global.pos.goal(0)) / 4.0;
    CoM.pos.goal(1) = (RL.global.pos.goal(1) + RR.global.pos.goal(1) + FL.global.pos.goal(1) + FR.global.pos.goal(1)) / 4.0;

    // cout<<"RL:"<<RL.global.pos.goal.transpose()<<endl;
    // cout<<"RR:"<<RR.global.pos.goal.transpose()<<endl;
    // cout<<"FL:"<<FL.global.pos.goal.transpose()<<endl;
    // cout<<"FR:"<<FR.global.pos.goal.transpose()<<endl;
    // cout<<"-----"<<endl;
    // cout<<CoM.pos.goal.transpose()<<endl;
    // cout<<"-----"<<endl;
}

void QRobot::getRotationMatrix(void){
	CoM.ori.R.roll.now<<1,0,0\
    							,0,cos(CoM.ori.euler.pos.now(0)),-sin(CoM.ori.euler.pos.now(0))\
    							,0,sin(CoM.ori.euler.pos.now(0)),cos(CoM.ori.euler.pos.now(0));

	CoM.ori.R.pitch.now<<cos(CoM.ori.euler.pos.now(1)),0,sin(CoM.ori.euler.pos.now(1))\
    							,0,1,0\
    							,-sin(CoM.ori.euler.pos.now(1)),0,cos(CoM.ori.euler.pos.now(1));

   CoM.ori.R.yaw.now<<cos(CoM.ori.euler.pos.ref(2)),-sin(CoM.ori.euler.pos.ref(2)),0\
    							,sin(CoM.ori.euler.pos.ref(2)),cos(CoM.ori.euler.pos.ref(2)),0\
    							,0,0,1;

   CoM.ori.R.now=CoM.ori.R.yaw.now*CoM.ori.R.pitch.now*CoM.ori.R.roll.now;

	/**************************************************************************************************************/
   CoM.ori.R.roll.ref<<1,0,0\
    							,0,cos(CoM.ori.euler.pos.ref(0)),-sin(CoM.ori.euler.pos.ref(0))\
    							,0,sin(CoM.ori.euler.pos.ref(0)),cos(CoM.ori.euler.pos.ref(0));

	CoM.ori.R.pitch.ref<<cos(CoM.ori.euler.pos.ref(1)),0,sin(CoM.ori.euler.pos.ref(1))\
    							,0,1,0\
    							,-sin(CoM.ori.euler.pos.ref(1)),0,cos(CoM.ori.euler.pos.ref(1));

   CoM.ori.R.yaw.ref<<cos(CoM.ori.euler.pos.ref(2)),-sin(CoM.ori.euler.pos.ref(2)),0\
    							,sin(CoM.ori.euler.pos.ref(2)),cos(CoM.ori.euler.pos.ref(2)),0\
    							,0,0,1;

   CoM.ori.R.ref=CoM.ori.R.yaw.ref*CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref;
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

void QRobot::FootMove_Traj(unsigned int _i){
	if(Traj.moveState.stop==false){
		if(_i<Traj.walk.fly.cnt.ref){
			if(_i==0){
				RR.Contact=0;
				RL.global.pos.pre_init(2)=RL.global.pos.ref(2);
				RR.global.pos.pre_init(2)=RR.global.pos.ref(2);
				FL.global.pos.pre_init(2)=FL.global.pos.ref(2);
				FR.global.pos.pre_init(2)=FR.global.pos.ref(2);

				RL.global.pos.init(2)=RL.global.pos.pre_init(2);
				RR.global.pos.init(2)=RR.global.pos.pre_init(2);
				FL.global.pos.init(2)=FL.global.pos.pre_init(2);
				FR.global.pos.init(2)=FR.global.pos.pre_init(2);
			}

			RL.global.pos.ref=RL.global.pos.pre_init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;

			Bezier_Traj(RR.global.pos.pre_init,RR.global.pos.init);
			RR.global.pos.ref=Traj.walk.bezier.pos;
			RR.global.vel.ref=Traj.walk.bezier.vel;

			FL.global.pos.ref=FL.global.pos.pre_init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;

			FR.global.pos.ref=FR.global.pos.pre_init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
		}else if(_i<(Traj.walk.fly.cnt.ref+Traj.walk.stance.cnt.ref)){
			RL.global.pos.ref=RL.global.pos.pre_init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;
			RL.Contact=1;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;
			RR.Contact=1;

			FL.global.pos.ref=FL.global.pos.pre_init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;
			FL.Contact=1;

			FR.global.pos.ref=FR.global.pos.pre_init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
			FR.Contact=1;
		}else if(_i<(Traj.walk.cnt.ref/4+Traj.walk.fly.cnt.ref)){
			if(_i==Traj.walk.cnt.ref/4){
				RL.Contact=0;
				RL.global.pos.pre_init(2)=RL.global.pos.ref(2);
				RR.global.pos.pre_init(2)=RR.global.pos.ref(2);
				FL.global.pos.pre_init(2)=FL.global.pos.ref(2);
				FR.global.pos.pre_init(2)=FR.global.pos.ref(2);

				RL.global.pos.init(2)=RL.global.pos.pre_init(2);
				RR.global.pos.init(2)=RR.global.pos.pre_init(2);
				FL.global.pos.init(2)=FL.global.pos.pre_init(2);
				FR.global.pos.init(2)=FR.global.pos.pre_init(2);
			}

			Bezier_Traj(RL.global.pos.pre_init,RL.global.pos.init);
			RL.global.pos.ref=Traj.walk.bezier.pos;
			RL.global.vel.ref=Traj.walk.bezier.vel;
			// RL.global.pos.ref=RL.global.pos.ref;
			// RL.global.vel.ref<<0.,0.,0.;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;

			FL.global.pos.ref=FL.global.pos.pre_init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;

			FR.global.pos.ref=FR.global.pos.pre_init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
		}else if(_i<Traj.walk.cnt.ref*2/4){
			RL.global.pos.ref=RL.global.pos.init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;
			RL.Contact=1;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;
			RR.Contact=1;

			FL.global.pos.ref=FL.global.pos.pre_init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;
			FL.Contact=1;

			FR.global.pos.ref=FR.global.pos.pre_init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
			FR.Contact=1;

		}else if(_i<(Traj.walk.cnt.ref*2/4+Traj.walk.fly.cnt.ref)){
			if(_i==Traj.walk.cnt.ref*2/4){
				FL.Contact=0;
				RL.global.pos.pre_init(2)=RL.global.pos.ref(2);
				RR.global.pos.pre_init(2)=RR.global.pos.ref(2);
				FL.global.pos.pre_init(2)=FL.global.pos.ref(2);
				FR.global.pos.pre_init(2)=FR.global.pos.ref(2);

				RL.global.pos.init(2)=RL.global.pos.pre_init(2);
				RR.global.pos.init(2)=RR.global.pos.pre_init(2);
				FL.global.pos.init(2)=FL.global.pos.pre_init(2);
				FR.global.pos.init(2)=FR.global.pos.pre_init(2);
			}

			RL.global.pos.ref=RL.global.pos.init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;

			Bezier_Traj(FL.global.pos.pre_init,FL.global.pos.init);
			FL.global.pos.ref=Traj.walk.bezier.pos;
			FL.global.vel.ref=Traj.walk.bezier.vel;
			// FL.global.pos.ref=FL.global.pos.ref;
			// FL.global.vel.ref<<0.,0.,0.;
			
			FR.global.pos.ref=FR.global.pos.pre_init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
		}else if(_i<Traj.walk.cnt.ref*3/4){
			RL.global.pos.ref=RL.global.pos.init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;
			RL.Contact=1;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;
			RR.Contact=1;

			FL.global.pos.ref=FL.global.pos.init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;
			FL.Contact=1;

			FR.global.pos.ref=FR.global.pos.pre_init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
			FR.Contact=1;
		}else if(_i<(Traj.walk.cnt.ref*3/4+Traj.walk.fly.cnt.ref)){
			if(_i==Traj.walk.cnt.ref*3/4){
				FR.Contact=0;
				RL.global.pos.pre_init(2)=RL.global.pos.ref(2);
				RR.global.pos.pre_init(2)=RR.global.pos.ref(2);
				FL.global.pos.pre_init(2)=FL.global.pos.ref(2);
				FR.global.pos.pre_init(2)=FR.global.pos.ref(2);

				RL.global.pos.init(2)=RL.global.pos.pre_init(2);
				RR.global.pos.init(2)=RR.global.pos.pre_init(2);
				FL.global.pos.init(2)=FL.global.pos.pre_init(2);
				FR.global.pos.init(2)=FR.global.pos.pre_init(2);
			}

			RL.global.pos.ref=RL.global.pos.init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;

			FL.global.pos.ref=FL.global.pos.init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;

			Bezier_Traj(FR.global.pos.pre_init,FR.global.pos.init);
			FR.global.pos.ref=Traj.walk.bezier.pos;
			FR.global.vel.ref=Traj.walk.bezier.vel;
			// FR.global.pos.ref=FR.global.pos.ref;
			// FR.global.vel.ref<<0.,0.,0.;
		}else{
			RL.global.pos.ref=RL.global.pos.init;
			RL.global.pos.ref(2)=RL.global.pos.ref(2);
			RL.global.vel.ref<<0.,0.,0.;
			RL.Contact=1;

			RR.global.pos.ref=RR.global.pos.init;
			RR.global.pos.ref(2)=RR.global.pos.ref(2);
			RR.global.vel.ref<<0.,0.,0.;
			RR.Contact=1;

			FL.global.pos.ref=FL.global.pos.init;
			FL.global.pos.ref(2)=FL.global.pos.ref(2);
			FL.global.vel.ref<<0.,0.,0.;
			FL.Contact=1;

			FR.global.pos.ref=FR.global.pos.init;
			FR.global.pos.ref(2)=FR.global.pos.ref(2);
			FR.global.vel.ref<<0.,0.,0.;
			FR.Contact=1;
		}

		// if(_i==(Traj.walk.cnt.ref-1)){
		// 	CoM.pos.init(0)=CoM.pos.goal(0);
		// 	CoM.pos.init(1)=CoM.pos.goal(1);
			 
		// 	CoM.pos.pre_init(2) = CoM.pos.init(2);
  //           CoM.pos.init(2)=CoM.pos.goal(2);
  //           // init_yaw_HS = goal_yaw_HS;

		// 	RL.global.pos.pre_init.segment(0,2)=RL.global.pos.init.segment(0,2);
		// 	RR.global.pos.pre_init.segment(0,2)=RR.global.pos.init.segment(0,2);
		// 	FL.global.pos.pre_init.segment(0,2)=FL.global.pos.init.segment(0,2);
		// 	FR.global.pos.pre_init.segment(0,2)=FR.global.pos.init.segment(0,2);

		// 	RL.global.pos.init.segment(0,2)=RL.global.pos.goal.segment(0,2);
		// 	RR.global.pos.init.segment(0,2)=RR.global.pos.goal.segment(0,2);
		// 	FL.global.pos.init.segment(0,2)=FL.global.pos.goal.segment(0,2);
		// 	FR.global.pos.init.segment(0,2)=FR.global.pos.goal.segment(0,2);

		// }
	}
}

void QRobot::Bezier_Traj(VectorNd init_EP_pos_3d, VectorNd goal_EP_pos_3d) {
	VectorNd Dist_2d_global(2);
	VectorNd now_vel_global(2);
	double tmp_foot_height;
	double alpha=1.2;
	double beta=1.2;

	now_vel_global = CoM.ori.R.yaw.ref.block(0,0,2,2) * Traj.walk.vel.now.segment(0, 2);
	Dist_2d_global = now_vel_global * Traj.walk.time.ref;

	if((RL.Contact+RR.Contact+FL.Contact+FR.Contact)==3){
		Traj.walk.bezier.enable=true;
		if(Traj.walk.vel.now(2)>0){
			if(RL.Contact==1 || FL.Contact==1){
				tmp_foot_height=Traj.walk.bezier.foot_height/beta;
			}
			if(RR.Contact==1 || FR.Contact==1){
				tmp_foot_height=Traj.walk.bezier.foot_height+0.05;
			}
		}else if(Traj.walk.vel.now(2)<0){
			if(RL.Contact==1 || FL.Contact==1){
				tmp_foot_height=Traj.walk.bezier.foot_height+0.05;
			}
			if(RR.Contact==1 || FR.Contact==1){
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
			Traj.walk.bezier.cnt.now++;
		}else if (Traj.walk.bezier.cnt.now<Traj.walk.fly.cnt.ref){
			Traj.walk.bezier.b0 = pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 6);
			Traj.walk.bezier.b1 = 6 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 5) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 1);
			Traj.walk.bezier.b2 = 15 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 4) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 2);
			Traj.walk.bezier.b3 = 20 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 3) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 3);
			Traj.walk.bezier.b4 = 15 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 2) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 4);
			Traj.walk.bezier.b5 = 6 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 1) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 5);	
			Traj.walk.bezier.b6 = pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 6);

			Traj.walk.bezier.b0_dot = 6 * pow((1. - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 5)*(-1 / (Traj.walk.fly.time.ref));
			Traj.walk.bezier.b1_dot = 30 * pow((1. - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 4)*(-1 / (Traj.walk.fly.time.ref)) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 1) + 6 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 5)*(1 / (Traj.walk.fly.time.ref));
			Traj.walk.bezier.b2_dot = 60 * pow((1. - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 3)*(-1 / (Traj.walk.fly.time.ref)) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 2) + 30 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 4) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 1)*(1 / (Traj.walk.fly.time.ref));
			Traj.walk.bezier.b3_dot = 60 * pow((1. - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 2)*(-1 / (Traj.walk.fly.time.ref)) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 3) + 60 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 3) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 2)*(1 / (Traj.walk.fly.time.ref));
			Traj.walk.bezier.b4_dot = 30 * pow((1. - Traj.walk.bezier.time.now/ (Traj.walk.fly.time.ref)), 1)*(-1 / (Traj.walk.fly.time.ref)) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 4) + 60 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 2) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 3)*(1 / (Traj.walk.fly.time.ref));
			Traj.walk.bezier.b5_dot = 6 * (-1 / (Traj.walk.fly.time.ref)) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 5) + 30 * pow((1 - Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 1) * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 4)*(1 / (Traj.walk.fly.time.ref));
			Traj.walk.bezier.b6_dot = 6 * pow((Traj.walk.bezier.time.now / (Traj.walk.fly.time.ref)), 5)*(1 / (Traj.walk.fly.time.ref));

			Traj.walk.bezier.pos = Traj.walk.bezier.P0 * Traj.walk.bezier.b0 + Traj.walk.bezier.P1 * Traj.walk.bezier.b1 + Traj.walk.bezier.P2 * Traj.walk.bezier.b2 + Traj.walk.bezier.P3 * Traj.walk.bezier.b3 + Traj.walk.bezier.P4 * Traj.walk.bezier.b4 + Traj.walk.bezier.P5 * Traj.walk.bezier.b5 + Traj.walk.bezier.P6*Traj.walk.bezier.b6;
			Traj.walk.bezier.vel = Traj.walk.bezier.P0 * Traj.walk.bezier.b0_dot + Traj.walk.bezier.P1 * Traj.walk.bezier.b1_dot + Traj.walk.bezier.P2 * Traj.walk.bezier.b2_dot + Traj.walk.bezier.P3 * Traj.walk.bezier.b3_dot + Traj.walk.bezier.P4 * Traj.walk.bezier.b4_dot + Traj.walk.bezier.P5 * Traj.walk.bezier.b5_dot + Traj.walk.bezier.P6*Traj.walk.bezier.b6_dot;
			Traj.walk.bezier.cnt.now++;
			// cout<<Traj.walk.bezier.pos.transpose()<<endl;
		}
		// if(Traj.walk.bezier.cnt==Traj.walk.fly_time/dt){
		if(Traj.walk.bezier.cnt.now==Traj.walk.fly.cnt.ref){
				Traj.walk.bezier.cnt.now=0;
				Traj.walk.bezier.pos=Traj.walk.bezier.P6;
				Traj.walk.bezier.vel<<0.,0.,0.;
				Traj.walk.bezier.enable=false;
		}
	}

	if(Traj.walk.bezier.enable==false){
		Traj.walk.bezier.cnt.now=0;
		Traj.walk.bezier.pos = init_EP_pos_3d;
		Traj.walk.bezier.vel << 0., 0., 0.;
	}
	// cout<<"---"<<endl;
	// cout<<"init"<<init_EP_pos_3d<<endl;
	// cout<<"---"<<endl;
}	

void QRobot::BodyMove_Traj(void){
	// double tmp_target_yaw;
	MatrixNd tmp_R(2,2);

	if(Traj.walk.zmp.cnt.now==0){
		Traj.walk.zmp.swing.local_x<<0.05,0.;
		Traj.walk.zmp.swing.local_y<<0.,0.08;
	}
	
	tmp_target_yaw = CoM.ori.euler.pos.init(2) + (CoM.ori.euler.pos.goal(2) - CoM.ori.euler.pos.init(2)) / 2.0 * (1. - cos(PI / Traj.walk.time.ref * Traj.walk.zmp.cnt.now*dt));
	// cout<<tmp_target_yaw<<endl;
	// cout<<"Yaw:"<<"---"<<endl;
	tmp_R<<cos(tmp_target_yaw),-sin(tmp_target_yaw)\
	,sin(tmp_target_yaw),cos(tmp_target_yaw);
	
	Traj.walk.zmp.swing.global_x=tmp_R*Traj.walk.zmp.swing.local_x;
	Traj.walk.zmp.swing.global_y=tmp_R*Traj.walk.zmp.swing.local_y;

	if(Traj.walk.zmp.cnt.now<=(Traj.walk.fly.cnt.ref)){
		Traj.walk.zmp.ref=(RL.global.pos.init.segment(0,2)+FR.global.pos.init.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref/4)){
		Traj.walk.zmp.ref=(RR.global.pos.goal.segment(0,2)+FL.global.pos.init.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref/4+Traj.walk.fly.cnt.ref)){
		Traj.walk.zmp.ref=(RR.global.pos.goal.segment(0,2)+FL.global.pos.init.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*2/4)){
		Traj.walk.zmp.ref=(RL.global.pos.goal.segment(0,2)+FR.global.pos.init.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;	

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*2/4+Traj.walk.fly.cnt.ref)){
		Traj.walk.zmp.ref=(RL.global.pos.goal.segment(0,2)+FR.global.pos.init.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x-Traj.walk.zmp.swing.global_y;	

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*3/4)){
		Traj.walk.zmp.ref=(RR.global.pos.goal.segment(0,2)+FL.global.pos.goal.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;	

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*3/4+Traj.walk.fly.cnt.ref)){
		Traj.walk.zmp.ref=(RR.global.pos.goal.segment(0,2)+FL.global.pos.goal.segment(0,2))/2.0-Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;	

	}else if(Traj.walk.zmp.cnt.now<=(Traj.walk.cnt.ref*4/4)){
		Traj.walk.zmp.ref=(RL.global.pos.goal.segment(0,2)+FR.global.pos.goal.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;	

	}else{
		Traj.walk.zmp.ref=(RL.global.pos.goal.segment(0,2)+FR.global.pos.goal.segment(0,2))/2.0+Traj.walk.zmp.swing.global_x+Traj.walk.zmp.swing.global_y;	
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

void QRobot::ZMP_PreviewControl(void){
    Traj.walk.zmp.ref_old=Traj.walk.zmp.CC.transpose()*Traj.walk.zmp.X_new;
    Traj.walk.zmp.err=Traj.walk.zmp.ref_old-Traj.walk.zmp.ref_array.block(0,0,2,1);
    Traj.walk.zmp.sum_e=Traj.walk.zmp.sum_e+Traj.walk.zmp.err;

    Traj.walk.zmp.sum_p<<0.,0.;
    Traj.walk.zmp.sum_p=Traj.walk.zmp.ref_array*Traj.walk.zmp.Gp;
     
    Traj.walk.zmp.u = -Traj.walk.zmp.Gi*Traj.walk.zmp.sum_e-Traj.walk.zmp.X_new.transpose()*Traj.walk.zmp.Gx-Traj.walk.zmp.sum_p;
    Traj.walk.zmp.X_new=Traj.walk.zmp.AA*Traj.walk.zmp.X_new+Traj.walk.zmp.BB*Traj.walk.zmp.u.transpose();
}

// VectorNd QRobot::QuadPP_Controller(void){
void QRobot::QuadPP_Controller(void){
	VectorNd CoM_ref(3);
	VectorNd CoM_now(3);
	double alpha=0.001;

	RL.local.pos.now_from_com=RL.local.pos.now-offset_B2C;
    RR.local.pos.now_from_com=RR.local.pos.now-offset_B2C;
    FL.local.pos.now_from_com=FL.local.pos.now-offset_B2C;
    FR.local.pos.now_from_com=FR.local.pos.now-offset_B2C;

    RL.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(RL.local.pos.now_from_com);
    RR.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(RR.local.pos.now_from_com);
    FL.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(FL.local.pos.now_from_com);
    FR.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(FR.local.pos.now_from_com);
    CoM_now=-(RL.Contact*RL.local.semi.pos.now_from_com+RR.Contact*RR.local.semi.pos.now_from_com+FL.Contact*FL.local.semi.pos.now_from_com+FR.Contact*FR.local.semi.pos.now_from_com)/(RL.Contact+RR.Contact+FL.Contact+FR.Contact);

    controller.grf.qp.A_qp.block(0,0,3,12) << RL.Contact, 0, 0, RR.Contact, 0, 0, FL.Contact, 0, 0, FR.Contact, 0, 0\
											, 0, RL.Contact, 0, 0, RR.Contact, 0, 0, FL.Contact, 0, 0, FR.Contact, 0\
											, 0, 0, RL.Contact, 0, 0, RR.Contact, 0, 0, FL.Contact, 0, 0, FR.Contact;
	controller.grf.qp.A_qp.block(3,0,3,3)=RL.Contact*skewMat(RL.local.semi.pos.now_from_com);
	controller.grf.qp.A_qp.block(3,3,3,3)=RR.Contact*skewMat(RR.local.semi.pos.now_from_com);
	controller.grf.qp.A_qp.block(3,6,3,3)=FL.Contact*skewMat(FL.local.semi.pos.now_from_com);
	controller.grf.qp.A_qp.block(3,9,3,3)=FR.Contact*skewMat(FR.local.semi.pos.now_from_com);

    /*********/
    RL.local.pos.ref=CoM.ori.R.ref.transpose()*(RL.global.pos.ref-CoM.pos.ref);
	RR.local.pos.ref=CoM.ori.R.ref.transpose()*(RR.global.pos.ref-CoM.pos.ref);
	FL.local.pos.ref=CoM.ori.R.ref.transpose()*(FL.global.pos.ref-CoM.pos.ref);
	FR.local.pos.ref=CoM.ori.R.ref.transpose()*(FR.global.pos.ref-CoM.pos.ref);
    RL.local.pos.ref_from_com=RL.local.pos.ref-offset_B2C;
    RR.local.pos.ref_from_com=RR.local.pos.ref-offset_B2C;
    FL.local.pos.ref_from_com=FL.local.pos.ref-offset_B2C;
    FR.local.pos.ref_from_com=FR.local.pos.ref-offset_B2C;
	RL.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(RL.local.pos.ref_from_com);
    RR.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(RR.local.pos.ref_from_com);
    FL.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(FL.local.pos.ref_from_com);
    FR.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(FR.local.pos.ref_from_com);
    CoM_ref=-(RL.Contact*RL.local.semi.pos.ref_from_com+RR.Contact*RR.local.semi.pos.ref_from_com+FL.Contact*FL.local.semi.pos.ref_from_com+FR.Contact*FR.local.semi.pos.ref_from_com)/(RL.Contact+RR.Contact+FL.Contact+FR.Contact);

	controller.grf.qp.StateErr(0)=CoM_ref(0)-CoM_now(0);
	controller.grf.qp.StateErr(1)=CoM_ref(1)-CoM_now(1);
	controller.grf.qp.StateErr(2)=CoM.pos.goal(2)/cos(abs(CoM.ori.euler.pos.ref(1)))-CoM.pos.now(2)/cos(abs(CoM.ori.euler.pos.now(1)));
	controller.grf.qp.StateErr(3)=CoM.ori.euler.pos.ref(0)-CoM.ori.euler.pos.now(0);
	controller.grf.qp.StateErr(4)=CoM.ori.euler.pos.ref(1)-CoM.ori.euler.pos.now(1);
	controller.grf.qp.StateErr(5)=0.0;

	controller.grf.qp.StateDotErr(0)=0.0;
	controller.grf.qp.StateDotErr(1)=0.0;
	controller.grf.qp.StateDotErr(2)=0.0;
	controller.grf.qp.StateDotErr(3)=CoM.ori.euler.vel.ref(0)-CoM.ori.euler.vel.now(0);
	controller.grf.qp.StateDotErr(4)=CoM.ori.euler.vel.ref(1)-CoM.ori.euler.vel.now(1);
	controller.grf.qp.StateDotErr(5)=CoM.ori.euler.vel.ref(2)-CoM.ori.euler.vel.now(2);

	CoM.semi.acc.ref = CoM.ori.R.yaw.ref.transpose()*CoM.acc.ref;
	controller.grf.qp.Inertia_term.segment(0,3)<<Robot_mass*CoM.semi.acc.ref(0),Robot_mass*CoM.semi.acc.ref(1),Robot_mass*CoM.semi.acc.ref(2)+Robot_mass*GRAVITY;
	controller.grf.qp.Inertia_term.segment(3,3)<<0.,0.,0.;

	controller.grf.qp.b_qp=controller.grf.qp.Inertia_term+controller.grf.qp.kp_mat*controller.grf.qp.StateErr+controller.grf.qp.kd_mat*controller.grf.qp.StateDotErr;

	controller.grf.qp.tmp_G=2*(controller.grf.qp.A_qp.transpose()*controller.grf.qp.A_qp+alpha*controller.grf.qp.W);
	controller.grf.qp.tmp_g0=-2*(controller.grf.qp.A_qp.transpose()*controller.grf.qp.b_qp);

	for (int i = 0; i < controller.grf.qp.n; i++) {
		for (int j = 0; j < controller.grf.qp.n; j++) {
			controller.grf.qp.G[i][j] = controller.grf.qp.tmp_G(i, j);
		}
	}

	for (int i = 0; i < controller.grf.qp.n; i++) {
		controller.grf.qp.g0[i] = controller.grf.qp.tmp_g0[i];
	}
    

	for (int i = 0; i < controller.grf.qp.n; i++) {
		for (int j = 0; j < controller.grf.qp.m; j++) {
			controller.grf.qp.CE[i][j] = 0;
		}
	}

	for (int j = 0; j < controller.grf.qp.m; j++) {
		controller.grf.qp.ce0[j] = 0;
	}

	for (int i = 0; i < controller.grf.qp.n; i++) {
		for (int j = 0; j < controller.grf.qp.p; j++) {
			controller.grf.qp.CI[i][j] = 0;
		}
	}

	controller.grf.qp.n_local<<0,0,1;
	controller.grf.qp.b_local<<0,1,0;
	controller.grf.qp.t_local<<1,0,0;

	controller.grf.qp.n_semi_global=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*controller.grf.qp.n_local;
	controller.grf.qp.b_semi_global=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*controller.grf.qp.b_local;
	controller.grf.qp.t_semi_global=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*controller.grf.qp.t_local;

	// cout<<"n:"<<controller.grf.qp.n_semi_global.transpose()<<endl;
	// cout<<"b:"<<controller.grf.qp.b_semi_global.transpose()<<endl;
	// cout<<"t:"<<controller.grf.qp.t_semi_global.transpose()<<endl;

	controller.grf.qp.tmp_CI.block(0,0,1,3)=RL.Contact*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(1,0,1,3)=RL.Contact*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(2,0,1,3)=RL.Contact*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(3,0,1,3)=RL.Contact*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(4,0,1,3)=RL.Contact*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(5,0,1,3)<<0,0,-RL.Contact;

	controller.grf.qp.tmp_CI.block(6,3,1,3)=RR.Contact*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(7,3,1,3)=RR.Contact*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(8,3,1,3)=RR.Contact*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(9,3,1,3)=RR.Contact*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(10,3,1,3)=RR.Contact*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(11,3,1,3)<<0,0,-RR.Contact;

	controller.grf.qp.tmp_CI.block(12,6,1,3)<<FL.Contact*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(13,6,1,3)<<FL.Contact*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(14,6,1,3)<<FL.Contact*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(15,6,1,3)<<FL.Contact*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(16,6,1,3)<<FL.Contact*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(17,6,1,3)<<0,0,-FL.Contact;

	controller.grf.qp.tmp_CI.block(18,9,1,3)<<FR.Contact*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(19,9,1,3)<<FR.Contact*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(20,9,1,3)<<FR.Contact*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(21,9,1,3)<<FR.Contact*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(22,9,1,3)<<FR.Contact*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(23,9,1,3)<<0,0,-FR.Contact;

	for (int i = 0; i < controller.grf.qp.n; i++) {
		for (int j = 0; j < controller.grf.qp.p; j++) {
			controller.grf.qp.CI[i][j] = controller.grf.qp.tmp_CI(j, i);
		}
	}

    for (int i=0; i<controller.grf.qp.p;i++){
    	controller.grf.qp.ci0[i]=0;
    }

    controller.grf.qp.ci0[5] = 2.0*Robot_mass*GRAVITY;
    controller.grf.qp.ci0[11] = 2.0*Robot_mass*GRAVITY;
    controller.grf.qp.ci0[17] = 2.0*Robot_mass*GRAVITY;
    controller.grf.qp.ci0[23] = 2.0*Robot_mass*GRAVITY;

    if (isfinite(solve_quadprog(controller.grf.qp.G, controller.grf.qp.g0, controller.grf.qp.CE, controller.grf.qp.ce0, controller.grf.qp.CI, controller.grf.qp.ci0, controller.grf.qp.x))) {
        for (int i = 0; i < controller.grf.qp.n; ++i) {
            controller.grf.qp.x_saved(i) = controller.grf.qp.x[i];
        }
    }
 
    controller.grf.value.segment(0,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved.segment(0,3);
    controller.grf.value.segment(3,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved.segment(3,3);
    controller.grf.value.segment(6,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved.segment(6,3);
    controller.grf.value.segment(9,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved.segment(9,3);
    // cout<<controller.grf.value.transpose()<<endl;
    // cout<<"----"<<endl;
}


MatrixNd QRobot::skewMat(VectorNd t){
	Matrix3d t_hat;
	t_hat<<0,-t(2), t(1)\
		,t(2), 0, -t(0)\
		,-t(1), t(0), 0;
	return t_hat;
}


void QRobot::KalmanFilter(double measure){
	static bool firstRun =true;

	if(firstRun==true){
		kalman.x_hat(0)=measure;
		kalman.x_hat(1)=0.0;
		kalman.p_hat(0,0)=0.0;
		kalman.p_hat(1,1)=0.0;
		firstRun=false;
	}

	kalman.A<< 1, dt\
			 , 0, 1;

	kalman.acc=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*CoM.local.acc.now;
		
	kalman.b<<(kalman.acc(2)-GRAVITY)*dt*dt\
			  ,(kalman.acc(2)-GRAVITY)*dt;
	
	kalman.H<< 1, 0;
 	kalman.R=30;
 	kalman.Q<<0.001,0\
 			,0,0.001;

	//Prediction
	kalman.x_bar=kalman.A*kalman.x_hat+kalman.b;
	kalman.p_bar=kalman.A*kalman.p_hat*kalman.A.transpose()+kalman.Q;
   
	//Kalman Gain
	kalman.gain=kalman.p_bar*kalman.H/(kalman.H.transpose()*kalman.p_bar*kalman.H+kalman.R);

	//Correction
	kalman.x_hat=kalman.x_bar+kalman.gain*(measure-kalman.H.transpose()*kalman.x_bar);
	kalman.p_hat=kalman.p_bar-kalman.gain*kalman.H.transpose()*kalman.p_bar.transpose();
	
}
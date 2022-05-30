#include "controller.h"

Controller::Controller(){
	std::cout<<"Class 'Controller' Called"<<std::endl;
	setParam();
}

void Controller::setParam() {
	RL.joint.kp<<400,0,0,0,200,0,0,0,200;
	RL.joint.kd<<15,0,0,0,10,0,0,0,10;
	RR.joint.kp<<400,0,0,0,200,0,0,0,200;
	RR.joint.kd<<15,0,0,0,10,0,0,0,10;
	FL.joint.kp<<400,0,0,0,200,0,0,0,200;
	FL.joint.kd<<15,0,0,0,10,0,0,0,10;
	FR.joint.kp<<400,0,0,0,200,0,0,0,200;
	FR.joint.kd<<15,0,0,0,10,0,0,0,10;

	RL.ep.kp<<2000,0,0,0,2000,0,0,0,1000;
	RL.ep.kd<<70,0,0,0,70,0,0,0,50;
	RR.ep.kp<<2000,0,0,0,2000,0,0,0,1000;
	RR.ep.kd<<70,0,0,0,70,0,0,0,50;
	FL.ep.kp<<2000,0,0,0,2000,0,0,0,1000;
	FL.ep.kd<<70,0,0,0,70,0,0,0,50;
	FR.ep.kp<<2000,0,0,0,2000,0,0,0,1000;
	FR.ep.kd<<70,0,0,0,70,0,0,0,50;

	controller.joint.value=VectorNd::Zero(12);
	controller.task.value=VectorNd::Zero(12);
	controller.grf.value=VectorNd::Zero(12);
	controller.grf.value2=VectorNd::Zero(12);
}

VectorNd Controller::Joint_PD_Controller(void){
	VectorNd Joint_PD(12);
	Joint_PD.segment(0,3)=RL.joint.kp*(RL.joint.pos.ref-RL.joint.pos.now)+RL.joint.kd*(RL.joint.vel.ref-RL.joint.vel.now);
	Joint_PD.segment(3,3)=RR.joint.kp*(RR.joint.pos.ref-RR.joint.pos.now)+RR.joint.kd*(RR.joint.vel.ref-RR.joint.vel.now);
	Joint_PD.segment(6,3)=FL.joint.kp*(FL.joint.pos.ref-FL.joint.pos.now)+FL.joint.kd*(FL.joint.vel.ref-FL.joint.vel.now);
	Joint_PD.segment(9,3)=FR.joint.kp*(FR.joint.pos.ref-FR.joint.pos.now)+FR.joint.kd*(FR.joint.vel.ref-FR.joint.vel.now);
	return Joint_PD;
}

VectorNd Controller::Compensation_Controller(void){
	VectorNd Comp_Torque_(12);
	VectorNd tmp_hatNonLinearEffects = VectorNd::Zero(18);
	VectorNd tmp_G_term = VectorNd::Zero(18);
	VectorNd tmp_C_term = VectorNd::Zero(18);

	NonlinearEffects(*rbdl.pModel, rbdl.RobotState, rbdl.RobotStateDot, tmp_hatNonLinearEffects);
	NonlinearEffects(*rbdl.pModel, rbdl.RobotState, VectorNd::Zero(rbdl.pModel->dof_count), tmp_G_term);
	tmp_C_term = tmp_hatNonLinearEffects - tmp_G_term;

	// C_term=tmp_C_term.segment(6,12);
	// G_term=tmp_G_term.segment(6,12);
	Comp_Torque_=tmp_C_term.segment(6,12)+tmp_G_term.segment(6,12);
	return Comp_Torque_;
}

VectorNd Controller::Task_PD_Controller(void){
	VectorNd Task_PD=VectorNd::Zero(12);

	VectorNd tmp_RL_F=VectorNd::Zero(3);
	VectorNd tmp_RR_F=VectorNd::Zero(3);
	VectorNd tmp_FL_F=VectorNd::Zero(3);
	VectorNd tmp_FR_F=VectorNd::Zero(3);

	// Base.pos.ref=GetBasePos(CoM.pos.ref, CoM.ori.R.ref);
	// Base.vel.ref=CoM.vel.ref;

	/************************************************************/
	RL.ep.local.semi.pos.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*RL.ep.local.pos.now;
	RR.ep.local.semi.pos.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*RR.ep.local.pos.now;
	FL.ep.local.semi.pos.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*FL.ep.local.pos.now;
	FR.ep.local.semi.pos.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*FR.ep.local.pos.now;

	RL.ep.local.semi.vel.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*RL.ep.local.vel.now;
	RR.ep.local.semi.vel.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*RR.ep.local.vel.now;
	FL.ep.local.semi.vel.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*FL.ep.local.vel.now;
	FR.ep.local.semi.vel.now=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*FR.ep.local.vel.now;

	/************************************************************/
	RL.ep.local.semi.pos.ref=CoM.ori.R.yaw.ref.transpose()*(RL.ep.global.pos.ref-Base.pos.ref);
	RR.ep.local.semi.pos.ref=CoM.ori.R.yaw.ref.transpose()*(RR.ep.global.pos.ref-Base.pos.ref);
	FL.ep.local.semi.pos.ref=CoM.ori.R.yaw.ref.transpose()*(FL.ep.global.pos.ref-Base.pos.ref);
	FR.ep.local.semi.pos.ref=CoM.ori.R.yaw.ref.transpose()*(FR.ep.global.pos.ref-Base.pos.ref);

	RL.ep.local.semi.vel.ref=CoM.ori.R.yaw.ref.transpose()*(RL.ep.global.vel.ref-Base.vel.ref);
	RR.ep.local.semi.vel.ref=CoM.ori.R.yaw.ref.transpose()*(RR.ep.global.vel.ref-Base.vel.ref);
	FL.ep.local.semi.vel.ref=CoM.ori.R.yaw.ref.transpose()*(FL.ep.global.vel.ref-Base.vel.ref);
	FR.ep.local.semi.vel.ref=CoM.ori.R.yaw.ref.transpose()*(FR.ep.global.vel.ref-Base.vel.ref);

	/************************************************************/
	tmp_RL_F=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*(RL.ep.kp*(RL.ep.local.semi.pos.ref-RL.ep.local.semi.pos.now)+RL.ep.kd*(RL.ep.local.semi.vel.ref-RL.ep.local.semi.vel.now));
	tmp_RR_F=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*(RR.ep.kp*(RR.ep.local.semi.pos.ref-RR.ep.local.semi.pos.now)+RR.ep.kd*(RR.ep.local.semi.vel.ref-RR.ep.local.semi.vel.now));
	tmp_FL_F=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*(FL.ep.kp*(FL.ep.local.semi.pos.ref-FL.ep.local.semi.pos.now)+FL.ep.kd*(FL.ep.local.semi.vel.ref-FL.ep.local.semi.vel.now));
	tmp_FR_F=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*(FR.ep.kp*(FR.ep.local.semi.pos.ref-FR.ep.local.semi.pos.now)+FR.ep.kd*(FR.ep.local.semi.vel.ref-FR.ep.local.semi.vel.now));
    // tmp_RL_F=RL.kp*(RL.local.pos.ref-RL.local.pos.now)+RL.kd*(RL.local.vel.ref-RL.local.vel.now);
    // tmp_RR_F=RR.kp*(RR.local.pos.ref-RR.local.pos.now)+RR.kd*(RR.local.vel.ref-RR.local.vel.now);
    // tmp_FL_F=FL.kp*(FL.local.pos.ref-FL.local.pos.now)+FL.kd*(FL.local.vel.ref-FL.local.vel.now);
    // tmp_FR_F=FR.kp*(FR.local.pos.ref-FR.local.pos.now)+FR.kd*(FR.local.vel.ref-FR.local.vel.now);
	
	Task_PD<<tmp_RL_F,tmp_RR_F,tmp_FL_F,tmp_FR_F;

	return Task_PD;
}



void Controller::TorqueControl(void){

	if(CommandFlag==GOTO_INIT_POSE){
		CTC_Torque=Joint_PD_Controller()+Compensation_Controller();
	}else{
		// controller.joint.value=VectorNd::Zero(12);
		// controller.task.value=Task_PD_Controller();	
		// QuadPP_Controller_Stance();
    	// QuadPP_Controller_Fly();
    	// CTC_Torque=controller.joint.value;
		CTC_Torque=Compensation_Controller()+J_A.transpose()*Task_PD_Controller();
    	//CTC_Torque=Joint_PD_Controller()+Compensation_Controller();
	}
    //CTC_Torque=G_term+C_term+controller.joint.value+J_A.transpose()*(controller.task.value)-J_A.transpose()*(controller.grf.value);
	


    //CTC_Torque=G_term+C_term+controller.joint.value-J_A.transpose()*(controller.grf.value)+J_A.transpose()*(controller.grf.value2);
	//cout<<"CTC:"<<CTC_Torque.transpose()<<endl;
	//cout<<"---"<<endl;
	
	RL.joint.torque.ref=CTC_Torque.segment(0,3);
	RR.joint.torque.ref=CTC_Torque.segment(3,3);
	FL.joint.torque.ref=CTC_Torque.segment(6,3);
	FR.joint.torque.ref=CTC_Torque.segment(9,3);

}

void Controller::setQPParam(void){
	controller.grf.qp.kp =VectorNd::Zero(6);
	controller.grf.qp.kd =VectorNd::Zero(6);
	controller.grf.qp.kp_mat=MatrixNd::Zero(6,6);
	controller.grf.qp.kd_mat=MatrixNd::Zero(6,6);
	controller.grf.qp.StateErr=VectorNd::Zero(6);
	controller.grf.qp.StateDotErr=VectorNd::Zero(6);
	controller.grf.qp.Inertia_term=VectorNd::Zero(6);
	// controller.grf.qp.kp << 10, 10, 4000, 3000, 3000, 0;
 //    controller.grf.qp.kd << 0.1, 0.1, 40, 30, 30, 10;

	controller.grf.qp.kp << 2000, 2000, 2000, 1500, 1500, 0;
	controller.grf.qp.kd << 100, 100, 100, 30, 30, 10;

	controller.grf.qp.kp_mat=controller.grf.qp.kp.asDiagonal();
	controller.grf.qp.kd_mat=controller.grf.qp.kd.asDiagonal();

	controller.grf.qp.n=12;
	controller.grf.qp.m=0;
	controller.grf.qp.p=24;

	controller.grf.qp.A_qp=MatrixNd::Zero(6,12);
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





void Controller::QuadPP_Controller_Stance(void){
	VectorNd CoM_ref(3);
	VectorNd CoM_now(3);
	VectorNd CoM_vel(3);
	double alpha=0.001;
	// double alpha=1.0;

	RL.ep.local.pos.now_from_com=RL.ep.local.pos.now-offset_B2C;
	RR.ep.local.pos.now_from_com=RR.ep.local.pos.now-offset_B2C;
	FL.ep.local.pos.now_from_com=FL.ep.local.pos.now-offset_B2C;
	FR.ep.local.pos.now_from_com=FR.ep.local.pos.now-offset_B2C;


	RL.ep.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(RL.ep.local.pos.now_from_com);
	RR.ep.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(RR.ep.local.pos.now_from_com);
	FL.ep.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(FL.ep.local.pos.now_from_com);
	FR.ep.local.semi.pos.now_from_com=CoM.ori.R.pitch.now*CoM.ori.R.roll.now*(FR.ep.local.pos.now_from_com);
    // CoM_now=-(RL.Contact*RL.local.semi.pos.now_from_com+RR.Contact*RR.local.semi.pos.now_from_com+FL.Contact*FL.local.semi.pos.now_from_com+FR.Contact*FR.local.semi.pos.now_from_com)/(RL.Contact+RR.Contact+FL.Contact+FR.Contact);
	
	// RL.global.semi.pos.now=CoM.pos.ref+CoM.ori.R.pitch.now*CoM.ori.R.roll.now*RL.local.pos.now;
	// RR.global.semi.pos.now=CoM.pos.ref+CoM.ori.R.pitch.now*CoM.ori.R.roll.now*RR.local.pos.now;
	// FL.global.semi.pos.now=CoM.pos.ref+CoM.ori.R.pitch.now*CoM.ori.R.roll.now*FL.local.pos.now;
	// FR.global.semi.pos.now=CoM.pos.ref+CoM.ori.R.pitch.now*CoM.ori.R.roll.now*FR.local.pos.now;

	// RL.global.semi.pos.now=CoM.pos.now+RL.local.semi.pos.now_from_com;
	// RR.global.semi.pos.now=CoM.pos.now+RR.local.semi.pos.now_from_com;
	// FL.global.semi.pos.now=CoM.pos.now+FL.local.semi.pos.now_from_com;
	// FR.global.semi.pos.now=CoM.pos.now+FR.local.semi.pos.now_from_com;
	RL.ep.global.semi.pos.now=RL.ep.local.semi.pos.now_from_com;
	RR.ep.global.semi.pos.now=RR.ep.local.semi.pos.now_from_com;
	FL.ep.global.semi.pos.now=FL.ep.local.semi.pos.now_from_com;
	FR.ep.global.semi.pos.now=FR.ep.local.semi.pos.now_from_com;

	RL.ep.global.semi.vel.now=RL.ep.local.vel.now;
	RR.ep.global.semi.vel.now=RR.ep.local.vel.now;
	FL.ep.global.semi.vel.now=FL.ep.local.vel.now;
	FR.ep.global.semi.vel.now=FR.ep.local.vel.now;

	CoM_now=-(RL.contact.ref*RL.ep.global.semi.pos.now+RR.contact.ref*RR.ep.global.semi.pos.now+FL.contact.ref*FL.ep.global.semi.pos.now+FR.contact.ref*FR.ep.global.semi.pos.now)/(RL.contact.ref+RR.contact.ref+FL.contact.ref+FR.contact.ref);
	CoM_vel=-(RL.contact.ref*RL.ep.global.semi.vel.now+RR.contact.ref*RR.ep.global.semi.vel.now+FL.contact.ref*FL.ep.global.semi.vel.now+FR.contact.ref*FR.ep.global.semi.vel.now)/(RL.contact.ref+RR.contact.ref+FL.contact.ref+FR.contact.ref);

	controller.grf.qp.A_qp.block(0,0,3,12) << RL.contact.ref, 0, 0, RR.contact.ref, 0, 0, FL.contact.ref, 0, 0, FR.contact.ref, 0, 0\
	, 0, RL.contact.ref, 0, 0, RR.contact.ref, 0, 0, FL.contact.ref, 0, 0, FR.contact.ref, 0\
	, 0, 0, RL.contact.ref, 0, 0, RR.contact.ref, 0, 0, FL.contact.ref, 0, 0, FR.contact.ref;
	controller.grf.qp.A_qp.block(3,0,3,3)=RL.contact.ref*skewMat(RL.ep.local.semi.pos.now_from_com);
	controller.grf.qp.A_qp.block(3,3,3,3)=RR.contact.ref*skewMat(RR.ep.local.semi.pos.now_from_com);
	controller.grf.qp.A_qp.block(3,6,3,3)=FL.contact.ref*skewMat(FL.ep.local.semi.pos.now_from_com);
	controller.grf.qp.A_qp.block(3,9,3,3)=FR.contact.ref*skewMat(FR.ep.local.semi.pos.now_from_com);

    /*********/
	RL.ep.local.pos.ref=CoM.ori.R.ref.transpose()*(RL.ep.global.pos.ref-Base.pos.ref);
	RR.ep.local.pos.ref=CoM.ori.R.ref.transpose()*(RR.ep.global.pos.ref-Base.pos.ref);
	FL.ep.local.pos.ref=CoM.ori.R.ref.transpose()*(FL.ep.global.pos.ref-Base.pos.ref);
	FR.ep.local.pos.ref=CoM.ori.R.ref.transpose()*(FR.ep.global.pos.ref-Base.pos.ref);
	RL.ep.local.pos.ref_from_com=RL.ep.local.pos.ref-offset_B2C;
	RR.ep.local.pos.ref_from_com=RR.ep.local.pos.ref-offset_B2C;
	FL.ep.local.pos.ref_from_com=FL.ep.local.pos.ref-offset_B2C;
	FR.ep.local.pos.ref_from_com=FR.ep.local.pos.ref-offset_B2C;
	RL.ep.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(RL.ep.local.pos.ref_from_com);
	RR.ep.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(RR.ep.local.pos.ref_from_com);
	FL.ep.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(FL.ep.local.pos.ref_from_com);
	FR.ep.local.semi.pos.ref_from_com=CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref*(FR.ep.local.pos.ref_from_com);
	CoM_ref=-(RL.contact.ref*RL.ep.local.semi.pos.ref_from_com+RR.contact.ref*RR.ep.local.semi.pos.ref_from_com+FL.contact.ref*FL.ep.local.semi.pos.ref_from_com+FR.contact.ref*FR.ep.local.semi.pos.ref_from_com)/(RL.contact.ref+RR.contact.ref+FL.contact.ref+FR.contact.ref);

 //    cout<<"ref:"<<CoM_ref.transpose()<<endl;
	// cout<<"now:"<<CoM_now.transpose()<<endl;
	// cout<<"now1:"<<CoM.pos.now.transpose()<<endl;
	// cout<<"now2:"<<((RL.local.semi.pos.now_from_com+RR.local.semi.pos.now_from_com+FL.local.semi.pos.now_from_com+FR.local.semi.pos.now_from_com)/4.0).transpose()<<endl;
	// cout<<"---"<<endl;

	controller.grf.qp.StateErr(0)=CoM_ref(0)-CoM_now(0);
	controller.grf.qp.StateErr(1)=CoM_ref(1)-CoM_now(1);
	controller.grf.qp.StateErr(2)=CoM.pos.goal(2)/cos(abs(CoM.ori.euler.pos.ref(1)))-CoM.pos.now(2)/cos(abs(CoM.ori.euler.pos.now(1)));
	controller.grf.qp.StateErr(3)=CoM.ori.euler.pos.ref(0)-CoM.ori.euler.pos.now(0);
	controller.grf.qp.StateErr(4)=CoM.ori.euler.pos.ref(1)-CoM.ori.euler.pos.now(1);
	controller.grf.qp.StateErr(5)=0.0;

	controller.grf.qp.StateDotErr(0)=0.0-CoM_vel(0);
	controller.grf.qp.StateDotErr(1)=0.0-CoM_vel(1);
	controller.grf.qp.StateDotErr(2)=0.0-CoM.vel.now(2);
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

	controller.grf.qp.tmp_CI.block(0,0,1,3)=RL.contact.ref*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(1,0,1,3)=RL.contact.ref*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(2,0,1,3)=RL.contact.ref*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(3,0,1,3)=RL.contact.ref*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(4,0,1,3)=RL.contact.ref*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(5,0,1,3)<<0,0,-RL.contact.ref;

	controller.grf.qp.tmp_CI.block(6,3,1,3)=RR.contact.ref*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(7,3,1,3)=RR.contact.ref*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(8,3,1,3)=RR.contact.ref*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(9,3,1,3)=RR.contact.ref*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(10,3,1,3)=RR.contact.ref*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(11,3,1,3)<<0,0,-RR.contact.ref;

	controller.grf.qp.tmp_CI.block(12,6,1,3)<<FL.contact.ref*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(13,6,1,3)<<FL.contact.ref*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(14,6,1,3)<<FL.contact.ref*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(15,6,1,3)<<FL.contact.ref*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(16,6,1,3)<<FL.contact.ref*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(17,6,1,3)<<0,0,-FL.contact.ref;

	controller.grf.qp.tmp_CI.block(18,9,1,3)<<FR.contact.ref*controller.grf.qp.n_semi_global.transpose();
	controller.grf.qp.tmp_CI.block(19,9,1,3)<<FR.contact.ref*(-controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(20,9,1,3)<<FR.contact.ref*(controller.grf.qp.t_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(21,9,1,3)<<FR.contact.ref*(-controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(22,9,1,3)<<FR.contact.ref*(controller.grf.qp.b_semi_global+controller.grf.qp.mu*controller.grf.qp.n_semi_global).transpose();
	controller.grf.qp.tmp_CI.block(23,9,1,3)<<0,0,-FR.contact.ref;

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
}

void Controller::QuadPP_Controller_Fly(void){
	Eigen::Vector3d EP_acc_ref;
	Eigen::Vector3d EP_pos_ref;
	Eigen::Vector3d EP_pos;
	Eigen::Vector3d EP_vel_ref;
	Eigen::Vector3d EP_vel;
	double alpha=0.001;

	if(RL.contact.ref==0){
		EP_acc_ref=CoM.ori.R.yaw.ref.transpose()*RL.ep.global.acc.ref;
		EP_pos_ref=RL.ep.local.semi.pos.ref;
		EP_pos=RL.ep.local.semi.pos.now;
		EP_vel_ref=RL.ep.local.semi.vel.ref;
		EP_vel=RL.ep.local.semi.vel.now;

	}else if(RR.contact.ref==0){
		EP_acc_ref=CoM.ori.R.yaw.ref.transpose()*RR.ep.global.acc.ref;
		EP_pos_ref=RR.ep.local.semi.pos.ref;
		EP_pos=RR.ep.local.semi.pos.now;
		EP_vel_ref=RR.ep.local.semi.vel.ref;
		EP_vel=RR.ep.local.semi.vel.now;

	}else if(FL.contact.ref==0){
		EP_acc_ref=CoM.ori.R.yaw.ref.transpose()*FL.ep.global.acc.ref;
		EP_pos_ref=FL.ep.local.semi.pos.ref;
		EP_pos=FL.ep.local.semi.pos.now;
		EP_vel_ref=FL.ep.local.semi.vel.ref;
		EP_vel=FL.ep.local.semi.vel.now;
	}else if(FR.contact.ref==0){
		EP_acc_ref=CoM.ori.R.yaw.ref.transpose()*FR.ep.global.acc.ref;
		EP_pos_ref=FR.ep.local.semi.pos.ref;
		EP_pos=FR.ep.local.semi.pos.now;
		EP_vel_ref=FR.ep.local.semi.vel.ref;
		EP_vel=FR.ep.local.semi.vel.now;
	}
	if(RL.contact.ref==1&&RR.contact.ref==1&&FL.contact.ref==1&&FR.contact.ref==1){
		EP_acc_ref=VectorNd::Zero(3);
		EP_pos_ref=VectorNd::Zero(3);
		EP_pos=VectorNd::Zero(3);
		EP_vel_ref=VectorNd::Zero(3);
		EP_vel=VectorNd::Zero(3);
	}

	controller.grf.qp.A_qp2.block(0,0,3,12) << (1-RL.contact.ref), 0, 0, (1-RR.contact.ref), 0, 0, (1-FL.contact.ref), 0, 0, (1-FR.contact.ref), 0, 0\
	, 0, (1-RL.contact.ref), 0, 0, (1-RR.contact.ref), 0, 0, (1-FL.contact.ref), 0, 0, (1-FR.contact.ref), 0\
	, 0, 0, (1-RL.contact.ref), 0, 0, (1-RR.contact.ref), 0, 0, (1-FL.contact.ref), 0, 0, (1-FR.contact.ref);
	// cout<<controller.grf.qp.A_qp2<<endl;
	// cout<<"----"<<endl;
	controller.grf.qp.StateErr2=EP_pos_ref-EP_pos;	
	controller.grf.qp.StateDotErr2=EP_vel_ref-EP_vel;

	controller.grf.qp.Inertia_term2<<Leg_mass*EP_acc_ref(0),Leg_mass*EP_acc_ref(1),Leg_mass*EP_acc_ref(2)+Leg_mass*GRAVITY;
	//controller.grf.qp.b_qp2=controller.grf.qp.Inertia_term2+controller.grf.qp.kp_mat2*controller.grf.qp.StateErr2+controller.grf.qp.kd_mat2*controller.grf.qp.StateDotErr2;
	controller.grf.qp.b_qp2=controller.grf.qp.kp_mat2*controller.grf.qp.StateErr2+controller.grf.qp.kd_mat2*controller.grf.qp.StateDotErr2;

	controller.grf.qp.tmp_G2=2*(controller.grf.qp.A_qp2.transpose()*controller.grf.qp.A_qp2+alpha*controller.grf.qp.W);
	controller.grf.qp.tmp_g02=-2*(controller.grf.qp.A_qp2.transpose()*controller.grf.qp.b_qp2);


	for (int i = 0; i < controller.grf.qp.n2; i++) {
		for (int j = 0; j < controller.grf.qp.n2; j++) {
			controller.grf.qp.G2[i][j] = controller.grf.qp.tmp_G2(i, j);
		}
	}

	for (int i = 0; i < controller.grf.qp.n2; i++) {
		controller.grf.qp.g02[i] = controller.grf.qp.tmp_g02[i];
	}


	for (int i = 0; i < controller.grf.qp.n2; i++) {
		for (int j = 0; j < controller.grf.qp.m2; j++) {
			controller.grf.qp.CE2[i][j] = 0;
		}
	}

	for (int j = 0; j < controller.grf.qp.m2; j++) {
		controller.grf.qp.ce02[j] = 0;
	}

	for (int i = 0; i < controller.grf.qp.n2; i++) {
		for (int j = 0; j < controller.grf.qp.p2; j++) {
			controller.grf.qp.CI2[i][j] = 0;
		}
	}


	// for (int i = 0; i < controller.grf.qp.n; i++) {
	// 	for (int j = 0; j < controller.grf.qp.p; j++) {
	// 		controller.grf.qp.CI[i][j] = controller.grf.qp.tmp_CI(j, i);
	// 	}
	// }

	for (int i=0; i<controller.grf.qp.p2;i++){
		controller.grf.qp.ci02[i]=0;
	}

	if (isfinite(solve_quadprog(controller.grf.qp.G2, controller.grf.qp.g02, controller.grf.qp.CE2, controller.grf.qp.ce02, controller.grf.qp.CI2, controller.grf.qp.ci02, controller.grf.qp.x2))) {
		for (int i = 0; i < controller.grf.qp.n2; ++i) {
			controller.grf.qp.x_saved2(i) = controller.grf.qp.x2[i];
		}
	}

	controller.grf.value2.segment(0,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved2.segment(0,3);
	controller.grf.value2.segment(3,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved2.segment(3,3);
	controller.grf.value2.segment(6,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved2.segment(6,3);
	controller.grf.value2.segment(9,3)=(CoM.ori.R.pitch.ref*CoM.ori.R.roll.ref).transpose()*controller.grf.qp.x_saved2.segment(9,3);
    // cout<<"Err2:"<<controller.grf.qp.StateErr2.transpose()<<endl;
    // cout<<"ref:"<<EP_pos_ref.transpose()<<endl;
    // cout<<"now:"<<EP_pos.transpose()<<endl;
    // cout<<"F2:"<<controller.grf.qp.x_saved2.transpose()<<endl;
    // cout<<"---"<<endl;
}
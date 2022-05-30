#include "QRobot.h"

QRobot::QRobot() {
	cout<<"Class QRobot Called"<<endl;
	ControlMode=CTRLMODE_INIT_POSE;

	setPath();
	setPreviewParam();
	setQPParam();
	
	setKalmanParam();
	setDataSave();

}

QRobot::~QRobot() {

}

void QRobot::Robot_RUN(){

	switch (ControlMode){
		case CTRLMODE_NONE:
		
		break;
		
		case CTRLMODE_INIT_POSE:
		printf("CTRLMODE_INIT_POSE\n");
		resetTrajPram();
		CommandFlag=GOTO_INIT_POSE;
		ControlMode=CTRLMODE_NONE;
		break;

		case CTRLMODE_WALK_READY:
		printf("CTRLMODE_HOME_POSE\n");
		resetTrajPram();
		CommandFlag=GOTO_READY_POSE;
		ControlMode=CTRLMODE_NONE;
		break;

		case CTRLMODE_SLOW_WALK:
		printf("CTRLMODE_SLOW_WALK\n");
		resetTrajPram();
		CommandFlag=GOTO_SLOW_WALK;
		ControlMode=CTRLMODE_NONE;
		break;
	}

	switch (CommandFlag){
		case NO_ACT:
		break;

		case GOTO_INIT_POSE:
		StateUpdate();
		Init_Pose_Traj();
		TorqueControl();
		break;

		case GOTO_READY_POSE:
		StateUpdate();
		// Init_Pose_Traj();
		Ready_Pose_Traj();
		TorqueControl();
		break;

		case GOTO_SLOW_WALK:
		StateUpdate();
		Slow_Walk_Traj();
		TorqueControl();
		break;
	}
}
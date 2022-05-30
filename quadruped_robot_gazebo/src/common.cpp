#include "common.h"

Common::Common() {
	cout<<"Class 'Common' called"<<endl;
	offset_B2C<<-0.05, 0.0, -0.0;
	setPath();
	setKalmanParam();
}

void Common::setPath(void){
	filepath.URDF="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_description/urdf/quadruped_robot.urdf";
	filepath.PREVIEW_GAIN_Gp="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/gain/Gp.txt";
	filepath.PREVIEW_GAIN_Gx="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/gain/Gx.txt";
	filepath.PREVIEW_GAIN_Gi="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/gain/Gi.txt";
	filepath.FILE_SAVE="/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/data/data.txt";
}

void Common::setKalmanParam(void){
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


VectorNd Common::GetComPos(VectorNd Base_Pos_, MatrixNd R_WB_) {
	VectorNd Com_Pos_(3);

	Com_Pos_ = Base_Pos_ + R_WB_*offset_B2C;
	return Com_Pos_;
}

VectorNd Common::GetBasePos(VectorNd Com_Pos_, MatrixNd R_WB_) {
	VectorNd Base_Pos_(3);
	Base_Pos_ = Com_Pos_ - R_WB_*offset_B2C;
	return Base_Pos_;
}

VectorNd Common::FK(VectorNd JointState_){
	VectorNd EP_State_(12);
	VectorNd Offset_Base2Hip(3);
	const double L1=0.1045;
	const double L2=0.305;
	const double L3=0.309;

	double q1=0.0;
	double q2=0.0;
	double q3=0.0;
	Offset_Base2Hip <<0.35,0.115,-0.053;

	q1=JointState_[0];
	q2=JointState_[1];
	q3=JointState_[2];

	EP_State_[0] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	EP_State_[1] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	EP_State_[2] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	EP_State_[0]=EP_State_[0]-Offset_Base2Hip(0);
	EP_State_[1]=EP_State_[1]+Offset_Base2Hip(1);
	EP_State_[2]=EP_State_[2]+Offset_Base2Hip(2);

    //RR_EP
	q1 = JointState_[3];
	q2 = JointState_[4];
	q3 = JointState_[5];
	EP_State_[3] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	EP_State_[4] = -L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	EP_State_[5] = -L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	EP_State_[3]=EP_State_[3]-Offset_Base2Hip(0);
	EP_State_[4]=EP_State_[4]-Offset_Base2Hip(1);
	EP_State_[5]=EP_State_[5]+Offset_Base2Hip(2);

    //FL_EP
	q1 = JointState_[6];
	q2 = JointState_[7];
	q3 = JointState_[8];
	EP_State_[6] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	EP_State_[7] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	EP_State_[8] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	EP_State_[6]=EP_State_[6]+Offset_Base2Hip(0);
	EP_State_[7]=EP_State_[7]+Offset_Base2Hip(1);
	EP_State_[8]=EP_State_[8]+Offset_Base2Hip(2);

    //FR_EP
	q1 = JointState_[9];
	q2 = JointState_[10];
	q3 = JointState_[11];
	EP_State_[9] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
	EP_State_[10] = -L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
	EP_State_[11] = -L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

	EP_State_[9]=EP_State_[9]+Offset_Base2Hip(0);
	EP_State_[10]=EP_State_[10]-Offset_Base2Hip(1);
	EP_State_[11]=EP_State_[11]+Offset_Base2Hip(2);

	return EP_State_;
}

MatrixNd Common::skewMat(VectorNd vec_){
	Matrix3d Mat_;
	Mat_<<0,-vec_(2), vec_(1)\
	,vec_(2), 0, -vec_(0)\
	,-vec_(1), vec_(0), 0;
	return Mat_;
}

void Common::KalmanFilter(double measure_){
	static bool firstRun =true;

	if(firstRun==true){
		kalman.x_hat(0)=measure_;
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
	kalman.x_hat=kalman.x_bar+kalman.gain*(measure_-kalman.H.transpose()*kalman.x_bar);
	kalman.p_hat=kalman.p_bar-kalman.gain*kalman.H.transpose()*kalman.p_bar.transpose();
	
}

void Common::getRotationMatrix(void){
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

void Common::setDataSave(void){
	datasave.SAVE_LENGTH=12;
	datasave.SAVE_TIME=60;
	datasave.SAVE_COUNT =datasave.SAVE_TIME*1000;
	datasave.save_cnt = 0;

	datasave.save_array=new double*[datasave.SAVE_COUNT];
	for(int i=0; i<datasave.SAVE_COUNT; i++){
		datasave.save_array[i]=new double[datasave.SAVE_LENGTH];
	}
}

void Common::DataSave(void) {
	datasave.save_array[datasave.save_cnt][0] = RL.joint.torque.ref[0];
	datasave.save_array[datasave.save_cnt][1] = RL.joint.torque.ref[1];
	datasave.save_array[datasave.save_cnt][2] = RL.joint.torque.ref[2];
	datasave.save_array[datasave.save_cnt][3] = RL.joint.vel.now[0];
	datasave.save_array[datasave.save_cnt][4] = RL.joint.vel.now[1];
	datasave.save_array[datasave.save_cnt][5] = RL.joint.vel.now[2];

	datasave.save_array[datasave.save_cnt][6] = FR.joint.torque.ref[0];
	datasave.save_array[datasave.save_cnt][7] = FR.joint.torque.ref[1];
	datasave.save_array[datasave.save_cnt][8] = FR.joint.torque.ref[2];
	datasave.save_array[datasave.save_cnt][9] = FR.joint.vel.now[0];
	datasave.save_array[datasave.save_cnt][10] = FR.joint.vel.now[1];
	datasave.save_array[datasave.save_cnt][11] = FR.joint.vel.now[2];

	if (datasave.save_cnt < datasave.SAVE_COUNT - 1){
		datasave.save_cnt++;
		if(datasave.save_cnt == datasave.SAVE_COUNT - 1){
			cout<<"Save End"<<endl;	
		}
	}
}
// #include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <stdio.h>
#include <iostream>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include "QRobot.h" 


Model* qbot_model=new Model();

using namespace std;
namespace gazebo
{
 class qrobot_plugin:public ModelPlugin
 {
 	
	physics::ModelPtr model;

	physics::LinkPtr BODY;
	physics::LinkPtr RL_HIP;
	physics::LinkPtr RL_THIGH;
	physics::LinkPtr RL_CALF;
	physics::LinkPtr RL_TIP;
	physics::LinkPtr RR_HIP;
	physics::LinkPtr RR_THIGH;
	physics::LinkPtr RR_CALF;
	physics::LinkPtr RR_TIP;
	physics::LinkPtr FL_HIP;
	physics::LinkPtr FL_THIGH;
	physics::LinkPtr FL_CALF;
	physics::LinkPtr FL_TIP;
	physics::LinkPtr FR_HIP;
	physics::LinkPtr FR_THIGH;
	physics::LinkPtr FR_CALF;
	physics::LinkPtr FR_TIP;

	physics::JointPtr RL_HR_JOINT;
	physics::JointPtr RL_HP_JOINT;
	physics::JointPtr RL_KN_JOINT;
	physics::JointPtr RL_TIP_JOINT;
	physics::JointPtr RR_HR_JOINT;
	physics::JointPtr RR_HP_JOINT;
	physics::JointPtr RR_KN_JOINT;
	physics::JointPtr RR_TIP_JOINT;
	physics::JointPtr FL_HR_JOINT;
	physics::JointPtr FL_HP_JOINT;
	physics::JointPtr FL_KN_JOINT;
	physics::JointPtr FL_TIP_JOINT;
	physics::JointPtr FR_HR_JOINT;
	physics::JointPtr FR_HP_JOINT;
	physics::JointPtr FR_KN_JOINT;
	physics::JointPtr FR_TIP_JOINT;

	sensors::SensorPtr Sensor;
    sensors::ImuSensorPtr IMU;

	event::ConnectionPtr update_connection;
	ros::NodeHandle nh;
	ros::Publisher P_data;
	std_msgs::Float64MultiArray m_data;
	ros::Subscriber server_sub;
	ros::Subscriber sub_gui;
	ros::Publisher pub_gui;

	QRobot Qbot;
	
	public:
	void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);	
	void UpdateAlgorithm(void);
	void SensorSetting(void);
	void RBDLSetting(void);
	void InitROSCOMM(void);
	void GetJoints(void);
	void GetLinks(void);
	void EncoderRead(void);
	void IMUSensorRead(void);
	void jointController(void);
	void Callback(const std_msgs::UInt16 &msg);
	void GUICallback(const std_msgs::UInt16 &msg);
	void ROSMsgPub();
	private :
 };
 GZ_REGISTER_MODEL_PLUGIN(qrobot_plugin);
}

void gazebo::qrobot_plugin::GetLinks()
 {
	this->BODY = this->model->GetLink("BODY");
  
    this->RL_HIP = this->model->GetLink("RL_HIP");
    this->RL_THIGH = this->model->GetLink("RL_THIGH");
    this->RL_CALF = this->model->GetLink("RL_CALF");
    this->RL_TIP = this->model->GetLink("RL_TIP");

    this->RR_HIP = this->model->GetLink("RR_HIP");
    this->RR_THIGH = this->model->GetLink("RR_THIGH");
    this->RR_CALF = this->model->GetLink("RR_CALF");
    this->RR_TIP = this->model->GetLink("RR_TIP");

    this->FL_HIP = this->model->GetLink("FL_HIP");
    this->FL_THIGH = this->model->GetLink("FL_THIGH");
    this->FL_CALF = this->model->GetLink("FL_CALF");
    this->FL_TIP = this->model->GetLink("FL_TIP");

    this->FR_HIP = this->model->GetLink("FR_HIP");
    this->FR_THIGH = this->model->GetLink("FR_THIGH");
    this->FR_CALF = this->model->GetLink("FR_CALF");
    this->FR_TIP = this->model->GetLink("FR_TIP");
}

void gazebo::qrobot_plugin::GetJoints(){
	this->RL_HR_JOINT = this->model->GetJoint("RL_HR_JOINT");
	this->RL_HP_JOINT = this->model->GetJoint("RL_HP_JOINT");
	this->RL_KN_JOINT = this->model->GetJoint("RL_KN_JOINT");
	this->RL_TIP_JOINT = this->model->GetJoint("RL_TIP_JOINT");

	this->RR_HR_JOINT = this->model->GetJoint("RR_HR_JOINT");
	this->RR_HP_JOINT = this->model->GetJoint("RR_HP_JOINT");
	this->RR_KN_JOINT = this->model->GetJoint("RR_KN_JOINT");
	this->RR_TIP_JOINT = this->model->GetJoint("RR_TIP_JOINT");

	this->FL_HR_JOINT = this->model->GetJoint("FL_HR_JOINT");
	this->FL_HP_JOINT = this->model->GetJoint("FL_HP_JOINT");
	this->FL_KN_JOINT = this->model->GetJoint("FL_KN_JOINT");
	this->FL_TIP_JOINT = this->model->GetJoint("FL_TIP_JOINT");

	this->FR_HR_JOINT = this->model->GetJoint("FR_HR_JOINT");
	this->FR_HP_JOINT = this->model->GetJoint("FR_HP_JOINT");
	this->FR_KN_JOINT = this->model->GetJoint("FR_KN_JOINT");
	this->FR_TIP_JOINT = this->model->GetJoint("FR_TIP_JOINT");
}

void gazebo::qrobot_plugin::SensorSetting(){
	this->Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
}

void gazebo::qrobot_plugin::RBDLSetting(){
	Addons::URDFReadFromFile("/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_description/urdf/quadruped_robot.urdf", qbot_model, true, true);
    Qbot.setRobotModel(qbot_model);
}

void gazebo::qrobot_plugin::EncoderRead()
{
    //************************** Encoder ********************************//
	Qbot.joint[0].pos.now= this->RL_HR_JOINT->Position(0);
	Qbot.joint[1].pos.now= this->RL_HP_JOINT->Position(0);
	Qbot.joint[2].pos.now= this->RL_KN_JOINT->Position(0);

	Qbot.joint[3].pos.now= this->RR_HR_JOINT->Position(0);
	Qbot.joint[4].pos.now= this->RR_HP_JOINT->Position(0);
	Qbot.joint[5].pos.now= this->RR_KN_JOINT->Position(0);

	Qbot.joint[6].pos.now= this->FL_HR_JOINT->Position(0);
	Qbot.joint[7].pos.now= this->FL_HP_JOINT->Position(0);
	Qbot.joint[8].pos.now= this->FL_KN_JOINT->Position(0);

	Qbot.joint[9].pos.now= this->FR_HR_JOINT->Position(0);
	Qbot.joint[10].pos.now= this->FR_HP_JOINT->Position(0);
	Qbot.joint[11].pos.now= this->FR_KN_JOINT->Position(0);

    for (int i = 0; i < nDOF; i++) {
    	Qbot.joint[i].vel.now = (Qbot.joint[i].pos.now - Qbot.joint[i].pos.prev) / dt;
    	Qbot.joint[i].acc.now = (Qbot.joint[i].vel.now - Qbot.joint[i].vel.prev) / dt;
    	
    	Qbot.joint[i].pos.prev=Qbot.joint[i].pos.now;
    	Qbot.joint[i].vel.prev=Qbot.joint[i].vel.now;
    }
}

void gazebo::qrobot_plugin::IMUSensorRead()
{
    Qbot.CoM.ori.euler.vel.now(0) = this->IMU->AngularVelocity(false)[0];
    Qbot.CoM.ori.euler.vel.now(1) = this->IMU->AngularVelocity(false)[1];
    Qbot.CoM.ori.euler.vel.now(2) = this->IMU->AngularVelocity(false)[2];
    Qbot.CoM.ori.euler.pos.now = Qbot.CoM.ori.euler.pos.now + Qbot.CoM.ori.euler.vel.now*dt;
    
    Qbot.CoM.acc.now(0) =this->IMU->LinearAcceleration(false)[0];
    Qbot.CoM.acc.now(1) =this->IMU->LinearAcceleration(false)[1];
    Qbot.CoM.acc.now(2) =this->IMU->LinearAcceleration(false)[2];

    Qbot.getRotationMatrix();

    // cout<<Qbot.CoM.ori.euler.R.now<<endl;
    // cout<<"-----"<<endl;

    // cout<<Qbot.CoM.ori.euler.pos.now.transpose()<<endl;
    // cout<<Qbot.CoM.ori.euler.vel.now.transpose()<<endl;
    // cout<<Qbot.CoM.acc.now.transpose()<<endl;
    // cout<<"---"<<endl;

}

void gazebo::qrobot_plugin::jointController()
{
    //***************************Set Torque********************************//

    //* Torque Limit
    for (unsigned int i = 0; i < 13; ++i) {
        if (Qbot.joint[i].torque.ref >= 3000) {
            Qbot.joint[i].torque.ref = 3000;
        }
        else if (Qbot.joint[i].torque.ref <= -3000) {
            Qbot.joint[i].torque.ref = -3000;
        }
    }

    //* Applying torques
    this->RL_HR_JOINT->SetForce(0, Qbot.joint[0].torque.ref); //PongBotQ.target_tor[0]);
    this->RL_HP_JOINT->SetForce(0, Qbot.joint[1].torque.ref); //PongBotQ.target_tor[1]);
    this->RL_KN_JOINT->SetForce(0, Qbot.joint[2].torque.ref); //PongBotQ.target_tor[2]);

    this->RR_HR_JOINT->SetForce(0, Qbot.joint[3].torque.ref); //PongBotQ.target_tor[3]);
    this->RR_HP_JOINT->SetForce(0, Qbot.joint[4].torque.ref); //PongBotQ.target_tor[4]);
    this->RR_KN_JOINT->SetForce(0, Qbot.joint[5].torque.ref); //PongBotQ.target_tor[5]);

    this->FL_HR_JOINT->SetForce(0, Qbot.joint[6].torque.ref); //PongBotQ.target_tor[7]);
    this->FL_HP_JOINT->SetForce(0, Qbot.joint[7].torque.ref); //PongBotQ.target_tor[8]);
    this->FL_KN_JOINT->SetForce(0, Qbot.joint[8].torque.ref); //PongBotQ.target_tor[9]);

    this->FR_HR_JOINT->SetForce(0, Qbot.joint[9].torque.ref); //PongBotQ.target_tor[10]);
    this->FR_HP_JOINT->SetForce(0, Qbot.joint[10].torque.ref); //PongBotQ.target_tor[11]);
    this->FR_KN_JOINT->SetForce(0, Qbot.joint[11].torque.ref); //PongBotQ.target_tor[12]);

}

void gazebo::qrobot_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    printf("============= [Load] =============\n");
    this->model = _model;

    GetLinks();
    GetJoints();
    SensorSetting();
    RBDLSetting();
    InitROSCOMM();
    P_data = nh.advertise<std_msgs::Float64MultiArray>("ROS_DATA", 1);
    m_data.data.resize(10);
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&qrobot_plugin::UpdateAlgorithm, this));
}

void gazebo::qrobot_plugin::UpdateAlgorithm(void)
{  
	
	EncoderRead();
	IMUSensorRead();
	

	switch (Qbot.ControlMode){
		case CTRLMODE_NONE:
		
		break;

		case CTRLMODE_INIT_POSE:
			printf("CTRLMODE_INIT_POSE\n");
			Qbot.ResetTraj();
			Qbot.CommandFlag=GOTO_INIT_POSE;
			Qbot.ControlMode=CTRLMODE_NONE;
		break;

		case CTRLMODE_WALK_READY:
			printf("CTRLMODE_HOME_POSE\n");
			Qbot.ResetTraj();
			Qbot.CommandFlag=GOTO_WALK_READY;
			Qbot.ControlMode=CTRLMODE_NONE;
		break;
	}

	switch (Qbot.CommandFlag){
		case NO_ACT:
		break;

		case GOTO_INIT_POSE:
			Qbot.StateUpdate();
			Qbot.Init_Pose_Traj();
			Qbot.ComputeTorqueControl();
		break;

		case GOTO_WALK_READY:
			Qbot.StateUpdate();
			Qbot.WalkReady_Pose_Traj();
			Qbot.ComputeTorqueControl();
		break;
	}

	jointController();
	ROSMsgPub();
}

void gazebo::qrobot_plugin::Callback(const std_msgs::UInt16 &msg){
	if(msg.data==2){
		if(Qbot.Traj.move_done_flag==true){
			Qbot.ControlMode=CTRLMODE_WALK_READY;
		}
	}
}

void gazebo::qrobot_plugin::GUICallback(const std_msgs::UInt16 &msg){
	if(msg.data==0){
		// if(Qbot.Traj.move_done_flag==true){
		// 	Qbot.ControlMode=CTRLMODE_WALK_READY;
		// }
		printf("NONE\n");
	}else if(msg.data==1){
		if(Qbot.Traj.move_done_flag==true){
			Qbot.ControlMode=CTRLMODE_INIT_POSE;
		}
		printf("Init\n");
	}else if(msg.data==2){
		if(Qbot.Traj.move_done_flag==true){
			Qbot.ControlMode=CTRLMODE_WALK_READY;
		}
		printf("Ready\n");	
	}
}

void gazebo::qrobot_plugin::InitROSCOMM(){
	    server_sub = nh.subscribe("/Mode", 1, &gazebo::qrobot_plugin::Callback, this);
	    sub_gui = nh.subscribe("ROSGUI_PUB_MODE", 1, &gazebo::qrobot_plugin::GUICallback, this);
	    pub_gui=nh.advertise<std_msgs::UInt16>("ROSGUI_SUB_MODE",1000);
}

void gazebo::qrobot_plugin::ROSMsgPub(){
	std_msgs::UInt16 msg;

	if(Qbot.CommandFlag==GOTO_INIT_POSE){
		msg.data=1;
	}else if(Qbot.CommandFlag==GOTO_WALK_READY){
		msg.data=2;
	} 
	pub_gui.publish(msg);
}
// #include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <stdio.h>
#include <iostream>
#include <string>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "QRobot.h" 

#include <sensor_msgs/Joy.h>
//#include <linux/joystick.h>
#include "joystick.h"


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

		std_msgs::Float64MultiArray m_data;
		ros::Subscriber server_sub;
		ros::Subscriber sub_gui_mode;
		ros::Subscriber sub_gui_joy;
		ros::Subscriber sub_gui_pos_ref;
		ros::Subscriber sub_gui_gain;
		ros::Subscriber sub_gui_save;
		ros::Subscriber sub_joy;

		ros::Publisher pub_data;
		ros::Publisher pub_gui_mode;

		QRobot Qbot;
		Joystick joystick;
		// int js;
		// struct js_event event;
	//Save
 //    #define SAVE_LENGTH 12    //The number of data
 //    #define SAVE_TIME 60
 //    #define SAVE_COUNT SAVE_TIME*1000
	// unsigned int save_cnt = 0;
	// double save_array[SAVE_COUNT][SAVE_LENGTH];

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
		void GUICallback_Mode(const std_msgs::UInt16 &msg);
		void GUICallback_Joy(const std_msgs::Float32MultiArray::ConstPtr &msg);
		void GUICallback_POS_REF(const std_msgs::Float32MultiArray::ConstPtr &msg);
		void GUICallback_QP_GAIN(const std_msgs::Float32MultiArray::ConstPtr &msg);
		void GUICallback_Save_flag(const std_msgs::UInt16 &msg);
		void ROSMsgPub2GUI();
		void ROSMsgPub2RQT();
		void Joystick_INFO(const sensor_msgs::Joy::ConstPtr &msg); 
		
		// int read_event(int fd, struct js_event *event);
		// void joystick_reader( Joystick* joystick );
		


		void DataSave();
		void FileSave();
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
	Addons::URDFReadFromFile(Qbot.filepath.URDF.c_str(), qbot_model, true, true);
	Qbot.setRobotModel(qbot_model);
}

void gazebo::qrobot_plugin::EncoderRead()
{
    //************************** Encoder ********************************//

	Qbot.RL.joint.pos.now[0]= this->RL_HR_JOINT->Position(0);
	Qbot.RL.joint.pos.now[1]= this->RL_HP_JOINT->Position(0);
	Qbot.RL.joint.pos.now[2]= this->RL_KN_JOINT->Position(0);

	Qbot.RR.joint.pos.now[0]= this->RR_HR_JOINT->Position(0);
	Qbot.RR.joint.pos.now[1]= this->RR_HP_JOINT->Position(0);
	Qbot.RR.joint.pos.now[2]= this->RR_KN_JOINT->Position(0);

	Qbot.FL.joint.pos.now[0]= this->FL_HR_JOINT->Position(0);
	Qbot.FL.joint.pos.now[1]= this->FL_HP_JOINT->Position(0);
	Qbot.FL.joint.pos.now[2]= this->FL_KN_JOINT->Position(0);

	Qbot.FR.joint.pos.now[0]= this->FR_HR_JOINT->Position(0);
	Qbot.FR.joint.pos.now[1]= this->FR_HP_JOINT->Position(0);
	Qbot.FR.joint.pos.now[2]= this->FR_KN_JOINT->Position(0);

	Qbot.RL.joint.vel.now= (Qbot.RL.joint.pos.now - Qbot.RL.joint.pos.prev) / dt;
	Qbot.RR.joint.vel.now= (Qbot.RR.joint.pos.now - Qbot.RR.joint.pos.prev) / dt;
	Qbot.FL.joint.vel.now= (Qbot.FL.joint.pos.now - Qbot.FL.joint.pos.prev) / dt;
	Qbot.FR.joint.vel.now= (Qbot.FR.joint.pos.now - Qbot.FR.joint.pos.prev) / dt;

	Qbot.RL.joint.acc.now= (Qbot.RL.joint.vel.now - Qbot.RL.joint.vel.prev) / dt;
	Qbot.RR.joint.acc.now= (Qbot.RR.joint.vel.now - Qbot.RR.joint.vel.prev) / dt;
	Qbot.FL.joint.acc.now= (Qbot.FL.joint.vel.now - Qbot.FL.joint.vel.prev) / dt;
	Qbot.FR.joint.acc.now= (Qbot.FR.joint.vel.now - Qbot.FR.joint.vel.prev) / dt;
	
	Qbot.RL.joint.pos.prev=Qbot.RL.joint.pos.now;
	Qbot.RR.joint.pos.prev=Qbot.RR.joint.pos.now;
	Qbot.FL.joint.pos.prev=Qbot.FL.joint.pos.now;
	Qbot.FR.joint.pos.prev=Qbot.FR.joint.pos.now;

	Qbot.RL.joint.vel.prev=Qbot.RL.joint.vel.now;
	Qbot.RR.joint.vel.prev=Qbot.RR.joint.vel.now;
	Qbot.FL.joint.vel.prev=Qbot.FL.joint.vel.now;
	Qbot.FR.joint.vel.prev=Qbot.FR.joint.vel.now;

    // for (int i = 0; i < nDOF; i++) {
    // 	Qbot.joint[i].vel.now = (Qbot.joint[i].pos.now - Qbot.joint[i].pos.prev) / dt;
    // 	Qbot.joint[i].acc.now = (Qbot.joint[i].vel.now - Qbot.joint[i].vel.prev) / dt;

    // 	Qbot.joint[i].pos.prev=Qbot.joint[i].pos.now;
    // 	Qbot.joint[i].vel.prev=Qbot.joint[i].vel.now;
    // }
}

void gazebo::qrobot_plugin::IMUSensorRead()
{
	Qbot.CoM.ori.euler.vel.now(0) = this->IMU->AngularVelocity(false)[0];
	Qbot.CoM.ori.euler.vel.now(1) = this->IMU->AngularVelocity(false)[1];
	Qbot.CoM.ori.euler.vel.now(2) = this->IMU->AngularVelocity(false)[2];
	Qbot.CoM.ori.euler.pos.now = Qbot.CoM.ori.euler.pos.now + Qbot.CoM.ori.euler.vel.now*dt;

	Qbot.CoM.local.acc.now(0) =this->IMU->LinearAcceleration(false)[0];
	Qbot.CoM.local.acc.now(1) =this->IMU->LinearAcceleration(false)[1];
	Qbot.CoM.local.acc.now(2) =this->IMU->LinearAcceleration(false)[2];

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
	for (unsigned int i = 0; i < 3; ++i) {
		if (Qbot.RL.joint.torque.ref[i] >= 3000) {
			Qbot.RL.joint.torque.ref[i] = 3000;
		}
		else if (Qbot.RL.joint.torque.ref[i] <= -3000) {
			Qbot.RL.joint.torque.ref[i] = -3000;
		}

		if (Qbot.RR.joint.torque.ref[i] >= 3000) {
			Qbot.RR.joint.torque.ref[i] = 3000;
		}
		else if (Qbot.RR.joint.torque.ref[i] <= -3000) {
			Qbot.RR.joint.torque.ref[i] = -3000;
		}

		if (Qbot.FL.joint.torque.ref[i] >= 3000) {
			Qbot.FL.joint.torque.ref[i] = 3000;
		}
		else if (Qbot.FL.joint.torque.ref[i] <= -3000) {
			Qbot.FL.joint.torque.ref[i] = -3000;
		}

		if (Qbot.FR.joint.torque.ref[i] >= 3000) {
			Qbot.FR.joint.torque.ref[i] = 3000;
		}
		else if (Qbot.FR.joint.torque.ref[i] <= -3000) {
			Qbot.FR.joint.torque.ref[i] = -3000;
		}

	}

    //* Applying torques
    this->RL_HR_JOINT->SetForce(0, Qbot.RL.joint.torque.ref[0]); //PongBotQ.target_tor[0]);
    this->RL_HP_JOINT->SetForce(0, Qbot.RL.joint.torque.ref[1]); //PongBotQ.target_tor[1]);
    this->RL_KN_JOINT->SetForce(0, Qbot.RL.joint.torque.ref[2]); //PongBotQ.target_tor[2]);

    this->RR_HR_JOINT->SetForce(0, Qbot.RR.joint.torque.ref[0]); //PongBotQ.target_tor[3]);
    this->RR_HP_JOINT->SetForce(0, Qbot.RR.joint.torque.ref[1]); //PongBotQ.target_tor[4]);
    this->RR_KN_JOINT->SetForce(0, Qbot.RR.joint.torque.ref[2]); //PongBotQ.target_tor[5]);

    this->FL_HR_JOINT->SetForce(0, Qbot.FL.joint.torque.ref[0]); //PongBotQ.target_tor[7]);
    this->FL_HP_JOINT->SetForce(0, Qbot.FL.joint.torque.ref[1]); //PongBotQ.target_tor[8]);
    this->FL_KN_JOINT->SetForce(0, Qbot.FL.joint.torque.ref[2]); //PongBotQ.target_tor[9]);

    this->FR_HR_JOINT->SetForce(0, Qbot.FR.joint.torque.ref[0]); //PongBotQ.target_tor[10]);
    this->FR_HP_JOINT->SetForce(0, Qbot.FR.joint.torque.ref[1]); //PongBotQ.target_tor[11]);
    this->FR_KN_JOINT->SetForce(0, Qbot.FR.joint.torque.ref[2]); //PongBotQ.target_tor[12]);
}

void gazebo::qrobot_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{

    Joystick joystick("/dev/input/js0");
	// const char *device;
	// device="/dev/input/js0";
	// js=open(device, O_RDONLY | O_NONBLOCK);

	// if((joy_fd=open(JOY_DEV,O_RDONLY)) < 0)
	// {
	// 	cerr<<"Failed to open "<<JOY_DEV<<endl;
	// 	return -1;
	// }

	// ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
	// ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
	// ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

	// joy_button.resize(num_of_buttons,0);
	// joy_axis.resize(num_of_axis,0);

	// cout<<"Joystick: "<<name_of_joystick<<endl
	// <<"  axis: "<<num_of_axis<<endl
	// <<"  buttons: "<<num_of_buttons<<endl;

 //  	fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode
	
	printf("============= [Load] =============\n");
	this->model = _model;

	GetLinks();
	GetJoints();
	SensorSetting();
	RBDLSetting();
	InitROSCOMM();

	this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&qrobot_plugin::UpdateAlgorithm, this));
}

void gazebo::qrobot_plugin::UpdateAlgorithm(void)
{  
	
	JoystickEvent event;
    if (joystick.sample(&event))
    {
        // std::cout << "[isInitialState?" << event.isInitialState() << "] ";
        // std:: cout << " t=" << event.time;
        if (event.isButton())
        {
        	cout<<"A"<<endl;
            // std::cout << " Button#" << (int)event.number ;
            // std::cout << " value=" << ((event.value == 0) ? "up" : "down") ;
            // std::cout << std::endl;
        }
        else if (event.isAxis())
        {
            // std::cout << "Axis#" << (int)event.number;
            // std::cout << " value=" << (int)event.value;
            // std::cout << std::endl;
        }
    }

	EncoderRead();
	IMUSensorRead();
	
	Qbot.Robot_RUN();
	jointController();
	
	ROSMsgPub2GUI();
	ROSMsgPub2RQT();
}

void gazebo::qrobot_plugin::Callback(const std_msgs::UInt16 &msg){
	if(msg.data==2){
		if(Qbot.Traj.moveState.done==true){
			Qbot.ControlMode=CTRLMODE_WALK_READY;
		}
	}
}


void gazebo::qrobot_plugin::GUICallback_Mode(const std_msgs::UInt16 &msg){
	if(msg.data==0){
		// if(Qbot.Traj.move_done_flag==true){
		// 	Qbot.ControlMode=CTRLMODE_WALK_READY;
		// }
		printf("NONE\n");
	}else if(msg.data==1){
		if(Qbot.Traj.moveState.done==true){
			Qbot.ControlMode=CTRLMODE_INIT_POSE;
			printf("Init\n");
		}
		
	}else if(msg.data==2){
		if(Qbot.Traj.moveState.done==true){
			Qbot.ControlMode=CTRLMODE_WALK_READY;
			printf("Ready\n");
		}

	}else if(msg.data==3){
		if(Qbot.Traj.moveState.done==true){
			Qbot.ControlMode=CTRLMODE_SLOW_WALK;
			printf("Slow walk\n");
		}
	}
}


void gazebo::qrobot_plugin::GUICallback_Joy(const std_msgs::Float32MultiArray::ConstPtr &msg){
	// int i = 0;
	// float arr[3];
	// cout<<msg->data.size()<<endl;

	// for (int i=0;i<msg->data.size();++i){
	// 	const std_msgs::Float32MultiArray &MSG=msg->data[i];
	// 	cout<<MSG[0]<<endl;
	// }
	Qbot.joy.vel_x=msg->data[0];
	Qbot.joy.vel_y=msg->data[1];
	Qbot.joy.vel_yaw=msg->data[2];

	// cout<<"vel[x]:"<<msg->data[0]<<endl;
	// cout<<"vel[y]:"<<msg->data[1]<<endl;
	// cout<<"vel[yaw]:"<<msg->data[2]<<endl;
	// cout<<"-----"<<endl;
}


void gazebo::qrobot_plugin::GUICallback_POS_REF(const std_msgs::Float32MultiArray::ConstPtr &msg){
	// Qbot.CoM.ori.euler.pos.goal[0]=msg->data[0];
	// Qbot.CoM.ori.euler.pos.goal[1]=msg->data[1];
	// Qbot.CoM.ori.euler.pos.ref[0]=msg->data[0];
	// Qbot.CoM.ori.euler.pos.ref[1]=msg->data[1];
	// Qbot.CoM.pos.ref[0]=msg->data[2];
	// Qbot.CoM.pos.ref[1]=msg->data[3];
	// Qbot.CoM.pos.ref[2]=msg->data[4];
	
	// if(msg->data[6]==1){
	// 	Qbot.RL.global.pos.ref[2]=msg->data[5];	
	// }else if(msg->data[6]==2){
	// 	Qbot.RR.global.pos.ref[2]=msg->data[5];	
	// }else if(msg->data[6]==3){
	// 	Qbot.FL.global.pos.ref[2]=msg->data[5];	
	// }else if(msg->data[6]==4){
	// 	Qbot.FR.global.pos.ref[2]=msg->data[5];	
	// }

	// cout<<"leg:"<<msg->data[6]<<endl;
	// cout<<"---"<<endl;
}
void gazebo::qrobot_plugin::GUICallback_QP_GAIN(const std_msgs::Float32MultiArray::ConstPtr &msg){
	// controller.grf.qp.kp
	// Qbot.controller.grf.qp.kp(3)=msg->data[0];
	// Qbot.controller.grf.qp.kd(3)=msg->data[1];
	
	// Qbot.controller.grf.qp.kp(4)=msg->data[2];
	// Qbot.controller.grf.qp.kd(4)=msg->data[3];

	// Qbot.controller.grf.qp.kp(6)=msg->data[4];
	// Qbot.controller.grf.qp.kd(6)=msg->data[5];
	// Qbot.controller.grf.qp.kp(7)=msg->data[6];
	// Qbot.controller.grf.qp.kd(7)=msg->data[7];
	// Qbot.controller.grf.qp.kp(8)=msg->data[8];
	// Qbot.controller.grf.qp.kd(8)=msg->data[9];

	// Qbot.controller.grf.qp.kp(9)=msg->data[4];
	// Qbot.controller.grf.qp.kd(9)=msg->data[5];
	// Qbot.controller.grf.qp.kp(10)=msg->data[6];
	// Qbot.controller.grf.qp.kd(10)=msg->data[7];
	// Qbot.controller.grf.qp.kp(11)=msg->data[8];
	// Qbot.controller.grf.qp.kd(11)=msg->data[9];

	// Qbot.controller.grf.qp.kp(12)=msg->data[4];
	// Qbot.controller.grf.qp.kd(12)=msg->data[5];
	// Qbot.controller.grf.qp.kp(13)=msg->data[6];
	// Qbot.controller.grf.qp.kd(13)=msg->data[7];
	// Qbot.controller.grf.qp.kp(14)=msg->data[8];
	// Qbot.controller.grf.qp.kd(14)=msg->data[9];

	// Qbot.controller.grf.qp.kp(15)=msg->data[4];
	// Qbot.controller.grf.qp.kd(15)=msg->data[5];
	// Qbot.controller.grf.qp.kp(16)=msg->data[6];
	// Qbot.controller.grf.qp.kd(16)=msg->data[7];
	// Qbot.controller.grf.qp.kp(17)=msg->data[8];
	// Qbot.controller.grf.qp.kd(17)=msg->data[9];
	// cout<<msg->data[0]<<endl;
	// cout<<msg->data[1]<<endl;
	// cout<<msg->data[2]<<endl;
	// cout<<msg->data[3]<<endl;
	// cout<<msg->data[4]<<endl;
	// cout<<msg->data[5]<<endl;
	// cout<<msg->data[6]<<endl;
	// cout<<msg->data[7]<<endl;
	// cout<<msg->data[8]<<endl;
	// cout<<msg->data[9]<<endl;
	// cout<<"-----"<<endl;
}

void gazebo::qrobot_plugin::GUICallback_Save_flag(const std_msgs::UInt16 &msg){
	if(msg.data==1){

		FileSave();
		
		cout<<"File Save Complete!"<<endl;
	}
}

void gazebo::qrobot_plugin::InitROSCOMM(){
	server_sub = nh.subscribe("/Mode", 1, &gazebo::qrobot_plugin::Callback, this);
	sub_gui_mode = nh.subscribe("ROSGUI_PUB_MODE", 1, &gazebo::qrobot_plugin::GUICallback_Mode, this);
	sub_gui_joy = nh.subscribe("ROSGUI_PUB_JOY", 1, &gazebo::qrobot_plugin::GUICallback_Joy, this);
	sub_gui_pos_ref = nh.subscribe("ROSGUI_PUB_POS_REF", 1, &gazebo::qrobot_plugin::GUICallback_POS_REF, this);
	sub_gui_gain = nh.subscribe("ROSGUI_PUB_QP_GAIN", 1, &gazebo::qrobot_plugin::GUICallback_QP_GAIN, this);
	sub_gui_save = nh.subscribe("ROSGUI_PUB_SAVE", 1, &gazebo::qrobot_plugin::GUICallback_Save_flag, this);
	sub_joy = nh.subscribe("/joy", 1, &gazebo::qrobot_plugin::Joystick_INFO, this);

	pub_gui_mode=nh.advertise<std_msgs::UInt16>("ROSGUI_SUB_MODE",1000);
	pub_data = nh.advertise<std_msgs::Float64MultiArray>("/tmp_data/", 1);
	m_data.data.resize(15);
}

void gazebo::qrobot_plugin::ROSMsgPub2GUI(){
	std_msgs::UInt16 msg;

	if(Qbot.CommandFlag==GOTO_INIT_POSE){
		msg.data=1;
	}else if(Qbot.CommandFlag==GOTO_READY_POSE){
		msg.data=2;
	}else if(Qbot.CommandFlag==GOTO_SLOW_WALK){
		msg.data=3;
	}

	pub_gui_mode.publish(msg);
}

void gazebo::qrobot_plugin::ROSMsgPub2RQT(){
	m_data.data[0]=Qbot.CoM.pos.now[0];
	m_data.data[1]=Qbot.CoM.pos.now[1];
	m_data.data[2]=Qbot.CoM.pos.now[2];
	
	m_data.data[3]=Qbot.Base.pos.now[0];
	m_data.data[4]=Qbot.Base.pos.now[1];
	m_data.data[5]=Qbot.Base.pos.now[2];

	m_data.data[6]=Qbot.CoM.vel.now[0];
	m_data.data[7]=Qbot.CoM.vel.now[1];
	m_data.data[8]=Qbot.CoM.vel.now[2];
	
	// m_data.data[2]=Qbot.RL.joint.vel.ref[1];
	// m_data.data[3]=Qbot.RL.joint.vel.ref[2];
	// m_data.data[4]=Qbot.RL.joint.vel.now[1];
	// m_data.data[5]=Qbot.RL.joint.vel.now[2];

	// m_data.data[4]=Qbot.RL.global.vel.ref[2];
	// m_data.data[5]=Qbot.RR.global.vel.ref[2];
	// m_data.data[6]=Qbot.FL.global.vel.ref[2];
	// m_data.data[7]=Qbot.FR.global.vel.ref[2];

	// m_data.data[8]=Qbot.RL.global.acc.ref[2];
	// m_data.data[9]=Qbot.RR.global.acc.ref[2];
	// m_data.data[10]=Qbot.FL.global.acc.ref[2];
	// m_data.data[11]=Qbot.FR.global.acc.ref[2];
	
	// m_data.data[8]=Qbot.Traj.walk.zmp.X_new(0,0);
	// m_data.data[9]=Qbot.Traj.walk.zmp.X_new(0,1);

	pub_data.publish(m_data);
}


void gazebo::qrobot_plugin::FileSave(void){
	FILE *fp;
	fp = fopen(Qbot.filepath.FILE_SAVE.c_str(), "w");

	for (int j = 0; j <= Qbot.datasave.SAVE_COUNT - 1; j++) {
		for (int i = 0; i <= Qbot.datasave.SAVE_LENGTH - 1; i++) {
			fprintf(fp, "%f\t", Qbot.datasave.save_array[j][i]);
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
}

void gazebo::qrobot_plugin::Joystick_INFO(const sensor_msgs::Joy::ConstPtr &msg) { //Joystick Wire mode
    const double max_x_vel = 0.5; //1.4;
    const double max_y_vel = 0.5;
    const double max_yaw_ori = 5 * D2R; // rad

    // if (msg->buttons[8] == true) {
    //     // ========= [Walk Ready] ========== //
    //     if (Qbot.moving_done_flag == true) {
    //         Qbot.ControlMode = 3;
    //     }
    // } else if (msg->buttons[3] == true) {
    //     // ========= [Trot Walk] ========== //
    //     if (Qbot.moving_done_flag == true) {
    //         Qbot.ControlMode = 4;
    //         Qbot.ControlMode = 7;
    //         Qbot.move_stop_flag = 0;
    //     }
    // } else if (msg->buttons[4] == true) {
    //     if (Qbot.moving_done_flag == true) {
    //         Qbot.ControlMode = 5;
    //         Qbot.move_stop_flag = 0;
    //         Qbot.move_stop_flag = 0;
    //     }
    // } else if (msg->buttons[5] == true) {
    //     if (Qbot.moving_done_flag == true) {
    //         //            cout << "2" << endl;
    //     }
    // } else if (msg->buttons[10] == true) {
    //     if (Qbot.moving_done_flag == true) {
    //         Qbot.ControlMode = CTRLMODE_SLOW_WALK_HS;
    //         Qbot.move_stop_flag = false;
    //     }
    // } else {
    //     PongBotQ.ControlMode = 0;
    // }

    // if (msg->buttons[1] == true) {
    //     Qbot.moving_done_flag = true;
    //     Qbot.move_stop_flag = 1;
    //     if (Qbot.pre_sub_ctrl_flag == true) {
    //         Qbot.move_stop_flag = false;
    //     }
    // }

    // if (msg->buttons[9] == true) {
    //     Qbot.CommandFlag = TORQUE_OFF;
    // }

    // cout<<"b0:"<<msg->buttons[0]<<endl;
    // cout<<"b1:"<<msg->buttons[1]<<endl;	
    // cout<<"b2:"<<msg->buttons[2]<<endl;
    // cout<<"b3:"<<msg->buttons[3]<<endl;
    // cout<<"b4:"<<msg->buttons[4]<<endl;
    // cout<<"b5:"<<msg->buttons[5]<<endl;
    // cout<<"b6:"<<msg->buttons[6]<<endl;

    // cout<<"A0:"<<msg->axes[0]<<endl;
    // cout<<"A1:"<<msg->axes[1]<<endl;
    // cout<<"A2:"<<msg->axes[2]<<endl;
    // cout<<"A3:"<<msg->axes[3]<<endl;
    // cout<<"--------------"<<endl;


    // Qbot.speed_x = (msg->axes[1]) * 0.05;
    // Qbot.speed_y = (msg->axes[0]) * 0.04;
    // Qbot.speed_yaw = (msg->axes[3]) * 4.0 * (PI / 180);
}

// int gazebo::qrobot_plugin::read_event(int fd, struct js_event *event)
// {
// 	ssize_t bytes;

// 	bytes = read(fd, event, sizeof(*event));

// 	if (bytes == sizeof(*event))
// 		return 0;

//     /* Error, could not read full event. */
// 	return -1;
// }

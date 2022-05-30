#ifndef QROBOT_H
#define QROBOT_H

#include <stdio.h>
#include <math.h>
#include "Eigen/Dense"
#include <iostream>
#include <unistd.h>
#include <string>
//#include <boost/filesystem.hpp>
//#include <filesystem>

#include "robot_state.h"
#include "trajectory.h"
#include "controller.h"

struct FORCE{
	double now;
	double ref;
};

class QRobot : public RobotState, public Trajectory, public Controller
{
public:
	QRobot();
	~QRobot();
	void Robot_RUN();
	
private:
};
#endif 


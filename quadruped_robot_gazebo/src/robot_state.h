#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H


#include "common.h"

class QRobot;
class RobotState : virtual public Common{
public:
    RobotState();
    ~RobotState();
public:

    void setParam();
    void setRobotModel(Model* getModel);
    void StateUpdate(void);
    
private:
};

#endif

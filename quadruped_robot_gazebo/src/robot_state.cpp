#include "robot_state.h"

RobotState::RobotState() {
   std::cout<<"Class 'RobotState' Called"<<std::endl;
   setParam();
}

RobotState::~RobotState() {

}

void RobotState::setParam() {
    RL.joint.pos.now<<0,0,0;
    RR.joint.pos.now<<0,0,0;
    FL.joint.pos.now<<0,0,0;
    FR.joint.pos.now<<0,0,0;
    
    RL.joint.pos.prev<<0,0,0;
    RR.joint.pos.prev<<0,0,0;
    FL.joint.pos.prev<<0,0,0;
    FR.joint.pos.prev<<0,0,0;

    RL.joint.vel.now<<0,0,0;
    RR.joint.vel.now<<0,0,0;
    FL.joint.vel.now<<0,0,0;
    FR.joint.vel.now<<0,0,0;

    RL.joint.vel.prev<<0,0,0;
    RR.joint.vel.prev<<0,0,0;
    FL.joint.vel.prev<<0,0,0;
    FR.joint.vel.prev<<0,0,0;

}

void RobotState::setRobotModel(Model* getModel){
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

void RobotState::StateUpdate(void){
    double measured_height;
    // MatrixNd tmp_J_BASE = MatrixNd::Zero(6, 19);
    MatrixNd tmp_J_RL = MatrixNd::Zero(3, 18);
    MatrixNd tmp_J_RR = MatrixNd::Zero(3, 18);
    MatrixNd tmp_J_FL = MatrixNd::Zero(3, 18);
    MatrixNd tmp_J_FR = MatrixNd::Zero(3, 18);

    rbdl.BaseState<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    rbdl.BaseStateDot<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    rbdl.BaseState2Dot<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    
    rbdl.JointState<<RL.joint.pos.now,RR.joint.pos.now,FL.joint.pos.now,FR.joint.pos.now;
    rbdl.JointStateDot<<RL.joint.vel.now,RR.joint.vel.now,FL.joint.vel.now,FR.joint.vel.now;
    rbdl.JointState2Dot<<RL.joint.acc.now,RR.joint.acc.now,FL.joint.vel.now,FR.joint.acc.now;
    
    rbdl.RobotState<<rbdl.BaseState,rbdl.JointState,0;
    rbdl.RobotStateDot<<rbdl.BaseStateDot,rbdl.JointStateDot;
    rbdl.RobotState2Dot<<rbdl.BaseState2Dot,rbdl.JointState2Dot;

    CoM.ori.quat.pos.ref<<0.,0.,0.,1.;
    rbdl.pModel->SetQuaternion(Base.ID,CoM.ori.quat.pos.ref,rbdl.RobotState);
    
    // CalcPointJacobian6D(*rbdl.pModel, rbdl.RobotState, Base.ID, rbdl.BaseState.segment(0,3), tmp_J_BASE, true); 
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

    RL.ep.local.pos.now=rbdl.EPState.segment(0,3);
    RR.ep.local.pos.now=rbdl.EPState.segment(3,3);
    FL.ep.local.pos.now=rbdl.EPState.segment(6,3);
    FR.ep.local.pos.now=rbdl.EPState.segment(9,3);

    RL.ep.local.vel.now=rbdl.EPStateDot.segment(0,3);
    RR.ep.local.vel.now=rbdl.EPStateDot.segment(3,3);
    FL.ep.local.vel.now=rbdl.EPStateDot.segment(6,3);
    FR.ep.local.vel.now=rbdl.EPStateDot.segment(9,3);   

    if(CommandFlag==GOTO_INIT_POSE){
        Base.pos.now[0]=0.0;
        Base.pos.now[1]=0.0;
        Base.pos.now[2]=-(RL.ep.local.pos.now[2]+RR.ep.local.pos.now[2]+FL.ep.local.pos.now[2]+FR.ep.local.pos.now[2])/4.0;
        CoM.pos.now=GetComPos(Base.pos.now,CoM.ori.R.now);
        
        Base.pos.ref.segment(0,2)=Base.pos.now.segment(0,2);
        CoM.pos.prev=CoM.pos.now;
    }else{
        Base.pos.now.segment(0,2)=Base.pos.ref.segment(0,2);
        measured_height = -(RL.contact.ref * RL.ep.local.pos.now(2) + RR.contact.ref * RR.ep.local.pos.now(2) + FL.contact.ref * FL.ep.local.pos.now(2) + FR.contact.ref * FR.ep.local.pos.now(2)) / (RL.contact.ref + RR.contact.ref + FL.contact.ref + FR.contact.ref) / cos(abs(CoM.ori.euler.pos.now(1)));
        KalmanFilter(measured_height);
        Base.pos.now(2)=kalman.x_hat(0);
        // Base.vel.now(2)=kalman.x_hat(1);
    }

    CoM.pos.now=GetComPos(Base.pos.now,CoM.ori.R.now);  
    CoM.vel.now=(CoM.pos.now-CoM.pos.prev)/dt;
    CoM.pos.prev=CoM.pos.now;
    // cout<<"Base:"<<Base.pos.now.transpose()<<endl;
    // cout<<"Com:"<<CoM.pos.now.transpose()<<endl;
    // cout<<"--"<<endl;
}



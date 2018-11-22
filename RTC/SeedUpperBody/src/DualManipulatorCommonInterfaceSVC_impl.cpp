// -*-C++-*-
/*!
 * @file  DualManipulatorCommonInterfaceSVC_impl.cpp
 * @brief Service implementation code of DualManipulatorCommonInterface.idl
 *
 */

#include "DualManipulatorCommonInterfaceSVC_impl.h"
#include "LibSeednoidUpperUnit1.h"
#include "dual_ReturnID.h"

/*
 * Example implementational code for IDL interface DualManipulatorCommonInterface::CommonCommands
 */
DualManipulatorCommonInterface_CommonCommandsSVC_impl::DualManipulatorCommonInterface_CommonCommandsSVC_impl()
{
  // Please add extra constructor code here.
}


DualManipulatorCommonInterface_CommonCommandsSVC_impl::~DualManipulatorCommonInterface_CommonCommandsSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_CommonCommandsSVC_impl::servoOFF()
{
  
  int torque = 0;
  std::cout << "ServoOFF" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_CommonCommandsSVC_impl::servoON()
{
  
  int torque = 1;
  std::cout << "ServoON" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
  
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_CommonCommandsSVC_impl::servoOFFArm()
{
  
  int torque = 0;
  std::cout << "ServoOFFArm" << std::endl;
  
  noid.RightArmServoOnOff(torque);
  noid.LeftArmServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
  
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_CommonCommandsSVC_impl::servoOFFHand()
{
  
  int torque = 0;
  std::cout << "ServoOFFHand" << std::endl;
  
  noid.RightGripperServoOnOff(torque);
  noid.LeftGripperServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_CommonCommandsSVC_impl::servoONArm()
{
  
  int torque = 1;
  std::cout << "ServoONArm" << std::endl;
  
  noid.RightArmServoOnOff(torque);
  noid.LeftArmServoOnOff(torque);
  
  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_CommonCommandsSVC_impl::servoONHand()
{
  
  int torque = 1;
  std::cout << "ServoONHand" << std::endl;
  
  noid.RightGripperServoOnOff(torque);
  noid.LeftGripperServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface DualManipulatorCommonInterface::MotionCommands
 */
DualManipulatorCommonInterface_MotionCommandsSVC_impl::DualManipulatorCommonInterface_MotionCommandsSVC_impl()
{
  m_speedRatioJoint = 50;
  m_maxSpeedJoint.length(14);
  for(unsigned int i=0; i<14; i++){
    m_maxSpeedJoint[i] = 60;//deg/s
  }
}

DualManipulatorCommonInterface_MotionCommandsSVC_impl::~DualManipulatorCommonInterface_MotionCommandsSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::closeGripper()
{
  
  std::cout << "closeGripper" << std::endl;
 
  noid.setRightHandCurrent(100);
  noid.setLeftHandCurrent(100);
  usleep(1000*10);
  noid.CloseRightGripper();
  noid.CloseLeftGripper();
  //sleep(5);
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
  
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::moveGripper(const DualManipulatorCommonInterface::DoubleSeq& r_angle, const DualManipulatorCommonInterface::DoubleSeq& l_angle)
{

  std::cout << "moveGripper" << std::endl;

  double r_move;
  double l_move;
  
  r_move = (double)r_angle[0];
  l_move = (double)l_angle[0];
  
  
  if (r_angle[0]>0 && r_angle[0] <= 100){
    noid.MoveRightGripper(r_move);
  }
  if (l_angle[0]>0 && l_angle[0] <= 100){
    noid.MoveLeftGripper(l_move);
  }
  else{
    std::cout << "ERROR : angleRatio Wrong Value" << std::endl;
  }
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::moveLinearCartesianAbs(const DualManipulatorCommonInterface::CarPosWithElbow& rArm, const DualManipulatorCommonInterface::CarPosWithElbow& lArm)
{
  double now_r_eerot[9];
  double now_l_eerot[9];
  double now_r_eetrans[3];
  double now_l_eetrans[3];

  double via_r_eerot[9];
  double via_l_eerot[9];
  double via_r_eetrans[3];
  double via_l_eetrans[3];
  
  double goal_r_eerot[9];
  double goal_l_eerot[9];
  double goal_r_eetrans[3];
  double goal_l_eetrans[3];
  
  double nowRightpos[7];
  double nowLeftpos[7];

  double r_elbow;
  double l_elbow;

  int r_vianum;
  int l_vianum;
  
  int r_cartesianLength[3];
  int l_cartesianLength[3];
  int r_maxCartesianLength;
  int l_maxCartesianLength;

  DualManipulatorCommonInterface::CarPosWithElbow RARM;
  DualManipulatorCommonInterface::CarPosWithElbow LARM;
 
  goal_r_eerot[0]=rArm.carPos[0][0];
  goal_r_eerot[1]=rArm.carPos[0][1];
  goal_r_eerot[2]=rArm.carPos[0][2];  
  goal_r_eerot[3]=rArm.carPos[1][0];
  goal_r_eerot[4]=rArm.carPos[1][1];
  goal_r_eerot[5]=rArm.carPos[1][2]; 
  goal_r_eerot[6]=rArm.carPos[2][0];
  goal_r_eerot[7]=rArm.carPos[2][1];
  goal_r_eerot[8]=rArm.carPos[2][2];
  
  goal_r_eetrans[0]=rArm.carPos[0][3];
  goal_r_eetrans[1]=rArm.carPos[1][3];
  goal_r_eetrans[2]=rArm.carPos[2][3];

  goal_l_eerot[0]=lArm.carPos[0][0];
  goal_l_eerot[1]=lArm.carPos[0][1];
  goal_l_eerot[2]=lArm.carPos[0][2];  
  goal_l_eerot[3]=lArm.carPos[1][0];
  goal_l_eerot[4]=lArm.carPos[1][1];
  goal_l_eerot[5]=lArm.carPos[1][2]; 
  goal_l_eerot[6]=lArm.carPos[2][0];
  goal_l_eerot[7]=lArm.carPos[2][1];
  goal_l_eerot[8]=lArm.carPos[2][2];
  
  goal_l_eetrans[0]=lArm.carPos[0][3];
  goal_l_eetrans[1]=lArm.carPos[1][3];
  goal_l_eetrans[2]=lArm.carPos[2][3];

  r_elbow = rArm.elbow;
  l_elbow = lArm.elbow;
  
  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);

  noid.Solve_RightArmFk(now_r_eerot,now_r_eetrans,nowRightpos);
  noid.Solve_LeftArmFk(now_l_eerot,now_l_eetrans,nowLeftpos);

  std::cout << "now pos" << std::endl;
  std::cout << now_r_eetrans[0] << std::endl;
  std::cout << now_r_eetrans[1] << std::endl;
  std::cout << now_r_eetrans[2] << std::endl;

  for(int eetnum=0;eetnum<3;eetnum++)
    {
      r_cartesianLength[eetnum] = fabs(now_r_eetrans[eetnum] - goal_r_eetrans[eetnum]);
      l_cartesianLength[eetnum] = fabs(now_l_eetrans[eetnum] - goal_l_eetrans[eetnum]);
    }
  
  r_maxCartesianLength = r_cartesianLength[0];
  if(r_maxCartesianLength < r_cartesianLength[1]){
    r_maxCartesianLength = r_cartesianLength[1];
  }
  if(r_maxCartesianLength < r_cartesianLength[2]){
    r_maxCartesianLength = r_cartesianLength[2];
  }
  l_maxCartesianLength = l_cartesianLength[0];
  if(l_maxCartesianLength < l_cartesianLength[1]){
    l_maxCartesianLength = l_cartesianLength[1];
  }
  if(l_maxCartesianLength < l_cartesianLength[2]){
    l_maxCartesianLength = l_cartesianLength[2];
  }

  r_vianum = (int)r_maxCartesianLength/5;
  l_vianum = (int)l_maxCartesianLength/5;

  std::cout << "r_vianum" << r_vianum << std::endl;
  std::cout << "l_vianum" << l_vianum << std::endl;

  RARM.elbow = r_elbow;
  LARM.elbow = l_elbow;

  for(int r_viacnt=0; r_viacnt < r_vianum; r_viacnt++)
    {
      std::cout << "r_viacnt" << r_viacnt << std::endl;
      
      RARM.carPos[0][3] = now_r_eetrans[0] + (goal_r_eetrans[0] - now_r_eetrans[0])/r_vianum*r_viacnt;
      RARM.carPos[1][3] = now_r_eetrans[1] + (goal_r_eetrans[1] - now_r_eetrans[1])/r_vianum*r_viacnt;
      RARM.carPos[2][3] = now_r_eetrans[2] + (goal_r_eetrans[2] - now_r_eetrans[2])/r_vianum*r_viacnt;

      RARM.carPos[0][0] = now_r_eerot[0];
      RARM.carPos[0][1] = now_r_eerot[1];
      RARM.carPos[0][2] = now_r_eerot[2];
      RARM.carPos[1][0] = now_r_eerot[3];
      RARM.carPos[1][1] = now_r_eerot[4];
      RARM.carPos[1][2] = now_r_eerot[5]; 
      RARM.carPos[2][0] = now_r_eerot[6];
      RARM.carPos[2][1] = now_r_eerot[7];
      RARM.carPos[2][2] = now_r_eerot[8];

      RightArmPTPCartesianAbs(RARM);
    }

  RARM.carPos[0][3] = goal_r_eetrans[0];
  RARM.carPos[1][3] = goal_r_eetrans[1];
  RARM.carPos[2][3] = goal_r_eetrans[2];

  RARM.carPos[0][0] = goal_r_eerot[0];
  RARM.carPos[0][1] = goal_r_eerot[1];
  RARM.carPos[0][2] = goal_r_eerot[2];
  RARM.carPos[1][0] = goal_r_eerot[3];
  RARM.carPos[1][1] = goal_r_eerot[4];
  RARM.carPos[1][2] = goal_r_eerot[5]; 
  RARM.carPos[2][0] = goal_r_eerot[6];
  RARM.carPos[2][1] = goal_r_eerot[7];
  RARM.carPos[2][2] = goal_r_eerot[8];

  RightArmPTPCartesianAbs(RARM);

  
  for(int l_viacnt=0; l_viacnt < l_vianum; l_viacnt++)
    {
      std::cout << "l_viacnt" << l_viacnt << std::endl;
      
      LARM.carPos[0][3] = now_l_eetrans[0] + (goal_l_eetrans[0] - now_l_eetrans[0])/l_vianum*l_viacnt;
      LARM.carPos[1][3] = now_l_eetrans[1] + (goal_l_eetrans[1] - now_l_eetrans[1])/l_vianum*l_viacnt;
      LARM.carPos[2][3] = now_l_eetrans[2] + (goal_l_eetrans[2] - now_l_eetrans[2])/l_vianum*l_viacnt;

      LARM.carPos[0][0] = now_l_eerot[0];
      LARM.carPos[0][1] = now_l_eerot[1];
      LARM.carPos[0][2] = now_l_eerot[2];
      LARM.carPos[1][0] = now_l_eerot[3];
      LARM.carPos[1][1] = now_l_eerot[4];
      LARM.carPos[1][2] = now_l_eerot[5]; 
      LARM.carPos[2][0] = now_l_eerot[6];
      LARM.carPos[2][1] = now_l_eerot[7];
      LARM.carPos[2][2] = now_l_eerot[8];

      LeftArmPTPCartesianAbs(LARM);
    }

  LARM.carPos[0][3] = goal_l_eetrans[0];
  LARM.carPos[1][3] = goal_l_eetrans[1];
  LARM.carPos[2][3] = goal_l_eetrans[2];

  LARM.carPos[0][0] = goal_l_eerot[0];
  LARM.carPos[0][1] = goal_l_eerot[1];
  LARM.carPos[0][2] = goal_l_eerot[2];
  LARM.carPos[1][0] = goal_l_eerot[3];
  LARM.carPos[1][1] = goal_l_eerot[4];
  LARM.carPos[1][2] = goal_l_eerot[5]; 
  LARM.carPos[2][0] = goal_l_eerot[6];
  LARM.carPos[2][1] = goal_l_eerot[7];
  LARM.carPos[2][2] = goal_l_eerot[8];

  LeftArmPTPCartesianAbs(LARM);

  
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::moveLinearCartesianRel(const DualManipulatorCommonInterface::CarPosWithElbow& rArm, const DualManipulatorCommonInterface::CarPosWithElbow& lArm)
{
  double now_r_eerot[9];
  double now_l_eerot[9];
  double now_r_eetrans[3];
  double now_l_eetrans[3];

  double via_r_eerot[9];
  double via_l_eerot[9];
  double via_r_eetrans[3];
  double via_l_eetrans[3];
  
  double goal_r_eerot[9];
  double goal_l_eerot[9];
  double goal_r_eetrans[3];
  double goal_l_eetrans[3];
  
  double nowRightpos[7];
  double nowLeftpos[7];

  double r_elbow;
  double l_elbow;

  int r_vianum;
  int l_vianum;
  
  int r_cartesianLength[3];
  int l_cartesianLength[3];
  int r_maxCartesianLength;
  int l_maxCartesianLength;

  DualManipulatorCommonInterface::CarPosWithElbow RARM;
  DualManipulatorCommonInterface::CarPosWithElbow LARM;
 
  goal_r_eerot[0]=rArm.carPos[0][0];
  goal_r_eerot[1]=rArm.carPos[0][1];
  goal_r_eerot[2]=rArm.carPos[0][2];  
  goal_r_eerot[3]=rArm.carPos[1][0];
  goal_r_eerot[4]=rArm.carPos[1][1];
  goal_r_eerot[5]=rArm.carPos[1][2]; 
  goal_r_eerot[6]=rArm.carPos[2][0];
  goal_r_eerot[7]=rArm.carPos[2][1];
  goal_r_eerot[8]=rArm.carPos[2][2];
  
  goal_r_eetrans[0]=rArm.carPos[0][3];
  goal_r_eetrans[1]=rArm.carPos[1][3];
  goal_r_eetrans[2]=rArm.carPos[2][3];

  goal_l_eerot[0]=lArm.carPos[0][0];
  goal_l_eerot[1]=lArm.carPos[0][1];
  goal_l_eerot[2]=lArm.carPos[0][2];  
  goal_l_eerot[3]=lArm.carPos[1][0];
  goal_l_eerot[4]=lArm.carPos[1][1];
  goal_l_eerot[5]=lArm.carPos[1][2]; 
  goal_l_eerot[6]=lArm.carPos[2][0];
  goal_l_eerot[7]=lArm.carPos[2][1];
  goal_l_eerot[8]=lArm.carPos[2][2];
  
  goal_l_eetrans[0]=lArm.carPos[0][3];
  goal_l_eetrans[1]=lArm.carPos[1][3];
  goal_l_eetrans[2]=lArm.carPos[2][3];

  r_elbow = rArm.elbow;
  l_elbow = lArm.elbow;
  
  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);

  noid.Solve_RightArmFk(now_r_eerot,now_r_eetrans,nowRightpos);
  noid.Solve_LeftArmFk(now_l_eerot,now_l_eetrans,nowLeftpos);

  for (int eetnum=0;eetnum<3;eetnum++)
    {
      goal_r_eetrans[eetnum] = goal_r_eetrans[eetnum] + now_r_eetrans[eetnum];
      goal_l_eetrans[eetnum] = goal_l_eetrans[eetnum] + now_l_eetrans[eetnum];
    }

  for(int eetnum=0;eetnum<3;eetnum++)
    {
      r_cartesianLength[eetnum] = fabs(now_r_eetrans[eetnum] - goal_r_eetrans[eetnum]);
      l_cartesianLength[eetnum] = fabs(now_l_eetrans[eetnum] - goal_l_eetrans[eetnum]);
    }
  
  r_maxCartesianLength = r_cartesianLength[0];
  if(r_maxCartesianLength < r_cartesianLength[1]){
    r_maxCartesianLength = r_cartesianLength[1];
  }
  if(r_maxCartesianLength < r_cartesianLength[2]){
    r_maxCartesianLength = r_cartesianLength[2];
  }
  l_maxCartesianLength = l_cartesianLength[0];
  if(l_maxCartesianLength < l_cartesianLength[1]){
    l_maxCartesianLength = l_cartesianLength[1];
  }
  if(l_maxCartesianLength < l_cartesianLength[2]){
    l_maxCartesianLength = l_cartesianLength[2];
  }

  r_vianum = (int)r_maxCartesianLength/5;
  l_vianum = (int)l_maxCartesianLength/5;

  std::cout << "r_vianum" << r_vianum << std::endl;
  std::cout << "l_vianum" << l_vianum << std::endl;

  RARM.elbow = r_elbow;
  LARM.elbow = l_elbow;

  for(int r_viacnt=0; r_viacnt < r_vianum; r_viacnt++)
    {
      std::cout << "r_viacnt" << r_viacnt << std::endl;
      
      RARM.carPos[0][3] = now_r_eetrans[0] + (goal_r_eetrans[0] - now_r_eetrans[0])/r_vianum*r_viacnt;
      RARM.carPos[1][3] = now_r_eetrans[1] + (goal_r_eetrans[1] - now_r_eetrans[1])/r_vianum*r_viacnt;
      RARM.carPos[2][3] = now_r_eetrans[2] + (goal_r_eetrans[2] - now_r_eetrans[2])/r_vianum*r_viacnt;

      RARM.carPos[0][0] = now_r_eerot[0];
      RARM.carPos[0][1] = now_r_eerot[1];
      RARM.carPos[0][2] = now_r_eerot[2];
      RARM.carPos[1][0] = now_r_eerot[3];
      RARM.carPos[1][1] = now_r_eerot[4];
      RARM.carPos[1][2] = now_r_eerot[5]; 
      RARM.carPos[2][0] = now_r_eerot[6];
      RARM.carPos[2][1] = now_r_eerot[7];
      RARM.carPos[2][2] = now_r_eerot[8];

      RightArmPTPCartesianAbs(RARM);
    }

  RARM.carPos[0][3] = goal_r_eetrans[0];
  RARM.carPos[1][3] = goal_r_eetrans[1];
  RARM.carPos[2][3] = goal_r_eetrans[2];

  RARM.carPos[0][0] = goal_r_eerot[0];
  RARM.carPos[0][1] = goal_r_eerot[1];
  RARM.carPos[0][2] = goal_r_eerot[2];
  RARM.carPos[1][0] = goal_r_eerot[3];
  RARM.carPos[1][1] = goal_r_eerot[4];
  RARM.carPos[1][2] = goal_r_eerot[5]; 
  RARM.carPos[2][0] = goal_r_eerot[6];
  RARM.carPos[2][1] = goal_r_eerot[7];
  RARM.carPos[2][2] = goal_r_eerot[8];

  RightArmPTPCartesianAbs(RARM);

  
  for(int l_viacnt=0; l_viacnt < l_vianum; l_viacnt++)
    {
      std::cout << "l_viacnt" << l_viacnt << std::endl;
      
      LARM.carPos[0][3] = now_l_eetrans[0] + (goal_l_eetrans[0] - now_l_eetrans[0])/l_vianum*l_viacnt;
      LARM.carPos[1][3] = now_l_eetrans[1] + (goal_l_eetrans[1] - now_l_eetrans[1])/l_vianum*l_viacnt;
      LARM.carPos[2][3] = now_l_eetrans[2] + (goal_l_eetrans[2] - now_l_eetrans[2])/l_vianum*l_viacnt;

      LARM.carPos[0][0] = now_l_eerot[0];
      LARM.carPos[0][1] = now_l_eerot[1];
      LARM.carPos[0][2] = now_l_eerot[2];
      LARM.carPos[1][0] = now_l_eerot[3];
      LARM.carPos[1][1] = now_l_eerot[4];
      LARM.carPos[1][2] = now_l_eerot[5]; 
      LARM.carPos[2][0] = now_l_eerot[6];
      LARM.carPos[2][1] = now_l_eerot[7];
      LARM.carPos[2][2] = now_l_eerot[8];

      LeftArmPTPCartesianAbs(LARM);
    }

  LARM.carPos[0][3] = goal_l_eetrans[0];
  LARM.carPos[1][3] = goal_l_eetrans[1];
  LARM.carPos[2][3] = goal_l_eetrans[2];

  LARM.carPos[0][0] = goal_l_eerot[0];
  LARM.carPos[0][1] = goal_l_eerot[1];
  LARM.carPos[0][2] = goal_l_eerot[2];
  LARM.carPos[1][0] = goal_l_eerot[3];
  LARM.carPos[1][1] = goal_l_eerot[4];
  LARM.carPos[1][2] = goal_l_eerot[5]; 
  LARM.carPos[2][0] = goal_l_eerot[6];
  LARM.carPos[2][1] = goal_l_eerot[7];
  LARM.carPos[2][2] = goal_l_eerot[8];

  LeftArmPTPCartesianAbs(LARM);
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::movePTPJointAbs(const DualManipulatorCommonInterface::JointPos& jointPoints)
{
  
  std::cout << "movePTPJointAbs" << std::endl;
  
  double r_targetJointPos[7];
  double l_targetJointPos[7];
  int movetime;
  double rotationLength[14];
  double maxMoveJoint;
  double maxMoveJointNumber;
  double nowRightpos[7];
  double nowLeftpos[7];
  
  for (int i = 0; i<14; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }
  

  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);
  
  r_targetJointPos[0] = (double)jointPoints[0];
  r_targetJointPos[1] = (double)jointPoints[1];
  r_targetJointPos[2] = (double)jointPoints[2];
  r_targetJointPos[3] = (double)jointPoints[3];
  r_targetJointPos[4] = (double)jointPoints[4];
  r_targetJointPos[5] = (double)jointPoints[5];
  r_targetJointPos[6] = (double)jointPoints[6];
  
  l_targetJointPos[0] = (double)jointPoints[7];
  l_targetJointPos[1] = (double)jointPoints[8];
  l_targetJointPos[2] = (double)jointPoints[9];
  l_targetJointPos[3] = (double)jointPoints[10];
  l_targetJointPos[4] = (double)jointPoints[11];
  l_targetJointPos[5] = (double)jointPoints[12];
  l_targetJointPos[6] = (double)jointPoints[13];
  
  noid.setRightJointAngle(r_targetJointPos);
  noid.setLeftJointAngle(l_targetJointPos);
  
  //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
  for(int k=0;k<14;k++)
    {
      if(k<7)
      {
        rotationLength[k] = fabs(r_targetJointPos[k]-nowRightpos[k]);
      }
      else
      {
	rotationLength[k] = fabs(r_targetJointPos[k]-nowLeftpos[k]);
      }
    }

  maxMoveJoint = rotationLength[0];
  maxMoveJointNumber = 0;
  for(int j=0;j<13;j++)
    {
      if(maxMoveJoint<rotationLength[j+1]){
	maxMoveJoint = rotationLength[j+1];
	maxMoveJointNumber = j+1;
      }
    }

  movetime = ((maxMoveJoint/m_maxSpeedJoint[maxMoveJointNumber])*1000)*(100/(double)m_speedRatioJoint);
  noid.SeedAction(movetime);
  //std::cout << "movetime = " << movetime << std::endl;
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;

}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::movePTPJointRel(const DualManipulatorCommonInterface::JointPos& jointPoints)
{
  
  std::cout << "movePTPJointRel" << std::endl;
  
  double r_targetJointPos[7];
  double l_targetJointPos[7];
  int movetime;
  double rotationLength[14];
  double maxMoveJoint;
  double maxMoveJointNumber;
  double nowRightpos[7];
  double nowLeftpos[7];
  
  for (int i = 0; i<14; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }

  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);
  
  r_targetJointPos[0] = (double)nowRightpos[0] + (double)jointPoints[0];
  r_targetJointPos[1] = (double)nowRightpos[1] + (double)jointPoints[1];
  r_targetJointPos[2] = (double)nowRightpos[2] + (double)jointPoints[2];
  r_targetJointPos[3] = (double)nowRightpos[3] + (double)jointPoints[3];
  r_targetJointPos[4] = (double)nowRightpos[4] + (double)jointPoints[4];
  r_targetJointPos[5] = (double)nowRightpos[5] + (double)jointPoints[5];
  r_targetJointPos[6] = (double)nowRightpos[6] + (double)jointPoints[6];
  
  l_targetJointPos[0] = (double)nowLeftpos[0] + (double)jointPoints[7];
  l_targetJointPos[1] = (double)nowLeftpos[1] + (double)jointPoints[8];
  l_targetJointPos[2] = (double)nowLeftpos[2] + (double)jointPoints[9];
  l_targetJointPos[3] = (double)nowLeftpos[3] + (double)jointPoints[10];
  l_targetJointPos[4] = (double)nowLeftpos[4] + (double)jointPoints[11];
  l_targetJointPos[5] = (double)nowLeftpos[5] + (double)jointPoints[12];
  l_targetJointPos[6] = (double)nowLeftpos[6] + (double)jointPoints[13];
  
  noid.setRightJointAngle(r_targetJointPos);
  noid.setLeftJointAngle(l_targetJointPos);
  
  //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
  for(int k=0;k<14;k++)
    {
      if(k<7)
      {
        rotationLength[k] = fabs(r_targetJointPos[k]-nowRightpos[k]);
      }
      else
      {
	rotationLength[k] = fabs(r_targetJointPos[k]-nowLeftpos[k]);
      }
    }

  maxMoveJoint = rotationLength[0];
  maxMoveJointNumber = 0;
  for(int j=0;j<13;j++)
    {
      if(maxMoveJoint<rotationLength[j+1]){
	maxMoveJoint = rotationLength[j+1];
	maxMoveJointNumber = j+1;
      }
    }

  movetime = ((maxMoveJoint/m_maxSpeedJoint[maxMoveJointNumber])*1000)*(100/(double)m_speedRatioJoint);
  noid.SeedAction(movetime);
  //std::cout << "movetime = " << movetime << std::endl;
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::movePTPJointAbsSeq(const DualManipulatorCommonInterface::JointPosSeq& jointPointsSeq)
{
  DualManipulatorCommonInterface::RETURN_ID* result;
  std::cout << "Unimplemented..." << std::endl;
  return result;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::openGripper()
{
  
  std::cout << "closeGripper" << std::endl;
 
  noid.setRightHandCurrent(100);
  noid.setLeftHandCurrent(100);
  usleep(1000*10);
  noid.OpenRightGripper();
  noid.OpenLeftGripper();
  //sleep(5);
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
  
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::setSpeedCartesian(DualManipulatorCommonInterface::ULONG spdRatio)
{
  std::cout<<"setSpeedCartesian"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioCartesian = spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_NG;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::setSpeedJoint(DualManipulatorCommonInterface::ULONG spdRatio)
{
  
  std::cout<<"getSpeedJoint"<<std::endl;
  spdRatio = m_speedRatioJoint;
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::RightArmPTPCartesianAbs(const DualManipulatorCommonInterface::CarPosWithElbow& carPoint)
{
  std::cout<<"RightArmPTPCartesianAbs"<<std::endl;
  
  double eerot[9];//姿勢
  double eetrans[3];//座標
  double iksol[7];
  double elbow;
  double nowJointPos[7];
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  int movetime;
  double nowRightpos[7];
  
  noid.readRightJointAngle(nowRightpos);
  
  //姿勢の入力
  eerot[0]=carPoint.carPos[0][0];
  eerot[1]=carPoint.carPos[0][1];
  eerot[2]=carPoint.carPos[0][2];
  
  eerot[3]=carPoint.carPos[1][0];
  eerot[4]=carPoint.carPos[1][1];
  eerot[5]=carPoint.carPos[1][2];
  
  eerot[6]=carPoint.carPos[2][0];
  eerot[7]=carPoint.carPos[2][1];
  eerot[8]=carPoint.carPos[2][2];

  for(int j=0;j<9;j++){
    printf("eerot[%d] = %f\n",j,eerot[j]);
  }

  //座標の入力 関数への入力は[mm]だがikfastは[m]のため変換
  eetrans[0]=carPoint.carPos[0][3];
  eetrans[1]=carPoint.carPos[1][3];
  eetrans[2]=carPoint.carPos[2][3];

  for(int i=0;i<3;i++){
    printf("eetrans[%d] = %f\n",i,eetrans[i]);
  }

  for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowRightpos[d];
    }

  elbow = carPoint.elbow;

  int ret = noid.Solve_RightArmIk(eerot,eetrans,elbow,nowJointPos,iksol);
  if(!ret)
    {
      //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
      for(int k=0;k<7;k++)
	{
	  rotationLength[k] = fabs(iksol[k]-nowRightpos[k]);
	}
      maxMoveJoint = rotationLength[0];
      maxMoveJointNumber = 0;
  for(int j=0;j<6;j++)
    {
      if(maxMoveJoint<rotationLength[j+1]){
	maxMoveJoint = rotationLength[j+1];
	maxMoveJointNumber = j+1;
      }
    }
  movetime = ((maxMoveJoint/m_maxSpeedJoint[maxMoveJointNumber])*1000)*(100/(double)m_speedRatioJoint);
      noid.setRightJointAngle(iksol);
      noid.SeedAction(movetime);
    }
  else
    RETURNID_NG;

  std::cout << "Success" <<std::endl;
  RETURNID_OK;
}

DualManipulatorCommonInterface::RETURN_ID* DualManipulatorCommonInterface_MotionCommandsSVC_impl::LeftArmPTPCartesianAbs(const DualManipulatorCommonInterface::CarPosWithElbow& carPoint)
{
  std::cout << "LeftArmPTPCartesianAbs" << std::endl;

  double eerot[9];//姿勢
  double eetrans[3];//座標
  double iksol[7];
  double elbow;
  double nowJointPos[7];
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  int movetime;
  double nowLeftpos[7];
  
  noid.readLeftJointAngle(nowLeftpos);

  //姿勢の入力
  eerot[0]=carPoint.carPos[0][0];
  eerot[1]=carPoint.carPos[0][1];
  eerot[2]=carPoint.carPos[0][2];
  
  eerot[3]=carPoint.carPos[1][0];
  eerot[4]=carPoint.carPos[1][1];
  eerot[5]=carPoint.carPos[1][2];
  
  eerot[6]=carPoint.carPos[2][0];
  eerot[7]=carPoint.carPos[2][1];
  eerot[8]=carPoint.carPos[2][2];

  for(int j=0;j<9;j++){
    printf("eerot[%d] = %f\n",j,eerot[j]);
  }

  //座標の入力 関数への入力は[mm]だがikfastは[m]のため変換
  eetrans[0]=carPoint.carPos[0][3];
  eetrans[1]=carPoint.carPos[1][3];
  eetrans[2]=carPoint.carPos[2][3];

  for(int i=0;i<3;i++){
    printf("eetrans[%d] = %f\n",i,eetrans[i]);
  }

  for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowLeftpos[d];
    }

  elbow = carPoint.elbow;

  int ret = noid.Solve_LeftArmIk(eerot,eetrans,elbow,nowJointPos,iksol);
  if(!ret)
    {
      //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
      for(int k=0;k<7;k++)
	{
	  rotationLength[k] = fabs(iksol[k]-nowLeftpos[k]);
	}
      maxMoveJoint = rotationLength[0];
      maxMoveJointNumber = 0;
  for(int j=0;j<6;j++)
    {
      if(maxMoveJoint<rotationLength[j+1]){
	maxMoveJoint = rotationLength[j+1];
	maxMoveJointNumber = j+1;
      }
    }
  movetime = ((maxMoveJoint/m_maxSpeedJoint[maxMoveJointNumber])*1000)*(100/(double)m_speedRatioJoint);
      noid.setLeftJointAngle(iksol);
      noid.SeedAction(movetime);
    }
  else
    RETURNID_NG;

  std::cout << "Success" <<std::endl;
  RETURNID_OK;
}

// End of example implementational code




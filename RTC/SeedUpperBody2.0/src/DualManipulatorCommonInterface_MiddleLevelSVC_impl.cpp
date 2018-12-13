// -*-C++-*-
/*!
 * @file  DualManipulatorCommonInterface_MiddleLevelSVC_impl.cpp
 * @brief Service implementation code of DualManipulatorCommonInterface_MiddleLevel.idl
 *
 */

#include "DualManipulatorCommonInterface_MiddleLevelSVC_impl.h"
#include "LibSeednoidUpperUnit1.h"
#include "right_ReturnID.h"
#include "left_ReturnID.h"
#include "dual_ReturnID.h"

/*
 * Example implementational code for IDL interface JARA_ARM::ManipulatorCommonInterface_Middle
 */
JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl()
{
  m_speedRatioCartesian = 50;
  m_speedRatioJoint = 50;
  JARA_ARM::ManipInfo_var mInfovar;
  JARA_ARM::ManipInfo mInfo;
  m_ManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  m_maxSpeedJoint.length(mInfo.axisNum);
  m_homeJoint.length(mInfo.axisNum);
  m_homeJoint[0] = 0;
  m_homeJoint[1] = 0;
  m_homeJoint[2] = 5;
  m_homeJoint[3] = 160;
  m_homeJoint[4] = 0;
  m_homeJoint[5] = 0;
  m_homeJoint[6] = 0;
  for(unsigned int i=0;i<mInfo.axisNum;i++){
    m_maxSpeedJoint[i] = 60;//deg/s
  }
}


JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::~JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::closeGripper()
{
  std::cout << "closeRightGripper" << std::endl;
 
  noid.setRightHandCurrent(100);
  usleep(1000*10);
  noid.CloseRightGripper();
  //sleep(5);
  
  std::cout << "Success" << std::endl << std::endl;
  
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM::HgMatrix offset)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM::CarPosWithElbow& pos)
{
  std::cout<<"getFeedbackPosCartesian"<<std::endl;
  
  double JointPos[7];
  noid.readRightJointAngle(JointPos);
  
  double eerot[9];
  double eetrans[3];
  noid.Solve_RightArmFk(eerot,eetrans,JointPos);

  //1列目
  pos.carPos[0][0]=eerot[0];
  pos.carPos[0][1]=eerot[1];
  pos.carPos[0][2]=eerot[2];
  
  //2列目
  pos.carPos[1][0]=eerot[3];
  pos.carPos[1][1]=eerot[4];
  pos.carPos[1][2]=eerot[5];
  
  //3列目
  pos.carPos[2][0]=eerot[6];
  pos.carPos[2][1]=eerot[7];
  pos.carPos[2][2]=eerot[8];
  
  //4列目
  pos.carPos[0][3]=eetrans[0]*1000;//[mm]
  pos.carPos[1][3]=eetrans[1]*1000;
  pos.carPos[2][3]=eetrans[2]*1000;
  
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM::CartesianSpeed& speed)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM::DoubleSeq_out speed)
{
  std::cout<<"getMaxSpeedJoint"<<std::endl;
  speed = new JARA_ARM::DoubleSeq;
  speed->length(7);
  for(unsigned int i=0;i<7;i++){
    speed[i] = m_maxSpeedJoint[i];
  }
  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM::LimitValue& xLimit, JARA_ARM::LimitValue& yLimit, JARA_ARM::LimitValue& zLimit)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM::ULONG angleRatio)
{
  std::cout << "moveGripper" << std::endl;
  double move;
  
  move = (double)angleRatio;
  
  if (angleRatio>0 && angleRatio <= 100){
    noid.MoveRightGripper(move);
  }
  else{
    std::cout << "ERROR : angleRatio Wrong Value" << std::endl;
    R_RETURNID_VALUE_ERR;
  }
  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)
{
  std::cout<<"movePTPCartesianAbs"<<std::endl;
  
  double eerot[9];//姿勢
  double eetrans[3];//座標
  double iksol[7];
  double elbow;
  double nowJointPos[7];
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  int movetime;
  
  JARA_ARM::JointPos_var nowRightposvar;
  JARA_ARM::JointPos nowRightpos;
  
  //右腕現在値取得
  m_rid_right=m_ManipulatorCommonInterface_Common.getFeedbackPosJoint(nowRightposvar);
  if(m_rid_right->id != 0){//Error
    std::cout<<"RightgetFeedbackPosJoint ERROR"<<std::endl;
    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
  }
  nowRightpos = nowRightposvar;
  
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
    R_RETURNID_NG;

  std::cout << "Success" <<std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM::JointPos& jointPoints)
{
  std::cout << "movePTPJointAbs" << std::endl;
  
  double targetJointPos[7];
  int movetime;
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  
  for (int i = 0; i<7; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }
  
  JARA_ARM::JointPos_var nowRightposvar;
  JARA_ARM::JointPos nowRightpos;
  //右腕
  m_rid_right=m_ManipulatorCommonInterface_Common.getFeedbackPosJoint(nowRightposvar);
  if(m_rid_right->id != 0){//Error
    std::cout<<"RightgetFeedbackPosJoint ERROR"<<std::endl;
    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
  }
  nowRightpos = nowRightposvar;
  
  targetJointPos[0] = (double)jointPoints[0];
  targetJointPos[1] = (double)jointPoints[1];
  targetJointPos[2] = (double)jointPoints[2];
  targetJointPos[3] = (double)jointPoints[3];
  targetJointPos[4] = (double)jointPoints[4];
  targetJointPos[5] = (double)jointPoints[5];
  targetJointPos[6] = (double)jointPoints[6];
  
  noid.setRightJointAngle(targetJointPos);
  
  //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
  for(int k=0;k<7;k++)
    {
      rotationLength[k] = fabs(targetJointPos[k]-nowRightpos[k]);
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
  noid.SeedAction(movetime);
  //std::cout << "movetime = " << movetime << std::endl;
  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::openGripper()
{
  std::cout << "openRightGripper" << std::endl;
  
  noid.OpenRightGripper();
  //sleep(3);
  
  std::cout << "Success" << std::endl << std::endl;
  
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::pause()
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::resume()
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::stop()
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM::HgMatrix offset)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM::HgMatrix offset)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM::CartesianSpeed& speed)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM::DoubleSeq& speed)
{
  std::cout<<"setMaxSpeedJoint"<<std::endl;
  
  JARA_ARM::ManipInfo_var mInfovar;
  JARA_ARM::ManipInfo mInfo;
  m_ManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  
  m_maxSpeedJointFlag = true;
  
  if(speed.length() != (mInfo.axisNum)){
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    R_RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<speed.length();i++){
    m_maxSpeedJoint[i] = speed[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM::LimitValue& xLimit, const JARA_ARM::LimitValue& yLimit, const JARA_ARM::LimitValue& zLimit)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(JARA_ARM::ULONG spdRatio)
{
  std::cout<<"setSpeedCartesian"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioCartesian = spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    R_RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM::ULONG spdRatio)
{
  std::cout<<"setSpeedJoint"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioJoint = spdRatio;
  }
  else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    R_RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM::JointPos& jointPoint)
{
  std::cout<<"setHome"<<std::endl;
  
  JARA_ARM::ManipInfo_var mInfovar;
  JARA_ARM::ManipInfo mInfo;
  m_ManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  
  m_maxSpeedJointFlag = true;
  
  if(jointPoint.length() != (mInfo.axisNum)){
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    R_RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<jointPoint.length();i++){
    m_homeJoint[i] = jointPoint[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM::JointPos_out jointPoint)
{
	JARA_ARM::RETURN_ID* result;
 
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::goHome()
{
  std::cout<<"goHome"<<std::endl;
  double targetJointPos[7];
 
  targetJointPos[0] = (double)m_homeJoint[0];
  targetJointPos[1] = (double)m_homeJoint[1];
  targetJointPos[2] = (double)m_homeJoint[2];
  targetJointPos[3] = (double)m_homeJoint[3];
  targetJointPos[4] = (double)m_homeJoint[4];
  targetJointPos[5] = (double)m_homeJoint[5];
  targetJointPos[6] = (double)m_homeJoint[6];
  
  noid.setRightJointAngle(targetJointPos);
  
  noid.SeedAction(3000);

  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSpeedCartesian(JARA_ARM::ULONG& spdRatio)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSpeedJoint(JARA_ARM::ULONG& spdRatio)
{
  std::cout<<"getSpeedJoint"<<std::endl;
  spdRatio = m_speedRatioJoint;
  std::cout<<"Success"<<std::endl<<std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM::ULONG forceRatio)
{
  std::cout<<"setGraspForce"<<std::endl;
  int ratio = (int)forceRatio;
  m_forceRatio = forceRatio;
  noid.setRightHandCurrent(ratio);
  std::cout<<"Success"<<std::endl<<std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM::ULONG& forceRatio)
{
  std::cout<<"getGraspForce"<<std::endl;
  forceRatio = m_forceRatio;
  std::cout<<"Success"<<std::endl<<std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbsCmdCycle(const JARA_ARM::JointPos& jointPoints)
{
  std::cout << "movePTPJointAbsCmdCycle" << std::endl;
  
  double targetJointPos[7];
  int movetime;
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  
  for (int i = 0; i<7; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }

  JARA_ARM::ManipInfo_var mInfovar;
  JARA_ARM::ManipInfo mInfo;
  m_ManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  
  targetJointPos[0] = (double)jointPoints[0];
  targetJointPos[1] = (double)jointPoints[1];
  targetJointPos[2] = (double)jointPoints[2];
  targetJointPos[3] = (double)jointPoints[3];
  targetJointPos[4] = (double)jointPoints[4];
  targetJointPos[5] = (double)jointPoints[5];
  targetJointPos[6] = (double)jointPoints[6];
  
  noid.setRightJointAngle(targetJointPos);
  
  movetime = 1000/mInfo.cmdCycle;
  noid.SeedAction(movetime);
  //std::cout << "movetime = " << movetime << std::endl;
  //noid.SeedAction(1000/50);
  std::cout << "Success" << std::endl << std::endl;
  R_RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle
 */
JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl()
{
  m_speedRatioCartesian = 50;
  m_speedRatioJoint = 50;
  JARA_ARM_LEFT::ManipInfo_var mInfovar;
  JARA_ARM_LEFT::ManipInfo mInfo;
  m_LeftManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  m_maxSpeedJoint.length(mInfo.axisNum);
  m_homeJoint.length(mInfo.axisNum);
  m_homeJoint[0] = 0;
  m_homeJoint[1] = 0;
  m_homeJoint[2] = -5;
  m_homeJoint[3] = 160;
  m_homeJoint[4] = 0;
  m_homeJoint[5] = 0;
  m_homeJoint[6] = 0;
  for(unsigned int i=0;i<mInfo.axisNum;i++){
    m_maxSpeedJoint[i] = 60;//deg/s
  }
}


JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::~JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::closeGripper()
{
  std::cout << "closeLeftGripper" << std::endl;
  noid.setLeftHandCurrent(100);
  usleep(1000*10);
  noid.CloseLeftGripper();
  
  std::cout << "Success" << std::endl << std::endl;
  
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM_LEFT::HgMatrix offset)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM_LEFT::CarPosWithElbow& pos)
{
  std::cout<<"getFeedbackPosCartesian"<<std::endl;
  
  double JointPos[7];
  noid.readLeftJointAngle(JointPos);
  
  double eerot[9];
  double eetrans[3];
  noid.Solve_LeftArmFk(eerot,eetrans,JointPos);

  //1列目
  pos.carPos[0][0]=eerot[0];
  pos.carPos[0][1]=eerot[1];
  pos.carPos[0][2]=eerot[2];
  
  //2列目
  pos.carPos[1][0]=eerot[3];
  pos.carPos[1][1]=eerot[4];
  pos.carPos[1][2]=eerot[5];
  
  //3列目
  pos.carPos[2][0]=eerot[6];
  pos.carPos[2][1]=eerot[7];
  pos.carPos[2][2]=eerot[8];
  
  //4列目
  pos.carPos[0][3]=eetrans[0]*1000;//[mm]
  pos.carPos[1][3]=eetrans[1]*1000;
  pos.carPos[2][3]=eetrans[2]*1000;
  
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM_LEFT::CartesianSpeed& speed)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM_LEFT::DoubleSeq_out speed)
{
  std::cout<<"getMaxSpeedJoint"<<std::endl;
  speed = new JARA_ARM_LEFT::DoubleSeq;
  speed->length(7);
  for(unsigned int i=0;i<7;i++){
    speed[i] = m_maxSpeedJoint[i];
  }
  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM_LEFT::LimitValue& xLimit, JARA_ARM_LEFT::LimitValue& yLimit, JARA_ARM_LEFT::LimitValue& zLimit)
{
	JARA_ARM_LEFT::RETURN_ID* result;
 
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_LEFT::ULONG angleRatio)
{
  std::cout << "moveGripper" << std::endl;
  double move;
  
  move = (double)angleRatio;
  
  if (angleRatio>0 && angleRatio <= 100){
    noid.MoveLeftGripper(move);
  }
  else{
    std::cout << "ERROR : angleRatio Wrong Value" << std::endl;
    L_RETURNID_VALUE_ERR
  }
  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
  std::cout<<"movePTPCartesianAbs"<<std::endl;
  
  double eerot[9];//姿勢
  double eetrans[3];//座標
  double iksol[7];
  double elbow;
  double nowJointPos[7];
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  int movetime;
  
  JARA_ARM_LEFT::JointPos_var nowLeftposvar;
  JARA_ARM_LEFT::JointPos nowLeftpos;
  //左腕現在値取得
  m_rid_left=m_LeftManipulatorCommonInterface_Common.getFeedbackPosJoint(nowLeftposvar);
  if(m_rid_left->id != 0){//Error
    std::cout<<"LeftgetFeedbackPosJoint ERROR"<<std::endl;
    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
  }
  nowLeftpos = nowLeftposvar;

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
  
  elbow = carPoint.elbow;

    for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowLeftpos[d];
    }
    
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
    L_RETURNID_NG;
  
  std::cout << "Success" <<std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM_LEFT::JointPos& jointPoints)
{
  std::cout << "movePTPJointAbs" << std::endl;

  double targetJointPos[7];
  int movetime;
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;

  for (int i = 0; i<7; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }

  JARA_ARM_LEFT::JointPos_var nowLeftposvar;
  JARA_ARM_LEFT::JointPos nowLeftpos;
  //左腕現在値取得
  m_rid_left=m_LeftManipulatorCommonInterface_Common.getFeedbackPosJoint(nowLeftposvar);
  if(m_rid_left->id != 0){//Error
    std::cout<<"LeftgetFeedbackPosJoint ERROR"<<std::endl;
    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
  }
  nowLeftpos = nowLeftposvar;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  targetJointPos[0] = (double)jointPoints[0];
  targetJointPos[1] = (double)jointPoints[1];
  targetJointPos[2] = (double)jointPoints[2];
  targetJointPos[3] = (double)jointPoints[3];
  targetJointPos[4] = (double)jointPoints[4];
  targetJointPos[5] = (double)jointPoints[5];
  targetJointPos[6] = (double)jointPoints[6];
  
  //std::cout << "setJointAngle_Middle" << std::endl;
  noid.setLeftJointAngle(targetJointPos);
  
  //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
  for(int k=0;k<7;k++)
    {
      rotationLength[k] = fabs(targetJointPos[k]-nowLeftpos[k]);
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
  noid.SeedAction(movetime);
  
  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_LEFT::JointPos& jointPoints)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::openGripper()
{
  std::cout << "openLeftGripper" << std::endl;
  noid.OpenLeftGripper();
  
  std::cout << "Success" << std::endl << std::endl;
  
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::pause()
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::resume()
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::stop()
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM_LEFT::HgMatrix offset)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM_LEFT::HgMatrix offset)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM_LEFT::CartesianSpeed& speed)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM_LEFT::DoubleSeq& speed)
{
  std::cout<<"setMaxSpeedJoint"<<std::endl;

  JARA_ARM_LEFT::ManipInfo_var mInfovar;
  JARA_ARM_LEFT::ManipInfo mInfo;
  m_LeftManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;

  m_maxSpeedJointFlag = true;
  
  if(speed.length() != (mInfo.axisNum)){
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    L_RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<speed.length();i++){
    m_maxSpeedJoint[i] = speed[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM_LEFT::LimitValue& xLimit, const JARA_ARM_LEFT::LimitValue& yLimit, const JARA_ARM_LEFT::LimitValue& zLimit)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(JARA_ARM_LEFT::ULONG spdRatio)
{
  std::cout<<"setSpeedCartesian"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioCartesian = spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    L_RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM_LEFT::ULONG spdRatio)
{
  std::cout<<"setSpeedJoint"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioJoint = spdRatio;
  }
  else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    L_RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM_LEFT::JointPos& jointPoint)
{
  std::cout<<"setHome"<<std::endl;
  
  JARA_ARM_LEFT::ManipInfo_var mInfovar;
  JARA_ARM_LEFT::ManipInfo mInfo;
  m_LeftManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  
  m_maxSpeedJointFlag = true;
  
  if(jointPoint.length() != (mInfo.axisNum)){
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    L_RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<jointPoint.length();i++){
    m_homeJoint[i] = jointPoint[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM_LEFT::JointPos_out jointPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::goHome()
{
  std::cout<<"goHome"<<std::endl;
  double targetJointPos[7];
 
  targetJointPos[0] = (double)m_homeJoint[0];
  targetJointPos[1] = (double)m_homeJoint[1];
  targetJointPos[2] = (double)m_homeJoint[2];
  targetJointPos[3] = (double)m_homeJoint[3];
  targetJointPos[4] = (double)m_homeJoint[4];
  targetJointPos[5] = (double)m_homeJoint[5];
  targetJointPos[6] = (double)m_homeJoint[6];
  
  noid.setLeftJointAngle(targetJointPos);
  
  noid.SeedAction(3000);
  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSpeedCartesian(JARA_ARM_LEFT::ULONG& spdRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSpeedJoint(JARA_ARM_LEFT::ULONG& spdRatio)
{
  std::cout<<"getSpeedJoint"<<std::endl;
  spdRatio = m_speedRatioJoint;
  std::cout<<"Success"<<std::endl<<std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM_LEFT::ULONG forceRatio)
{
  std::cout<<"setGraspForce"<<std::endl;
  int ratio = (int)forceRatio;
  m_forceRatio = forceRatio;
  noid.setLeftHandCurrent(ratio);
  std::cout<<"Success"<<std::endl<<std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM_LEFT::ULONG& forceRatio)
{
  std::cout<<"getGraspForce"<<std::endl;
  forceRatio = m_forceRatio;
  std::cout<<"Success"<<std::endl<<std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbsCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints)
{
  std::cout << "movePTPJointAbsCmdCycle" << std::endl;

  double targetJointPos[7];
  int movetime;
  double rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;

  for (int i = 0; i<7; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }

  JARA_ARM_LEFT::ManipInfo_var mInfovar;
  JARA_ARM_LEFT::ManipInfo mInfo;
  m_LeftManipulatorCommonInterface_Common.getManipInfo(mInfovar);
  mInfo = mInfovar;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  targetJointPos[0] = (double)jointPoints[0];
  targetJointPos[1] = (double)jointPoints[1];
  targetJointPos[2] = (double)jointPoints[2];
  targetJointPos[3] = (double)jointPoints[3];
  targetJointPos[4] = (double)jointPoints[4];
  targetJointPos[5] = (double)jointPoints[5];
  targetJointPos[6] = (double)jointPoints[6];
  
  //std::cout << "setJointAngle_Middle" << std::endl;
  noid.setLeftJointAngle(targetJointPos);
  
  movetime = 1000/mInfo.cmdCycle;
  noid.SeedAction(movetime);
  
  std::cout << "Success" << std::endl << std::endl;
  L_RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}




// End of example implementational code

/*
 * Example implementational code for IDL interface JARA_ARM_DUAL::DualManipulatorCommonInterface_Middle
 */
JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl()
{
  m_speedRatioCartesian = 50;
  m_speedRatioJoint = 50;
  m_maxSpeedJoint.length(7);
  for(unsigned int i=0;i<7;i++){
    m_maxSpeedJoint[i] = 60;//deg/s
  }
}


JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::~JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::closeGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)
{

  std::cout << "closeGripper" << std::endl;
  noid.setRightHandCurrent(100);
  noid.setLeftHandCurrent(100);
  usleep(1000*10);
  
  if(rArm == 1 && lArm == 1)
    {
      noid.CloseRightGripper();
      noid.CloseLeftGripper();
    }
  if(rArm == 1 && lArm == 0)
    {
      noid.CloseRightGripper();
    }
  if(rArm == 0 && lArm == 1)
    {
      noid.CloseLeftGripper();
    }
  
  std::cout << "Success" << std::endl << std::endl;

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::getRelativePosition(const JARA_ARM_DUAL::HgMatrix RelPos)
{
	JARA_ARM_DUAL::RETURN_ID* result;

  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)
{
  std::cout << "moveGripper" << std::endl;

  double r_move;
  double l_move;
  
  r_move = (double)rArm;
  l_move = (double)lArm;
  
  if (r_move > 0 && r_move <= 100 && l_move>0 && l_move <= 100){
    noid.MoveRightGripper(r_move);
    noid.MoveLeftGripper(l_move);
  }

  else{
    std::cout << "ERROR : angleRatio Wrong Value" << std::endl;
  }
  
  std::cout << "Success" << std::endl << std::endl;

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)
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

  JARA_ARM_DUAL::CarPosWithElbow RARM;
  JARA_ARM_DUAL::CarPosWithElbow LARM;

  carPointToTransRot(rArm, goal_r_eerot, goal_r_eetrans);
  carPointToTransRot(lArm, goal_l_eerot, goal_l_eetrans);

  r_elbow = rArm.elbow;
  l_elbow = lArm.elbow;
  
  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);

  noid.Solve_RightArmFk(now_r_eerot,now_r_eetrans,nowRightpos);
  noid.Solve_LeftArmFk(now_l_eerot,now_l_eetrans,nowLeftpos);

  for(int eetnum=0;eetnum<3;eetnum++)
    {
      r_cartesianLength[eetnum] = fabs(now_r_eetrans[eetnum] - goal_r_eetrans[eetnum]);
      l_cartesianLength[eetnum] = fabs(now_l_eetrans[eetnum] - goal_l_eetrans[eetnum]);
    }
  
  r_maxCartesianLength = CalcmaxCartesianLength(r_cartesianLength);
  l_maxCartesianLength = CalcmaxCartesianLength(l_cartesianLength);

  r_vianum = (int)r_maxCartesianLength/5;
  l_vianum = (int)l_maxCartesianLength/5;

  RARM.elbow = r_elbow;
  LARM.elbow = l_elbow;

  for(int viacnt=0; viacnt < r_vianum; viacnt++)
    {

      for(int eetnum=0; eetnum<3; eetnum++)
	{
	  via_r_eetrans[eetnum] = now_r_eetrans[eetnum] + (goal_r_eetrans[eetnum] - now_r_eetrans[eetnum])/r_vianum*viacnt;
	}
      
      TransRotTocarPoint(RARM, now_r_eerot, via_r_eetrans);

      RightArmPTPCartesianAbs(RARM);

      if(l_vianum>viacnt)
	{
	  for(int eetnum=0; eetnum<3; eetnum++)
	    {
	      via_l_eetrans[eetnum] = now_l_eetrans[eetnum] + (goal_l_eetrans[eetnum] - now_l_eetrans[eetnum])/l_vianum*viacnt;
	    }
      
	  TransRotTocarPoint(LARM, now_l_eerot, via_l_eetrans);

	  LeftArmPTPCartesianAbs(LARM);
	}
    }

  TransRotTocarPoint(RARM, goal_r_eerot, goal_r_eetrans);
  TransRotTocarPoint(LARM, goal_l_eerot, goal_l_eetrans);
  
  RightArmPTPCartesianAbs(RARM);
  LeftArmPTPCartesianAbs(LARM);

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)
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

  JARA_ARM_DUAL::CarPosWithElbow RARM;
  JARA_ARM_DUAL::CarPosWithElbow LARM;
 
  carPointToTransRot(rArm, goal_r_eerot, goal_r_eetrans);
  carPointToTransRot(lArm, goal_l_eerot, goal_l_eetrans);

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

  r_maxCartesianLength = CalcmaxCartesianLength(r_cartesianLength);
  l_maxCartesianLength = CalcmaxCartesianLength(l_cartesianLength);


  r_vianum = (int)r_maxCartesianLength/5;
  l_vianum = (int)l_maxCartesianLength/5;


  RARM.elbow = r_elbow;
  LARM.elbow = l_elbow;

  for(int r_viacnt=0; r_viacnt < r_vianum; r_viacnt++)
    {

      for(int eetnum=0; eetnum<3; eetnum++)
	{
	  via_r_eetrans[eetnum] = now_r_eetrans[eetnum] + (goal_r_eetrans[eetnum] - now_r_eetrans[eetnum])/r_vianum*r_viacnt;
	}
      
      TransRotTocarPoint(RARM, now_r_eerot, via_r_eetrans);

      RightArmPTPCartesianAbs(RARM);
    }

  TransRotTocarPoint(RARM, goal_r_eerot, goal_r_eetrans);

  RightArmPTPCartesianAbs(RARM);

  
  for(int l_viacnt=0; l_viacnt < l_vianum; l_viacnt++)
    {
      
      for(int eetnum=0; eetnum<3; eetnum++)
	{
	  via_l_eetrans[eetnum] = now_l_eetrans[eetnum] + (goal_l_eetrans[eetnum] - now_l_eetrans[eetnum])/l_vianum*l_viacnt;
	}
      
      TransRotTocarPoint(LARM, now_l_eerot, via_l_eetrans);

      LeftArmPTPCartesianAbs(LARM);
    }

  TransRotTocarPoint(LARM, goal_l_eerot, goal_l_eetrans);
  LeftArmPTPCartesianAbs(LARM);

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)
{
	JARA_ARM_DUAL::RETURN_ID* result;

  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)
{
	JARA_ARM_DUAL::RETURN_ID* result;

  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm)
{
  std::cout << "movePTPJointAbs" << std::endl;
  
  double r_targetJointPos[7];
  double l_targetJointPos[7];
  int movetime;
  int r_movetime;
  int l_movetime;
  double r_rotationLength[7];
  double l_rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  double nowRightpos[7];
  double nowLeftpos[7];
  

  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);

  std::cout << "1" << std::endl;

  for(int posnum=0; posnum<7; posnum++)
    {
      r_targetJointPos[posnum] = (double)rArm[posnum];
      l_targetJointPos[posnum] = (double)lArm[posnum];
    }

  std::cout << "2" << std::endl;
  
  noid.setRightJointAngle(r_targetJointPos);
  noid.setLeftJointAngle(l_targetJointPos);

  std::cout << "3" << std::endl;
  
  for(int lengthnum=0; lengthnum<7; lengthnum++)
    {
      r_rotationLength[lengthnum] = fabs(r_targetJointPos[lengthnum]-nowRightpos[lengthnum]);
    }
  for(int lengthnum=0; lengthnum<7; lengthnum++)
    {
      l_rotationLength[lengthnum] = fabs(l_targetJointPos[lengthnum]-nowLeftpos[lengthnum]);
    }

  std::cout << "4" << std::endl;

  r_movetime = Calcmovetime(r_rotationLength);
  l_movetime = Calcmovetime(l_rotationLength);

  if(r_movetime >= l_movetime)
    {
      movetime = r_movetime;
    }
  else
    {
      movetime = l_movetime;
    }
  
  noid.SeedAction(movetime);

  std::cout << "Success" << std::endl << std::endl;

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm)
{
  std::cout << "movePTPJointRel" << std::endl;
  
  double r_targetJointPos[7];
  double l_targetJointPos[7];
  int movetime;
  int r_movetime;
  int l_movetime;
  double r_rotationLength[7];
  double l_rotationLength[7];
  double maxMoveJoint;
  double maxMoveJointNumber;
  double nowRightpos[7];
  double nowLeftpos[7];

  noid.readRightJointAngle(nowRightpos);
  noid.readLeftJointAngle(nowLeftpos);

  for(int posnum=0; posnum<7; posnum++)
    {
      r_targetJointPos[posnum] = (double)nowRightpos[posnum] + (double)rArm[posnum];
      l_targetJointPos[posnum] = (double)nowLeftpos[posnum] + (double)lArm[posnum];
    }
  
  noid.setRightJointAngle(r_targetJointPos);
  noid.setLeftJointAngle(l_targetJointPos);
  
  for(int lengthnum=0; lengthnum<7; lengthnum++)
    {
      r_rotationLength[lengthnum] = fabs(r_targetJointPos[lengthnum]-nowRightpos[lengthnum]);
    }
  for(int lengthnum=0; lengthnum<7; lengthnum++)
    {
      l_rotationLength[lengthnum] = fabs(l_targetJointPos[lengthnum]-nowLeftpos[lengthnum]);
    }

  r_movetime = Calcmovetime(r_rotationLength);
  l_movetime = Calcmovetime(l_rotationLength);

  if(r_movetime >= l_movetime)
    {
      movetime = r_movetime;
    }
  else
    {
      movetime = l_movetime;
    }
  
  noid.SeedAction(movetime);

  std::cout << "Success" << std::endl << std::endl;

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::openGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)
{
  std::cout << "openGripper" << std::endl;
  noid.setRightHandCurrent(100);
  noid.setLeftHandCurrent(100);
  usleep(1000*10);
  
  if(rArm == 1 && lArm == 1)
    {
      noid.OpenRightGripper();
      noid.OpenLeftGripper();
    }
  if(rArm == 1 && lArm == 0)
    {
      noid.OpenRightGripper();
    }
  if(rArm == 0 && lArm == 1)
    {
      noid.OpenLeftGripper();
    }
  
  std::cout << "Success" << std::endl << std::endl;

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::RightArmPTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& carPoint)
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
  
  carPointToTransRot(carPoint, eerot, eetrans);

  for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowRightpos[d];
    }

  elbow = carPoint.elbow;

  int ret = noid.Solve_RightArmIk(eerot,eetrans,elbow,nowJointPos,iksol);
  if(!ret)
    {
      for(int k=0;k<7;k++)
	{
	  rotationLength[k] = fabs(iksol[k]-nowRightpos[k]);
	}
      movetime = Calcmovetime(rotationLength);
      noid.setRightJointAngle(iksol);
      noid.SeedAction(movetime);
    }
  else
    D_RETURNID_NG;

  std::cout << "Success" <<std::endl;
  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::LeftArmPTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& carPoint)
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

  carPointToTransRot(carPoint, eerot, eetrans);

  for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowLeftpos[d];
    }

  elbow = carPoint.elbow;

  int ret = noid.Solve_LeftArmIk(eerot,eetrans,elbow,nowJointPos,iksol);
  if(!ret)
    {
      for(int k=0;k<7;k++)
	{
	  rotationLength[k] = fabs(iksol[k]-nowLeftpos[k]);
	}
      movetime = Calcmovetime(rotationLength);
      noid.setLeftJointAngle(iksol);
      noid.SeedAction(movetime);
    }
  else
    D_RETURNID_NG;

  std::cout << "Success" <<std::endl;
  D_RETURNID_OK;
}



void JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::carPointToTransRot(const JARA_ARM_DUAL::CarPosWithElbow& carPoint, double eerot[9], double eetrans[3])
{
  eerot[0]=carPoint.carPos[0][0];
  eerot[1]=carPoint.carPos[0][1];
  eerot[2]=carPoint.carPos[0][2];
  
  eerot[3]=carPoint.carPos[1][0];
  eerot[4]=carPoint.carPos[1][1];
  eerot[5]=carPoint.carPos[1][2];
  
  eerot[6]=carPoint.carPos[2][0];
  eerot[7]=carPoint.carPos[2][1];
  eerot[8]=carPoint.carPos[2][2];

  eetrans[0]=carPoint.carPos[0][3];
  eetrans[1]=carPoint.carPos[1][3];
  eetrans[2]=carPoint.carPos[2][3];
}

void JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::TransRotTocarPoint(JARA_ARM_DUAL::CarPosWithElbow& carPoint, double eerot[9], double eetrans[3])
{
  carPoint.carPos[0][0] = eerot[0];
  carPoint.carPos[0][1] = eerot[1];
  carPoint.carPos[0][2] = eerot[2];
  
  carPoint.carPos[1][0] = eerot[3];
  carPoint.carPos[1][1] = eerot[4];
  carPoint.carPos[1][2] = eerot[5];
  
  carPoint.carPos[2][0] = eerot[6];
  carPoint.carPos[2][1] = eerot[7];
  carPoint.carPos[2][2] = eerot[8];

  carPoint.carPos[0][3] = eetrans[0];
  carPoint.carPos[1][3] = eetrans[1];
  carPoint.carPos[2][3] = eetrans[2];
}

int JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::CalcmaxCartesianLength(int cartesianLength[3])
{
  int maxCartesianLength;
  
  maxCartesianLength = cartesianLength[0];
  
  if(maxCartesianLength < cartesianLength[1]){
    maxCartesianLength = cartesianLength[1];
  }
  
  if(maxCartesianLength < cartesianLength[2]){
    maxCartesianLength = cartesianLength[2];
  }

  return maxCartesianLength;
}

int JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::Calcmovetime(double rotationLength[7])
{
  int movetime;
  double maxMoveJoint;
  double maxMoveJointNumber;
  
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

  return movetime;
}


// End of example implementational code




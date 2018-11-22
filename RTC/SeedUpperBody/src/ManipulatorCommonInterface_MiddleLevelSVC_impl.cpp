// -*-C++-*-
/*!
 * @file  ManipulatorCommonInterface_MiddleLevelSVC_impl.cpp
 * @brief Service implementation code of ManipulatorCommonInterface_MiddleLevel.idl
 *
 */

#include "ManipulatorCommonInterface_MiddleLevelSVC_impl.h"
#include "LibSeednoidUpperUnit1.h"
#include "right_ReturnID.h"

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
  
  RETURNID_OK;
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
  
  RETURNID_OK;
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
  RETURNID_OK;
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
    RETURNID_VALUE_ERR;
  }
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
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
    RETURNID_NG;

  std::cout << "Success" <<std::endl;
  RETURNID_OK;
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
  RETURNID_OK;
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
  
  RETURNID_OK;
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
    RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<speed.length();i++){
    m_maxSpeedJoint[i] = speed[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
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
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(JARA_ARM::ULONG spdRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM::ULONG spdRatio)
{
  std::cout<<"setSpeedCartesian"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioCartesian = spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
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
    RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<jointPoint.length();i++){
    m_homeJoint[i] = jointPoint[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
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
  RETURNID_OK;
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
  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM::ULONG forceRatio)
{
  std::cout<<"setGraspForce"<<std::endl;
  int ratio = (int)forceRatio;
  m_forceRatio = forceRatio;
  noid.setRightHandCurrent(ratio);
  std::cout<<"Success"<<std::endl<<std::endl;
  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM::ULONG& forceRatio)
{
  std::cout<<"getGraspForce"<<std::endl;
  forceRatio = m_forceRatio;
  std::cout<<"Success"<<std::endl<<std::endl;
  RETURNID_OK;
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
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
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
  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;

  return result;
}



// End of example implementational code




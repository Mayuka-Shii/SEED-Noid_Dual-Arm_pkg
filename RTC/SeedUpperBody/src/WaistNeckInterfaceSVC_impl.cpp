// -*-C++-*-
/*!
 * @file  WaistNeckInterfaceSVC_impl.cpp
 * @brief Service implementation code of WaistNeckInterface.idl
 *
 */

#include "WaistNeckInterfaceSVC_impl.h"
#include "LibSeednoidUpperUnit1.h"
#include "waistneck_ReturnID.h"

/*
 * Example implementational code for IDL interface WaistNeck::NeckInterface
 */
WaistNeck_NeckInterfaceSVC_impl::WaistNeck_NeckInterfaceSVC_impl()
{
  m_speedRatioNeck = 50;
  WaistNeck::ManipInfo_var mInfovar;
  WaistNeck::ManipInfo mInfo;
  getNeckInfo(mInfovar);
  mInfo = mInfovar;
  m_maxSpeedNeck.length(mInfo.axisNum);
  for(unsigned int i=0;i<mInfo.axisNum;i++){
  m_maxSpeedNeck[i] = 60;//deg/s
  }
}


WaistNeck_NeckInterfaceSVC_impl::~WaistNeck_NeckInterfaceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getFeedbackPosNeck(WaistNeck::NeckPos_out pos)
{
  std::cout<<"getFeedbackPosNeck"<<std::endl;
  
  Frame Neckpos;
  pos = new WaistNeck::NeckPos;
  pos->length(3);

  Neckpos = noid.readNeckJointAngle();
  //Neckpos = noid.notReadGetNowNeckJointAngle();

  (*pos)[0] = Neckpos.yaw;
  (*pos)[1] = Neckpos.pitch;
  (*pos)[2] = Neckpos.roll;
  
  //for (int i = 0; i<3; i++)
  //  {
  //    std::cout << "Neckpos[" << i << "] = " << (*pos)[i] << "[°]" << std::endl;
  //  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getNeckInfo(WaistNeck::ManipInfo_out mInfo)
{
  std::cout << "getNeckInfo" << std::endl;
  
  mInfo = new WaistNeck::ManipInfo;
  mInfo->manufactur = "Hardware：(株)THK  Software：Meijo University robot systems design laboratory";
  mInfo->type = "Seed_Neck";
  mInfo->cmdCycle = 20;
  mInfo->axisNum = 3;
  
  std::cout << " manufactur : " << mInfo->manufactur << std::endl;
  std::cout << " type       : " << mInfo->type << std::endl;
  std::cout << " axisNum    : " << mInfo->axisNum << std::endl;
  std::cout << " cmdCycle   : " << mInfo->cmdCycle << std::endl;

  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getSoftLimitNeck(WaistNeck::LimitSeq_out softLimit)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getState(WaistNeck::ULONG& state)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::servoOFF()
{
  int torque = 0;
  std::cout << "ServoOFF (SERVO_OFF)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::servoON()
{
  int torque = 1;
  std::cout << "ServoON (SERVO_ON)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::setSoftLimitNeck(const WaistNeck::LimitSeq& softLimit)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getMaxSpeedNeck(WaistNeck::DoubleSeq_out speed)
{
  std::cout<<"getMaxSpeedNeck"<<std::endl;
  speed = new WaistNeck::DoubleSeq;
  speed->length(3);
  for(unsigned int i=0;i<3;i++){
    speed[i] = m_maxSpeedNeck[i];
  }
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getMinAccelTimeNeck(::CORBA::Double& aclTime)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::movePTPNeckAbs(const WaistNeck::NeckPos& neckPoints)
{
  std::cout << "movePTPNeckAbs" << std::endl;
  Frame pos;
  int movetime;
  double rotationLength[3];
  double maxMoveJoint;
  double maxMoveJointNumber;

  std::cout << "neckyaw   = " << neckPoints[0] << std::endl;
  std::cout << "neckpitch = " << neckPoints[1] << std::endl;
  std::cout << "neckroll   = " << neckPoints[2] << std::endl;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  pos.yaw  = (double)neckPoints[0];
  pos.pitch = (double)neckPoints[1];
  pos.roll   = (double)neckPoints[2];

  //腰の現在角度取得
  WaistNeck::NeckPos_var nowNeckposvar;
  WaistNeck::NeckPos nowNeckpos;
  getFeedbackPosNeck(nowNeckposvar);
  nowNeckpos = nowNeckposvar;

  //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
  rotationLength[0] = fabs(pos.yaw - nowNeckpos[0]);
  rotationLength[1] = fabs(pos.pitch - nowNeckpos[1]);
  rotationLength[2] = fabs(pos.roll - nowNeckpos[2]);  
  maxMoveJoint = rotationLength[0];
  maxMoveJointNumber = 0;
  for(int j=0;j<3;j++)
    {
      if(maxMoveJoint<rotationLength[j]){
	maxMoveJoint = rotationLength[j];
	maxMoveJointNumber = j;
      }
    }

  movetime = ((maxMoveJoint/m_maxSpeedNeck[maxMoveJointNumber])*1000)*(100/(double)m_speedRatioNeck);  

  noid.setNeckJointAngle(pos);
  
  noid.SeedAction(movetime);
  
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::movePTPNeckRel(const WaistNeck::NeckPos& neckPoints)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::pause()
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::resume()
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::stop()
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::setAccelTimeNeck(::CORBA::Double aclTime)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::setMaxSpeedNeck(const WaistNeck::DoubleSeq& speed)
{
  std::cout<<"setMaxSpeedNeck"<<std::endl;

  WaistNeck::ManipInfo_var mInfovar;
  WaistNeck::ManipInfo mInfo;
  getNeckInfo(mInfovar);
  mInfo = mInfovar;

  m_maxSpeedNeckFlag = true;
  
  if(speed.length() != (mInfo.axisNum)){
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<speed.length();i++){
    m_maxSpeedNeck[i] = speed[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::setMinAccelTimeNeck(::CORBA::Double aclTime)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::setSpeedNeck(WaistNeck::ULONG spdRatio)
{
  std::cout<<"setSpeedNeck"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioNeck = spdRatio;
  }
  else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::getSpeedNeck(WaistNeck::ULONG& spdRatio)
{
  std::cout<<"getSpeedNeck"<<std::endl;
  spdRatio = m_speedRatioNeck;
  std::cout<<"Success"<<std::endl<<std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::movePTPNeckAbsCmdCycle(const WaistNeck::NeckPos& neckPoints)
{
  std::cout << "movePTPNeckAbs" << std::endl;
  Frame pos;
  int movetime;
  double rotationLength[3];
  double maxMoveJoint;
  double maxMoveJointNumber;

  std::cout << "neckyaw   = " << neckPoints[0] << std::endl;
  std::cout << "neckpitch = " << neckPoints[1] << std::endl;
  std::cout << "neckroll   = " << neckPoints[2] << std::endl;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  pos.yaw  = (double)neckPoints[0];
  pos.pitch = (double)neckPoints[1];
  pos.roll   = (double)neckPoints[2];

  WaistNeck::ManipInfo_var mInfovar;
  WaistNeck::ManipInfo mInfo;
  getNeckInfo(mInfovar);
  mInfo = mInfovar;

  movetime = 1000/mInfo.cmdCycle;  
  
  noid.setNeckJointAngle(pos);
  
  noid.SeedAction(movetime);
  
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_NeckInterfaceSVC_impl::movePTPNeckRelCmdCycle(const WaistNeck::NeckPos& neckPoints)
{
	WaistNeck::RETURN_ID* result;

  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface WaistNeck::WaistInterface
 */
WaistNeck_WaistInterfaceSVC_impl::WaistNeck_WaistInterfaceSVC_impl()
{
  m_speedRatioWaist = 50;
  WaistNeck::ManipInfo_var mInfovar;
  WaistNeck::ManipInfo mInfo;
  getWaistInfo(mInfovar);
  mInfo = mInfovar;
  m_maxSpeedWaist.length(mInfo.axisNum);
  for(unsigned int i=0;i<mInfo.axisNum;i++){
    m_maxSpeedWaist[i] = 30;//deg/s
  }
}


WaistNeck_WaistInterfaceSVC_impl::~WaistNeck_WaistInterfaceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getFeedbackPosWaist(WaistNeck::WaistPos_out pos)
{
  std::cout<<"getFeedbackPosWaist"<<std::endl;

  Frame Waistpos;
  pos = new WaistNeck::WaistPos;
  pos->length(3);
  Waistpos = noid.readWaistJointAngle();
  //Waistpos = noid.notReadGetNowWaistJointAngle();

  (*pos)[0] = Waistpos.yaw;
  (*pos)[1] = Waistpos.pitch;
  (*pos)[2] = Waistpos.roll;

  //for (int i = 0; i<3; i++)
  //  {
  //    std::cout << "Waistpos[" << i << "] = " << (*pos)[i] << "[°]" << std::endl;
  //  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getWaistInfo(WaistNeck::ManipInfo_out mInfo)
{
  std::cout << "getWaistInfo" << std::endl;
  
  mInfo = new WaistNeck::ManipInfo;
  mInfo->manufactur = "Hardware：(株)THK  Software：Meijo University robot systems design laboratory";
  mInfo->type = "Seed_Waist";
  mInfo->cmdCycle = 20;
  mInfo->axisNum = 3;
  
  std::cout << " manufactur : " << mInfo->manufactur << std::endl;
  std::cout << " type       : " << mInfo->type << std::endl;
  std::cout << " axisNum    : " << mInfo->axisNum << std::endl;
  std::cout << " cmdCycle   : " << mInfo->cmdCycle << std::endl;

  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;	
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getSoftLimitWaist(WaistNeck::LimitSeq_out softLimit)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getState(WaistNeck::ULONG& state)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::servoOFF()
{
  int torque = 0;
  std::cout << "ServoOFF (SERVO_OFF)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::servoON()
{
  int torque = 1;
  std::cout << "ServoON (SERVO_ON)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::setSoftLimitWaist(const WaistNeck::LimitSeq& softLimit)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getMaxSpeedWaist(WaistNeck::DoubleSeq_out speed)
{
  std::cout<<"getMaxSpeedWaist"<<std::endl;
  speed = new WaistNeck::DoubleSeq;
  speed->length(3);
  for(unsigned int i=0;i<3;i++){
    speed[i] = m_maxSpeedWaist[i];
  }
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getMinAccelTimeWaist(::CORBA::Double& aclTime)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::movePTPWaistAbs(const WaistNeck::WaistPos& waistPoints)
{
  std::cout << "movePTPWaistAbs" << std::endl;
  Frame pos;
  int movetime;
  double rotationLength[3];
  double maxMoveJoint;
  double maxMoveJointNumber;

  std::cout << "waistyaw   = " << waistPoints[0] << std::endl;
  std::cout << "waistpitch = " << waistPoints[1] << std::endl;
  std::cout << "waistroll  = " << waistPoints[2] << std::endl;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  pos.yaw  = (double)waistPoints[0];
  pos.pitch = (double)waistPoints[1];
  pos.roll   = (double)waistPoints[2];
  
  //腰の現在角度取得
  WaistNeck::WaistPos_var nowWaistposvar;
  WaistNeck::WaistPos nowWaistpos;
  getFeedbackPosWaist(nowWaistposvar);
  nowWaistpos = nowWaistposvar;

  //現在値と目標値で最も離れている関節がm_maxSpeedJointになるように計算
  rotationLength[0] = fabs(pos.yaw - nowWaistpos[0]);
  rotationLength[1] = fabs(pos.pitch - nowWaistpos[1]);
  rotationLength[2] = fabs(pos.roll - nowWaistpos[2]);  
  maxMoveJoint = rotationLength[0];
  maxMoveJointNumber = 0;
  for(int j=0;j<3;j++)
    {
      if(maxMoveJoint<rotationLength[j]){
	maxMoveJoint = rotationLength[j];
	maxMoveJointNumber = j;
      }
    }

  movetime = ((maxMoveJoint/m_maxSpeedWaist[maxMoveJointNumber])*1000)*(100/(double)m_speedRatioWaist);  
  
  noid.setWaistJointAngle(pos);
  noid.SeedAction(movetime);
  //noid.SeedAction(1000/50);
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::movePTPWaistRel(const WaistNeck::WaistPos& waistPoints)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::pause()
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::resume()
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::stop()
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::setAccelTimeWaist(::CORBA::Double aclTime)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::setMaxSpeedWaist(const WaistNeck::DoubleSeq& speed)
{
  std::cout<<"setMaxSpeedWaist"<<std::endl;

  WaistNeck::ManipInfo_var mInfovar;
  WaistNeck::ManipInfo mInfo;
  getWaistInfo(mInfovar);
  mInfo = mInfovar;

  m_maxSpeedWaistFlag = true;
  
  if(speed.length() != (mInfo.axisNum)){
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  for(unsigned int i=0;i<speed.length();i++){
    m_maxSpeedWaist[i] = speed[i];
  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::setMinAccelTimeWaist(::CORBA::Double aclTime)
{
	WaistNeck::RETURN_ID* result;

  return result;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::setSpeedWaist(WaistNeck::ULONG spdRatio)
{
  std::cout<<"setSpeedWaist"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    m_speedRatioWaist = spdRatio;
  }
  else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::getSpeedWaist(WaistNeck::ULONG& spdRatio)
{
  std::cout<<"getSpeedWaist"<<std::endl;
  spdRatio = m_speedRatioWaist;
  std::cout<<"Success"<<std::endl<<std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::movePTPWaistAbsCmdCycle(const WaistNeck::WaistPos& waistPoints)
{
  std::cout << "movePTPWaistAbsCmdCycle" << std::endl;
  Frame pos;
  int movetime;
  double rotationLength[3];
  double maxMoveJoint;
  double maxMoveJointNumber;

  WaistNeck::ManipInfo_var mInfovar;
  WaistNeck::ManipInfo mInfo;
  getWaistInfo(mInfovar);
  mInfo = mInfovar;
  
  std::cout << "waistyaw   = " << waistPoints[0] << std::endl;
  std::cout << "waistpitch = " << waistPoints[1] << std::endl;
  std::cout << "waistroll  = " << waistPoints[2] << std::endl;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  pos.yaw  = (double)waistPoints[0];
  pos.pitch = (double)waistPoints[1];
  pos.roll   = (double)waistPoints[2];
  
  movetime = 1000/mInfo.cmdCycle;  
  
  noid.setWaistJointAngle(pos);
  noid.SeedAction(movetime);
  //noid.SeedAction(1000/50);
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

WaistNeck::RETURN_ID* WaistNeck_WaistInterfaceSVC_impl::movePTPWaistRelCmdCycle(const WaistNeck::WaistPos& waistPoints)
{
	WaistNeck::RETURN_ID* result;

  return result;
}



// End of example implementational code




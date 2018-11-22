﻿// -*-C++-*-
/*!
 * @file  LeftManipulatorCommonInterface_CommonSVC_impl.cpp
 * @brief Service implementation code of LeftManipulatorCommonInterface_Common.idl
 *
 */

#include "LeftManipulatorCommonInterface_CommonSVC_impl.h"
#include "LibSeednoidUpperUnit1.h"
#include "left_ReturnID.h"

/*
 * Example implementational code for IDL interface JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common
 */
JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl()
{
  // Please add extra constructor code here.
}


JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::~JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::clearAlarms()
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::getActiveAlarm(JARA_ARM_LEFT::AlarmSeq_out alarms)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::getFeedbackPosJoint(JARA_ARM_LEFT::JointPos_out pos)
{
  std::cout << "getFeedbackPosJoint" << std::endl;
  
  double JointPos[7];
  pos = new JARA_ARM_LEFT::JointPos;
  pos->length(7);
  
  noid.readLeftJointAngle(JointPos);
  //noid.notReadGetNowLeftJointAngle(JointPos);
    
  for (int i = 0; i<7; i++)
    {
      (*pos)[i] = JointPos[i];
      //std::cout << "Leftpos[" << i << "] = " << (*pos)[i] << "[°]" << std::endl;
    }
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::getManipInfo(JARA_ARM_LEFT::ManipInfo_out mInfo)
{
  std::cout << "GetManipInfo" << std::endl;
  
  mInfo = new JARA_ARM_LEFT::ManipInfo;
  mInfo->manufactur = "Hardware：(株)THK  Software：Meijo University robot systems design laboratory";
  mInfo->type = "Seed_LeftArm";
  mInfo->cmdCycle = 20;
  mInfo->axisNum = 7;
  mInfo->isGripper = true;

  
  std::cout << " manufactur : " << mInfo->manufactur << std::endl;
  std::cout << " type       : " << mInfo->type << std::endl;
  std::cout << " axisNum    : " << mInfo->axisNum << std::endl;
  std::cout << " cmdCycle   : " << mInfo->cmdCycle << std::endl;
  std::cout << " isGripper  : " << mInfo->isGripper << std::endl;

  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::getSoftLimitJoint(JARA_ARM_LEFT::LimitSeq_out softLimit)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::getState(JARA_ARM_LEFT::ULONG& state)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::servoOFF()
{
  int torque = 0;
  std::cout << "ServoOFF (SERVO_OFF)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::servoON()
{
  int torque = 1;
  std::cout << "ServoON (SERVO_ON)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl::setSoftLimitJoint(const JARA_ARM_LEFT::LimitSeq& softLimit)
{
	JARA_ARM_LEFT::RETURN_ID* result;

  return result;
}



// End of example implementational code




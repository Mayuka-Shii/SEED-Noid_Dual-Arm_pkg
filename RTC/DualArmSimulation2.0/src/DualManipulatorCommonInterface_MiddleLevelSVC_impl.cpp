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
  // Please add extra constructor code here.
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
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::closeGripper()>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM::HgMatrix offset)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM::HgMatrix offset)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM::CarPosWithElbow& pos)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM::CarPosWithElbow& pos)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM::CartesianSpeed& speed)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM::CartesianSpeed& speed)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM::DoubleSeq_out speed)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM::DoubleSeq_out speed)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM::LimitValue& xLimit, JARA_ARM::LimitValue& yLimit, JARA_ARM::LimitValue& zLimit)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM::LimitValue& xLimit, JARA_ARM::LimitValue& yLimit, JARA_ARM::LimitValue& zLimit)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM::ULONG angleRatio)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM::ULONG angleRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM::JointPos& jointPoints)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM::JointPos& jointPoints)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::openGripper()
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::openGripper()>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::pause()
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::pause()>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::resume()
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::resume()>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::stop()
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::stop()>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM::HgMatrix offset)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM::HgMatrix offset)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM::HgMatrix offset)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM::HgMatrix offset)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM::CartesianSpeed& speed)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM::CartesianSpeed& speed)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM::DoubleSeq& speed)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM::DoubleSeq& speed)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM::LimitValue& xLimit, const JARA_ARM::LimitValue& yLimit, const JARA_ARM::LimitValue& zLimit)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM::LimitValue& xLimit, const JARA_ARM::LimitValue& yLimit, const JARA_ARM::LimitValue& zLimit)>"
#endif
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
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM::ULONG spdRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM::JointPos& jointPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM::JointPos& jointPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM::JointPos_out jointPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM::JointPos_out jointPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::goHome()
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::goHome()>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSpeedCartesian(JARA_ARM::ULONG& spdRatio)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSpeedCartesian(JARA_ARM::ULONG& spdRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSpeedJoint(JARA_ARM::ULONG& spdRatio)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getSpeedJoint(JARA_ARM::ULONG& spdRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM::ULONG forceRatio)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM::ULONG forceRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM::ULONG& forceRatio)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM::ULONG& forceRatio)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
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
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbsCmdCycle(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbsCmdCycle(const JARA_ARM::JointPos& jointPoints)>"
#endif
  return result;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM::JointPos& jointPoints)
{
	JARA_ARM::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM::JointPos& jointPoints)>"
#endif
  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle
 */
JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra constructor code here.
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
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::closeGripper()>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM_LEFT::HgMatrix offset)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM_LEFT::HgMatrix offset)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM_LEFT::CarPosWithElbow& pos)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM_LEFT::CarPosWithElbow& pos)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM_LEFT::CartesianSpeed& speed)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM_LEFT::CartesianSpeed& speed)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM_LEFT::DoubleSeq_out speed)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM_LEFT::DoubleSeq_out speed)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM_LEFT::LimitValue& xLimit, JARA_ARM_LEFT::LimitValue& yLimit, JARA_ARM_LEFT::LimitValue& zLimit)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM_LEFT::LimitValue& xLimit, JARA_ARM_LEFT::LimitValue& yLimit, JARA_ARM_LEFT::LimitValue& zLimit)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_LEFT::ULONG angleRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_LEFT::ULONG angleRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM_LEFT::JointPos& jointPoints)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM_LEFT::JointPos& jointPoints)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_LEFT::JointPos& jointPoints)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_LEFT::JointPos& jointPoints)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::openGripper()
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::openGripper()>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::pause()
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::pause()>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::resume()
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::resume()>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::stop()
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::stop()>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM_LEFT::HgMatrix offset)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM_LEFT::HgMatrix offset)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM_LEFT::HgMatrix offset)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM_LEFT::HgMatrix offset)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM_LEFT::CartesianSpeed& speed)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM_LEFT::CartesianSpeed& speed)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM_LEFT::DoubleSeq& speed)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM_LEFT::DoubleSeq& speed)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM_LEFT::LimitValue& xLimit, const JARA_ARM_LEFT::LimitValue& yLimit, const JARA_ARM_LEFT::LimitValue& zLimit)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM_LEFT::LimitValue& xLimit, const JARA_ARM_LEFT::LimitValue& yLimit, const JARA_ARM_LEFT::LimitValue& zLimit)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(JARA_ARM_LEFT::ULONG spdRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(JARA_ARM_LEFT::ULONG spdRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM_LEFT::ULONG spdRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM_LEFT::ULONG spdRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM_LEFT::JointPos& jointPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM_LEFT::JointPos& jointPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM_LEFT::JointPos_out jointPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM_LEFT::JointPos_out jointPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::goHome()
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::goHome()>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSpeedCartesian(JARA_ARM_LEFT::ULONG& spdRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSpeedCartesian(JARA_ARM_LEFT::ULONG& spdRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSpeedJoint(JARA_ARM_LEFT::ULONG& spdRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSpeedJoint(JARA_ARM_LEFT::ULONG& spdRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM_LEFT::ULONG forceRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setGraspForce(JARA_ARM_LEFT::ULONG forceRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM_LEFT::ULONG& forceRatio)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getGraspForce(JARA_ARM_LEFT::ULONG& forceRatio)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbsCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbsCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints)>"
#endif
  return result;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints)
{
	JARA_ARM_LEFT::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRelCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints)>"
#endif
  return result;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface JARA_ARM_DUAL::DualManipulatorCommonInterface_Middle
 */
JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra constructor code here.
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
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::closeGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)>"
#endif
  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::getRelativePosition(const JARA_ARM_DUAL::HgMatrix RelPos)
{
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::getRelativePosition(const JARA_ARM_DUAL::HgMatrix RelPos)>"
#endif
  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)
{
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)>"
#endif
  return result;
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

  std::cout << "movePTPCartesianAbs" << std::endl;

  carPointToTransRot(rArm, goal_r_eerot, goal_r_eetrans);
  carPointToTransRot(lArm, goal_l_eerot, goal_l_eetrans);

  r_elbow = rArm.elbow;
  l_elbow = lArm.elbow;

  std::cout << "read position" << std::endl;

  m_rtcPtr->ReadRightJointAngle(nowRightpos);
  m_rtcPtr->ReadLeftJointAngle(nowLeftpos);

  std::cout << "ik solve" << std::endl;

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
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)>"
#endif
  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)
{
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)
{
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm)>"
#endif
  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm)
{
  std::cout << "movePTPJointAbs" << std::endl;

  double rarm[7];
  double larm[7];

  for(int i=0; i<7; i++)
    {
      rarm[i] = (double)rArm[i];
      larm[i] = (double)lArm[i];
    }

  m_rtcPtr->WriteRightJointAngle(rarm);
  m_rtcPtr->WriteLeftJointAngle(larm);

  std::cout << "Success" << std::endl << std::endl;

  D_RETURNID_OK;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm)
{
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm)>"
#endif
  return result;
}

JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::openGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)
{
	JARA_ARM_DUAL::RETURN_ID* result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <JARA_ARM_DUAL::RETURN_ID* JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl::openGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm)>"
#endif
  return result;
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

  m_rtcPtr->ReadRightJointAngle(nowRightpos);
    
  carPointToTransRot(carPoint, eerot, eetrans);

  for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowRightpos[d];
    }

  elbow = carPoint.elbow;

  int ret = noid.Solve_RightArmIk(eerot,eetrans,elbow,nowJointPos,iksol);
  if(!ret)
    {
      m_rtcPtr->WriteRightJointAngle(iksol);
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
  
  m_rtcPtr->ReadLeftJointAngle(nowLeftpos);

  carPointToTransRot(carPoint, eerot, eetrans);

  for(int d=0;d<7;d++)
    {
      nowJointPos[d] = nowLeftpos[d];
    }

  elbow = carPoint.elbow;

  int ret = noid.Solve_LeftArmIk(eerot,eetrans,elbow,nowJointPos,iksol);
  if(!ret)
    {
      m_rtcPtr->WriteLeftJointAngle(iksol);
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


// End of example implementational code




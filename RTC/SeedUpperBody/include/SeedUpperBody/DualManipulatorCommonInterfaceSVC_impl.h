// -*-C++-*-
/*!
 * @file  DualManipulatorCommonInterfaceSVC_impl.h
 * @brief Service implementation header of DualManipulatorCommonInterface.idl
 *
 */

#include "DualManipulatorCommonInterfaceSkel.h"

#ifndef DUALMANIPULATORCOMMONINTERFACESVC_IMPL_H
#define DUALMANIPULATORCOMMONINTERFACESVC_IMPL_H
 
/*!
 * @class DualManipulatorCommonInterface_CommonCommandsSVC_impl
 * Example class implementing IDL interface DualManipulatorCommonInterface::CommonCommands
 */
class DualManipulatorCommonInterface_CommonCommandsSVC_impl
 : public virtual POA_DualManipulatorCommonInterface::CommonCommands,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~DualManipulatorCommonInterface_CommonCommandsSVC_impl();


 public:
  /*!
   * @brief standard constructor
   */
   DualManipulatorCommonInterface_CommonCommandsSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~DualManipulatorCommonInterface_CommonCommandsSVC_impl();

   // attributes and operations
   DualManipulatorCommonInterface::RETURN_ID* servoOFF();
   DualManipulatorCommonInterface::RETURN_ID* servoON();
   DualManipulatorCommonInterface::RETURN_ID* servoOFFArm();
   DualManipulatorCommonInterface::RETURN_ID* servoOFFHand();
   DualManipulatorCommonInterface::RETURN_ID* servoONArm();
   DualManipulatorCommonInterface::RETURN_ID* servoONHand();
   DualManipulatorCommonInterface::RETURN_ID returnid;

};

/*!
 * @class DualManipulatorCommonInterface_MotionCommandsSVC_impl
 * Example class implementing IDL interface DualManipulatorCommonInterface::MotionCommands
 */
class DualManipulatorCommonInterface_MotionCommandsSVC_impl
 : public virtual POA_DualManipulatorCommonInterface::MotionCommands,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~DualManipulatorCommonInterface_MotionCommandsSVC_impl();
  DualManipulatorCommonInterface::DoubleSeq m_maxSpeedJoint;
  DualManipulatorCommonInterface::ULONG m_speedRatioJoint;
  DualManipulatorCommonInterface::ULONG m_speedRatioCartesian;

 public:
  /*!
   * @brief standard constructor
   */
   DualManipulatorCommonInterface_MotionCommandsSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~DualManipulatorCommonInterface_MotionCommandsSVC_impl();

   // attributes and operations
   DualManipulatorCommonInterface::RETURN_ID* closeGripper();
   DualManipulatorCommonInterface::RETURN_ID* moveGripper(const DualManipulatorCommonInterface::DoubleSeq& r_angle, const DualManipulatorCommonInterface::DoubleSeq& l_angle);
   DualManipulatorCommonInterface::RETURN_ID* moveLinearCartesianAbs(const DualManipulatorCommonInterface::CarPosWithElbow& rArm, const DualManipulatorCommonInterface::CarPosWithElbow& lArm);
   DualManipulatorCommonInterface::RETURN_ID* moveLinearCartesianRel(const DualManipulatorCommonInterface::CarPosWithElbow& rArm, const DualManipulatorCommonInterface::CarPosWithElbow& lArm);
   DualManipulatorCommonInterface::RETURN_ID* movePTPJointAbs(const DualManipulatorCommonInterface::JointPos& jointPoints);
   DualManipulatorCommonInterface::RETURN_ID* movePTPJointRel(const DualManipulatorCommonInterface::JointPos& jointPoints);
   DualManipulatorCommonInterface::RETURN_ID* movePTPJointAbsSeq(const DualManipulatorCommonInterface::JointPosSeq& jointPointsSeq);
   DualManipulatorCommonInterface::RETURN_ID* openGripper();
   DualManipulatorCommonInterface::RETURN_ID* setSpeedCartesian(DualManipulatorCommonInterface::ULONG spdRatio);
   DualManipulatorCommonInterface::RETURN_ID* setSpeedJoint(DualManipulatorCommonInterface::ULONG spdRatio);
   DualManipulatorCommonInterface::RETURN_ID returnid;
   DualManipulatorCommonInterface::RETURN_ID* RightArmPTPCartesianAbs(const DualManipulatorCommonInterface::CarPosWithElbow& CarPoint);
   DualManipulatorCommonInterface::RETURN_ID* LeftArmPTPCartesianAbs(const DualManipulatorCommonInterface::CarPosWithElbow& CarPoint);
  
};



#endif // DUALMANIPULATORCOMMONINTERFACESVC_IMPL_H



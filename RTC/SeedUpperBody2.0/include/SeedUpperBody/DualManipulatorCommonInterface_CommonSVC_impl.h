// -*-C++-*-
/*!
 * @file  DualManipulatorCommonInterface_CommonSVC_impl.h
 * @brief Service implementation header of DualManipulatorCommonInterface_Common.idl
 *
 */

#include "ManipulatorCommonInterface_CommonSkel.h"
#include "ManipulatorCommonInterface_DataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "LeftManipulatorCommonInterface_CommonSkel.h"
#include "LeftManipulatorCommonInterface_DataTypesSkel.h"
#include "DualManipulatorCommonInterface_DataTypesSkel.h"

#include "DualManipulatorCommonInterface_CommonSkel.h"

#ifndef DUALMANIPULATORCOMMONINTERFACE_COMMONSVC_IMPL_H
#define DUALMANIPULATORCOMMONINTERFACE_COMMONSVC_IMPL_H
 
/*!
 * @class JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl
 * Example class implementing IDL interface JARA_ARM::ManipulatorCommonInterface_Common
 */
class JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl
 : public virtual POA_JARA_ARM::ManipulatorCommonInterface_Common,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl();

   // attributes and operations
   JARA_ARM::RETURN_ID* clearAlarms();
   JARA_ARM::RETURN_ID* getActiveAlarm(JARA_ARM::AlarmSeq_out alarms);
   JARA_ARM::RETURN_ID* getFeedbackPosJoint(JARA_ARM::JointPos_out pos);
   JARA_ARM::RETURN_ID* getManipInfo(JARA_ARM::ManipInfo_out mInfo);
   JARA_ARM::RETURN_ID* getSoftLimitJoint(JARA_ARM::LimitSeq_out softLimit);
   JARA_ARM::RETURN_ID* getState(JARA_ARM::ULONG& state);
   JARA_ARM::RETURN_ID* servoOFF();
   JARA_ARM::RETURN_ID* servoON();
   JARA_ARM::RETURN_ID* setSoftLimitJoint(const JARA_ARM::LimitSeq& softLimit);

};

/*!
 * @class JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl
 * Example class implementing IDL interface JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common
 */
class JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl
 : public virtual POA_JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl();

   // attributes and operations
   JARA_ARM_LEFT::RETURN_ID* clearAlarms();
   JARA_ARM_LEFT::RETURN_ID* getActiveAlarm(JARA_ARM_LEFT::AlarmSeq_out alarms);
   JARA_ARM_LEFT::RETURN_ID* getFeedbackPosJoint(JARA_ARM_LEFT::JointPos_out pos);
   JARA_ARM_LEFT::RETURN_ID* getManipInfo(JARA_ARM_LEFT::ManipInfo_out mInfo);
   JARA_ARM_LEFT::RETURN_ID* getSoftLimitJoint(JARA_ARM_LEFT::LimitSeq_out softLimit);
   JARA_ARM_LEFT::RETURN_ID* getState(JARA_ARM_LEFT::ULONG& state);
   JARA_ARM_LEFT::RETURN_ID* servoOFF();
   JARA_ARM_LEFT::RETURN_ID* servoON();
   JARA_ARM_LEFT::RETURN_ID* setSoftLimitJoint(const JARA_ARM_LEFT::LimitSeq& softLimit);

};

/*!
 * @class JARA_ARM_DUAL_DualManipulatorCommonInterface_CommonSVC_impl
 * Example class implementing IDL interface JARA_ARM_DUAL::DualManipulatorCommonInterface_Common
 */
class JARA_ARM_DUAL_DualManipulatorCommonInterface_CommonSVC_impl
 : public virtual POA_JARA_ARM_DUAL::DualManipulatorCommonInterface_Common,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JARA_ARM_DUAL_DualManipulatorCommonInterface_CommonSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   JARA_ARM_DUAL_DualManipulatorCommonInterface_CommonSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~JARA_ARM_DUAL_DualManipulatorCommonInterface_CommonSVC_impl();

   // attributes and operations
   JARA_ARM_DUAL::RETURN_ID* servoOFFArm();
   JARA_ARM_DUAL::RETURN_ID* servoOFFHand();
   JARA_ARM_DUAL::RETURN_ID* servoONArm();
   JARA_ARM_DUAL::RETURN_ID* servoONHand();

};



#endif // DUALMANIPULATORCOMMONINTERFACE_COMMONSVC_IMPL_H



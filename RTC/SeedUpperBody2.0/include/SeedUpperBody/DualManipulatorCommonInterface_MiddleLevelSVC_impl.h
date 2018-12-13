// -*-C++-*-
/*!
 * @file  DualManipulatorCommonInterface_MiddleLevelSVC_impl.h
 * @brief Service implementation header of DualManipulatorCommonInterface_MiddleLevel.idl
 *
 */

#include "ManipulatorCommonInterface_MiddleLevelSkel.h"
#include "ManipulatorCommonInterface_DataTypesSkel.h"
#include "BasicDataTypeSkel.h"
#include "LeftManipulatorCommonInterface_MiddleLevelSkel.h"
#include "LeftManipulatorCommonInterface_DataTypesSkel.h"
#include "DualManipulatorCommonInterface_DataTypesSkel.h"

#include "DualManipulatorCommonInterface_MiddleLevelSkel.h"

#ifndef DUALMANIPULATORCOMMONINTERFACE_MIDDLELEVELSVC_IMPL_H
#define DUALMANIPULATORCOMMONINTERFACE_MIDDLELEVELSVC_IMPL_H

#include "DualManipulatorCommonInterface_CommonSVC_impl.h"

/*!
 * @class JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl
 * Example class implementing IDL interface JARA_ARM::ManipulatorCommonInterface_Middle
 */
class JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl
 : public virtual POA_JARA_ARM::ManipulatorCommonInterface_Middle,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl();
  JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl m_ManipulatorCommonInterface_Common;

  double m_nowPos[7];
  bool m_maxSpeedJointFlag,m_maxSpeedCartesianFlag,m_softLimitCartesianFlag;
  JARA_ARM::CartesianSpeed m_maxSpeedCartesian;
  JARA_ARM::DoubleSeq m_maxSpeedJoint;
  JARA_ARM::LimitValue m_xLimit, m_yLimit, m_zLimit;
  JARA_ARM::ULONG m_speedRatioCartesian,m_speedRatioJoint,m_forceRatio;
  JARA_ARM::RETURN_ID_var m_rid_right;
  JARA_ARM::JointPos m_homeJoint;

 public:
  /*!
   * @brief standard constructor
   */
   JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~JARA_ARM_ManipulatorCommonInterface_MiddleSVC_impl();

   // attributes and operations
   JARA_ARM::RETURN_ID* closeGripper();
   JARA_ARM::RETURN_ID* getBaseOffset(JARA_ARM::HgMatrix offset);
   JARA_ARM::RETURN_ID* getFeedbackPosCartesian(JARA_ARM::CarPosWithElbow& pos);
   JARA_ARM::RETURN_ID* getMaxSpeedCartesian(JARA_ARM::CartesianSpeed& speed);
   JARA_ARM::RETURN_ID* getMaxSpeedJoint(JARA_ARM::DoubleSeq_out speed);
   JARA_ARM::RETURN_ID* getMinAccelTimeCartesian(::CORBA::Double& aclTime);
   JARA_ARM::RETURN_ID* getMinAccelTimeJoint(::CORBA::Double& aclTime);
   JARA_ARM::RETURN_ID* getSoftLimitCartesian(JARA_ARM::LimitValue& xLimit, JARA_ARM::LimitValue& yLimit, JARA_ARM::LimitValue& zLimit);
   JARA_ARM::RETURN_ID* moveGripper(JARA_ARM::ULONG angleRatio);
   JARA_ARM::RETURN_ID* moveLinearCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* moveLinearCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* movePTPCartesianAbs(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* movePTPCartesianRel(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* movePTPJointAbs(const JARA_ARM::JointPos& jointPoints);
   JARA_ARM::RETURN_ID* movePTPJointRel(const JARA_ARM::JointPos& jointPoints);
   JARA_ARM::RETURN_ID* openGripper();
   JARA_ARM::RETURN_ID* pause();
   JARA_ARM::RETURN_ID* resume();
   JARA_ARM::RETURN_ID* stop();
   JARA_ARM::RETURN_ID* setAccelTimeCartesian(::CORBA::Double aclTime);
   JARA_ARM::RETURN_ID* setAccelTimeJoint(::CORBA::Double aclTime);
   JARA_ARM::RETURN_ID* setBaseOffset(const JARA_ARM::HgMatrix offset);
   JARA_ARM::RETURN_ID* setControlPointOffset(const JARA_ARM::HgMatrix offset);
   JARA_ARM::RETURN_ID* setMaxSpeedCartesian(const JARA_ARM::CartesianSpeed& speed);
   JARA_ARM::RETURN_ID* setMaxSpeedJoint(const JARA_ARM::DoubleSeq& speed);
   JARA_ARM::RETURN_ID* setMinAccelTimeCartesian(::CORBA::Double aclTime);
   JARA_ARM::RETURN_ID* setMinAccelTimeJoint(::CORBA::Double aclTime);
   JARA_ARM::RETURN_ID* setSoftLimitCartesian(const JARA_ARM::LimitValue& xLimit, const JARA_ARM::LimitValue& yLimit, const JARA_ARM::LimitValue& zLimit);
   JARA_ARM::RETURN_ID* setSpeedCartesian(JARA_ARM::ULONG spdRatio);
   JARA_ARM::RETURN_ID* setSpeedJoint(JARA_ARM::ULONG spdRatio);
   JARA_ARM::RETURN_ID* moveCircularCartesianAbs(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT);
   JARA_ARM::RETURN_ID* moveCircularCartesianRel(const JARA_ARM::CarPosWithElbow& carPointR, const JARA_ARM::CarPosWithElbow& carPointT);
   JARA_ARM::RETURN_ID* setHome(const JARA_ARM::JointPos& jointPoint);
   JARA_ARM::RETURN_ID* getHome(JARA_ARM::JointPos_out jointPoint);
   JARA_ARM::RETURN_ID* goHome();
   JARA_ARM::RETURN_ID* getSpeedCartesian(JARA_ARM::ULONG& spdRatio);
   JARA_ARM::RETURN_ID* getSpeedJoint(JARA_ARM::ULONG& spdRatio);
   JARA_ARM::RETURN_ID* setGraspForce(JARA_ARM::ULONG forceRatio);
   JARA_ARM::RETURN_ID* getGraspForce(JARA_ARM::ULONG& forceRatio);
   JARA_ARM::RETURN_ID* moveLinearCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* moveLinearCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* movePTPCartesianAbsCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* movePTPCartesianRelCmdCycle(const JARA_ARM::CarPosWithElbow& carPoint);
   JARA_ARM::RETURN_ID* movePTPJointAbsCmdCycle(const JARA_ARM::JointPos& jointPoints);
   JARA_ARM::RETURN_ID* movePTPJointRelCmdCycle(const JARA_ARM::JointPos& jointPoints);

};

/*!
 * @class JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl
 * Example class implementing IDL interface JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle
 */
class JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl
 : public virtual POA_JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl();
  JARA_ARM_LEFT_LeftManipulatorCommonInterface_CommonSVC_impl m_LeftManipulatorCommonInterface_Common;

  double m_nowPos[7];
  bool m_maxSpeedJointFlag,m_maxSpeedCartesianFlag,m_softLimitCartesianFlag;
  JARA_ARM_LEFT::CartesianSpeed m_maxSpeedCartesian;
  JARA_ARM_LEFT::DoubleSeq m_maxSpeedJoint;
  JARA_ARM_LEFT::LimitValue m_xLimit, m_yLimit, m_zLimit;
  JARA_ARM_LEFT::ULONG m_speedRatioCartesian,m_speedRatioJoint,m_forceRatio;
  JARA_ARM_LEFT::RETURN_ID_var m_rid_left;
  JARA_ARM_LEFT::JointPos m_homeJoint;
  
 public:
  /*!
   * @brief standard constructor
   */
   JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl();

   // attributes and operations
   JARA_ARM_LEFT::RETURN_ID* closeGripper();
   JARA_ARM_LEFT::RETURN_ID* getBaseOffset(JARA_ARM_LEFT::HgMatrix offset);
   JARA_ARM_LEFT::RETURN_ID* getFeedbackPosCartesian(JARA_ARM_LEFT::CarPosWithElbow& pos);
   JARA_ARM_LEFT::RETURN_ID* getMaxSpeedCartesian(JARA_ARM_LEFT::CartesianSpeed& speed);
   JARA_ARM_LEFT::RETURN_ID* getMaxSpeedJoint(JARA_ARM_LEFT::DoubleSeq_out speed);
   JARA_ARM_LEFT::RETURN_ID* getMinAccelTimeCartesian(::CORBA::Double& aclTime);
   JARA_ARM_LEFT::RETURN_ID* getMinAccelTimeJoint(::CORBA::Double& aclTime);
   JARA_ARM_LEFT::RETURN_ID* getSoftLimitCartesian(JARA_ARM_LEFT::LimitValue& xLimit, JARA_ARM_LEFT::LimitValue& yLimit, JARA_ARM_LEFT::LimitValue& zLimit);
   JARA_ARM_LEFT::RETURN_ID* moveGripper(JARA_ARM_LEFT::ULONG angleRatio);
   JARA_ARM_LEFT::RETURN_ID* moveLinearCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* moveLinearCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* movePTPCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* movePTPCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* movePTPJointAbs(const JARA_ARM_LEFT::JointPos& jointPoints);
   JARA_ARM_LEFT::RETURN_ID* movePTPJointRel(const JARA_ARM_LEFT::JointPos& jointPoints);
   JARA_ARM_LEFT::RETURN_ID* openGripper();
   JARA_ARM_LEFT::RETURN_ID* pause();
   JARA_ARM_LEFT::RETURN_ID* resume();
   JARA_ARM_LEFT::RETURN_ID* stop();
   JARA_ARM_LEFT::RETURN_ID* setAccelTimeCartesian(::CORBA::Double aclTime);
   JARA_ARM_LEFT::RETURN_ID* setAccelTimeJoint(::CORBA::Double aclTime);
   JARA_ARM_LEFT::RETURN_ID* setBaseOffset(const JARA_ARM_LEFT::HgMatrix offset);
   JARA_ARM_LEFT::RETURN_ID* setControlPointOffset(const JARA_ARM_LEFT::HgMatrix offset);
   JARA_ARM_LEFT::RETURN_ID* setMaxSpeedCartesian(const JARA_ARM_LEFT::CartesianSpeed& speed);
   JARA_ARM_LEFT::RETURN_ID* setMaxSpeedJoint(const JARA_ARM_LEFT::DoubleSeq& speed);
   JARA_ARM_LEFT::RETURN_ID* setMinAccelTimeCartesian(::CORBA::Double aclTime);
   JARA_ARM_LEFT::RETURN_ID* setMinAccelTimeJoint(::CORBA::Double aclTime);
   JARA_ARM_LEFT::RETURN_ID* setSoftLimitCartesian(const JARA_ARM_LEFT::LimitValue& xLimit, const JARA_ARM_LEFT::LimitValue& yLimit, const JARA_ARM_LEFT::LimitValue& zLimit);
   JARA_ARM_LEFT::RETURN_ID* setSpeedCartesian(JARA_ARM_LEFT::ULONG spdRatio);
   JARA_ARM_LEFT::RETURN_ID* setSpeedJoint(JARA_ARM_LEFT::ULONG spdRatio);
   JARA_ARM_LEFT::RETURN_ID* moveCircularCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT);
   JARA_ARM_LEFT::RETURN_ID* moveCircularCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT);
   JARA_ARM_LEFT::RETURN_ID* setHome(const JARA_ARM_LEFT::JointPos& jointPoint);
   JARA_ARM_LEFT::RETURN_ID* getHome(JARA_ARM_LEFT::JointPos_out jointPoint);
   JARA_ARM_LEFT::RETURN_ID* goHome();
   JARA_ARM_LEFT::RETURN_ID* getSpeedCartesian(JARA_ARM_LEFT::ULONG& spdRatio);
   JARA_ARM_LEFT::RETURN_ID* getSpeedJoint(JARA_ARM_LEFT::ULONG& spdRatio);
   JARA_ARM_LEFT::RETURN_ID* setGraspForce(JARA_ARM_LEFT::ULONG forceRatio);
   JARA_ARM_LEFT::RETURN_ID* getGraspForce(JARA_ARM_LEFT::ULONG& forceRatio);
   JARA_ARM_LEFT::RETURN_ID* moveLinearCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* moveLinearCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* movePTPCartesianAbsCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* movePTPCartesianRelCmdCycle(const JARA_ARM_LEFT::CarPosWithElbow& carPoint);
   JARA_ARM_LEFT::RETURN_ID* movePTPJointAbsCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints);
   JARA_ARM_LEFT::RETURN_ID* movePTPJointRelCmdCycle(const JARA_ARM_LEFT::JointPos& jointPoints);

};

/*!
 * @class JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl
 * Example class implementing IDL interface JARA_ARM_DUAL::DualManipulatorCommonInterface_Middle
 */
class JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl
 : public virtual POA_JARA_ARM_DUAL::DualManipulatorCommonInterface_Middle,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl();

  JARA_ARM_DUAL::DoubleSeq m_maxSpeedJoint;
  JARA_ARM_DUAL::ULONG m_speedRatioJoint;
  JARA_ARM_DUAL::ULONG m_speedRatioCartesian;

 public:
  /*!
   * @brief standard constructor
   */
   JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~JARA_ARM_DUAL_DualManipulatorCommonInterface_MiddleSVC_impl();

   // attributes and operations
   JARA_ARM_DUAL::RETURN_ID* closeGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm);
   JARA_ARM_DUAL::RETURN_ID* getRelativePosition(const JARA_ARM_DUAL::HgMatrix RelPos);
   JARA_ARM_DUAL::RETURN_ID* moveGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm);
   JARA_ARM_DUAL::RETURN_ID* moveLinearCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm);
   JARA_ARM_DUAL::RETURN_ID* moveLinearCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm);
   JARA_ARM_DUAL::RETURN_ID* movePTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm);
   JARA_ARM_DUAL::RETURN_ID* movePTPCartesianRel(const JARA_ARM_DUAL::CarPosWithElbow& rArm, const JARA_ARM_DUAL::CarPosWithElbow& lArm);
   JARA_ARM_DUAL::RETURN_ID* movePTPJointAbs(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm);
   JARA_ARM_DUAL::RETURN_ID* movePTPJointRel(const JARA_ARM_DUAL::JointPos& rArm, const JARA_ARM_DUAL::JointPos& lArm);
   JARA_ARM_DUAL::RETURN_ID* openGripper(JARA_ARM_DUAL::ULONG rArm, JARA_ARM_DUAL::ULONG lArm);
   JARA_ARM_DUAL::RETURN_ID* RightArmPTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& CarPoint);
   JARA_ARM_DUAL::RETURN_ID* LeftArmPTPCartesianAbs(const JARA_ARM_DUAL::CarPosWithElbow& CarPoint);
  void carPointToTransRot(const JARA_ARM_DUAL::CarPosWithElbow& carPoint, double eerot[9], double eetrans[3]);
  void TransRotTocarPoint(JARA_ARM_DUAL::CarPosWithElbow& carPoint, double eerot[9], double eetrans[3]);
  int CalcmaxCartesianLength(int cartesianLength[3]);
  int Calcmovetime(double rotationLength[7]);
};



#endif // DUALMANIPULATORCOMMONINTERFACE_MIDDLELEVELSVC_IMPL_H



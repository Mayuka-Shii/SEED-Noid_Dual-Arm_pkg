// -*- C++ -*-
/*!
 * @file  DualArmSimulation.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "DualArmSimulation.h"
#include "LibSeednoidUpperUnit1.h"

// Module specification
// <rtc-template block="module_spec">
static const char* dualarmsimulation_spec[] =
  {
    "implementation_id", "DualArmSimulation",
    "type_name",         "DualArmSimulation",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
DualArmSimulation::DualArmSimulation(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_r_JointAngle_inIn("r_JointAngle_in", m_r_JointAngle_in),
    m_l_JointAngle_inIn("l_JointAngle_in", m_l_JointAngle_in),
    m_r_JointAngle_outOut("r_JointAngle_out", m_r_JointAngle_out),
    m_l_JointAngle_outOut("l_JointAngle_out", m_l_JointAngle_out),
    m_DualManipulatorCommonInterface_CommonPort("DualManipulatorCommonInterface_Common"),
    m_DualManipulatorCommonInterface_MiddlePort("DualManipulatorCommonInterface_Middle")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
DualArmSimulation::~DualArmSimulation()
{
}



RTC::ReturnCode_t DualArmSimulation::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("r_JointAngle_in", m_r_JointAngle_inIn);
  addInPort("l_JointAngle_in", m_l_JointAngle_inIn);
  
  // Set OutPort buffer
  addOutPort("r_JointAngle_out", m_r_JointAngle_outOut);
  addOutPort("l_JointAngle_out", m_l_JointAngle_outOut);
  
  // Set service provider to Ports
  m_DualManipulatorCommonInterface_CommonPort.registerProvider("DualManipulatorCommonInterface_Common", "JARA_ARM_DUAL::DualManipulatorCommonInterface_Common", m_DualManipulatorCommonInterface_Common);
  m_DualManipulatorCommonInterface_MiddlePort.registerProvider("DualManipulatorCommonInterface_Middle", "JARA_ARM_DUAL::DualManipulatorCommonInterface_Middle", m_DualManipulatorCommonInterface_Middle);

  m_DualManipulatorCommonInterface_CommonPort.registerProvider("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_DualManipulatorCommonInterface_MiddlePort.registerProvider("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);
  m_DualManipulatorCommonInterface_CommonPort.registerProvider("LeftManipulatorCommonInterface_Common", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common", m_LeftManipulatorCommonInterface_Common);
  m_DualManipulatorCommonInterface_MiddlePort.registerProvider("LeftManipulatorCommonInterface_Middle", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle", m_LeftManipulatorCommonInterface_Middle);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_DualManipulatorCommonInterface_CommonPort);
  addPort(m_DualManipulatorCommonInterface_MiddlePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DualArmSimulation::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmSimulation::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmSimulation::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t DualArmSimulation::onActivated(RTC::UniqueId ec_id)
{
  m_DualManipulatorCommonInterface_Common.setCompPtr(this);
  m_DualManipulatorCommonInterface_Middle.setCompPtr(this);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DualArmSimulation::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DualArmSimulation::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DualArmSimulation::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmSimulation::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmSimulation::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmSimulation::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmSimulation::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void DualArmSimulationInit(RTC::Manager* manager)
  {
    coil::Properties profile(dualarmsimulation_spec);
    manager->registerFactory(profile,
                             RTC::Create<DualArmSimulation>,
                             RTC::Delete<DualArmSimulation>);
  }
  
};



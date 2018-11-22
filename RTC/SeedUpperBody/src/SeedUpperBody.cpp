﻿// -*- C++ -*-
/*!
 * @file  SeedUpperBody.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "SeedUpperBody.h"
#include "LibSeednoidUpperUnit1.h"

// Module specification
// <rtc-template block="module_spec">
static const char* seedupperbody_spec[] =
  {
    "implementation_id", "SeedUpperBody",
    "type_name",         "SeedUpperBody",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.port_name", "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FT98HKZC-if00-port0",

    // Widget
    "conf.__widget__.port_name", "text",
    // Constraints

    "conf.__type__.port_name", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SeedUpperBody::SeedUpperBody(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_RightManipulatorCommonInterface_CommonPort("RightManipulatorCommonInterface_Common"),
    m_RightManipulatorCommonInterface_MiddlePort("RightManipulatorCommonInterface_Middle"),
    m_LeftManipulatorCommonInterface_CommonPort("LeftManipulatorCommonInterface_Common"),
    m_LeftManipulatorCommonInterface_MiddlePort("LeftManipulatorCommonInterface_Middle"),
    m_DualManipulatorCommonInterface_CommonCommandsPort("DualManipulatorCommonInterface_CommonCommands"),
    m_DualManipulatorCommonInterface_MotionCommandsPort("DualManipulatorCommonInterface_MotionCommands"),
    m_WaistInterfacePort("WaistInterface"),
    m_NeckInterfacePort("NeckInterface")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SeedUpperBody::~SeedUpperBody()
{
}



RTC::ReturnCode_t SeedUpperBody::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_RightManipulatorCommonInterface_CommonPort.registerProvider("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_RightManipulatorCommonInterface_MiddlePort.registerProvider("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);
  m_LeftManipulatorCommonInterface_CommonPort.registerProvider("LeftManipulatorCommonInterface_Common", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common", m_LeftManipulatorCommonInterface_Common);
  m_LeftManipulatorCommonInterface_MiddlePort.registerProvider("LeftManipulatorCommonInterface_Middle", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle", m_LeftManipulatorCommonInterface_Middle);
  m_DualManipulatorCommonInterface_CommonCommandsPort.registerProvider("DualManipulatorCommonInterface_CommonCommands", "DualManipulatorCommonInterface::CommonCommands", m_DualManipulatorCommonInterface_CommonCommands);
  m_DualManipulatorCommonInterface_MotionCommandsPort.registerProvider("DualManipulatorCommonInterface_MotionCommands", "DualManipulatorCommonInterface::MotionCommands", m_DualManipulatorCommonInterface_MotionCommands);
  m_WaistInterfacePort.registerProvider("WaistInterface", "WaistNeck::WaistInterface", m_WaistInterface);
  m_NeckInterfacePort.registerProvider("NeckInterface", "WaistNeck::NeckInterface", m_NeckInterface);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_RightManipulatorCommonInterface_CommonPort);
  addPort(m_RightManipulatorCommonInterface_MiddlePort);
  addPort(m_LeftManipulatorCommonInterface_CommonPort);
  addPort(m_LeftManipulatorCommonInterface_MiddlePort);
  addPort(m_DualManipulatorCommonInterface_CommonCommandsPort);
  addPort(m_DualManipulatorCommonInterface_MotionCommandsPort);
  addPort(m_WaistInterfacePort);
  addPort(m_NeckInterfacePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("port_name", m_port_name, "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FT98HKZC-if00-port0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedUpperBody::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBody::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBody::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SeedUpperBody::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "Act" << std::endl;
  if (noid.OpenSerialPort(m_port_name.c_str())){
    std::cout << "Connect Error!" << std::endl;
  }
  noid.initPosition();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedUpperBody::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "DeAct" << std::endl;
  noid.CloseSerialPort();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedUpperBody::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedUpperBody::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBody::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBody::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBody::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBody::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SeedUpperBodyInit(RTC::Manager* manager)
  {
    coil::Properties profile(seedupperbody_spec);
    manager->registerFactory(profile,
                             RTC::Create<SeedUpperBody>,
                             RTC::Delete<SeedUpperBody>);
  }
  
};



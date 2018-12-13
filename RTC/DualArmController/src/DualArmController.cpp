// -*- C++ -*-
/*!
 * @file  DualArmController.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "DualArmController.h"
#include <math.h>

// Module specification
// <rtc-template block="module_spec">
static const char* dualarmcontroller_spec[] =
  {
    "implementation_id", "DualArmController",
    "type_name",         "DualArmController",
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
DualArmController::DualArmController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_DualManipulatorCommonInterface_CommonPort("DualManipulatorCommonInterface_Common"),
    m_DualManipulatorCommonInterface_MiddlePort("DualManipulatorCommonInterface_Middle")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
DualArmController::~DualArmController()
{
}



RTC::ReturnCode_t DualArmController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_DualManipulatorCommonInterface_CommonPort.registerConsumer("DualManipulatorCommonInterface_Common", "JARA_ARM_DUAL::DualManipulatorCommonInterface_Common", m_DualManipulatorCommonInterface_Common);
  m_DualManipulatorCommonInterface_MiddlePort.registerConsumer("DualManipulatorCommonInterface_Middle", "JARA_ARM_DUAL::DualManipulatorCommonInterface_Middle", m_DualManipulatorCommonInterface_Middle);
  
  m_DualManipulatorCommonInterface_CommonPort.registerConsumer("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  
  m_DualManipulatorCommonInterface_MiddlePort.registerConsumer("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);

  m_DualManipulatorCommonInterface_CommonPort.registerConsumer("LeftManipulatorCommonInterface_Common", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common", m_LeftManipulatorCommonInterface_Common);
  m_DualManipulatorCommonInterface_MiddlePort.registerConsumer("LeftManipulatorCommonInterface_Middle", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle", m_LeftManipulatorCommonInterface_Middle);
  
  // Set CORBA Service Ports
  addPort(m_DualManipulatorCommonInterface_CommonPort);
  addPort(m_DualManipulatorCommonInterface_MiddlePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DualArmController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t DualArmController::onActivated(RTC::UniqueId ec_id)
{

  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DualArmController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DualArmController::onExecute(RTC::UniqueId ec_id)
{
  int modesel;
  int childsel;
  
  std::cout << "------------------------------------" << std::endl;
  std::cout << "Please Select Mode :)" << std::endl;
  std::cout << "------------------------------------" << std::endl;
  std::cout << "1 : movePTPJointAbs" << std::endl;
  std::cout << "2 : moveLinearCartesianAbs" << std::endl;
  std::cout << "3 : Gripper Open/Close" << std::endl;
  std::cout << "------------------------------------" << std::endl;

  std::cout << ">>";
  std::cin >> modesel;
  std::cout << std::endl;

  
  if(modesel == 1)
  {
    JARA_ARM_DUAL::JointPos rarm;
    JARA_ARM_DUAL::JointPos larm;
    rarm.length(7);
    larm.length(7);
    
    std::cout << "Select : movePTPJointAbs"  << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Enter Each Joint Angle :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Right Arm" << std::endl;
    for(int i=0; i<7; i++)
    {
      std::cout << "Joint " << i+1 << std::endl;
      std::cin >> rarm[i];
    }
    std::cout << "------------------------------------" << std::endl;
    std::cout << "left Arm" << std::endl;
    for(int i=0; i<7; i++)
    {
      std::cout << "Joint " << i+1 << std::endl;
      std::cin >> larm[i];
    }

    d_returnid = m_DualManipulatorCommonInterface_Middle->movePTPJointAbs(rarm,larm);
    if(d_returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
   
  }

  if(modesel == 2)
  {
    double r_x;
    double r_y;
    double r_z;
    double r_roll;
    double r_pitch;
    double r_yaw;
    double r_eerot[9];
    
    double l_x;
    double l_y;
    double l_z;
    double l_roll;
    double l_pitch;
    double l_yaw;
    double l_eerot[9];
    
    JARA_ARM_DUAL::CarPosWithElbow r_pos;
    JARA_ARM_DUAL::CarPosWithElbow l_pos;
    
    std::cout << "Select : moveLinearCartesianAbs"  << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Enter Each Joint Angle :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Right Arm" << std::endl;
    std::cout<< "roll : ";
    std::cin >> r_roll;
    std::cout << "pitch : ";
    std::cin >> r_pitch;
    std::cout << "yaw : ";
    std::cin >> r_yaw;
    std::cout << "x : "; 
    std::cin >> r_x;
    std::cout << "y : ";
    std::cin >> r_y;
    std::cout << "z : ";
    std::cin >> r_z;
    std::cout << std::endl;

    std::cout << "------------------------------------" << std::endl;
    std::cout << "left Arm" << std::endl;
    std::cout<< "roll : ";
    std::cin >> l_roll;
    std::cout << "pitch : ";
    std::cin >> l_pitch;
    std::cout << "yaw : ";
    std::cin >> l_yaw;
    std::cout << "x : "; 
    std::cin >> l_x;
    std::cout << "y : ";
    std::cin >> l_y;
    std::cout << "z : ";
    std::cin >> l_z;
    std::cout << std::endl; 
	  
    //姿勢の行列の計算
    TransRot(r_eerot,r_roll,r_pitch,r_yaw);
    TransRot(l_eerot,l_roll,l_pitch,l_yaw);

    
    //1列目
    r_pos.carPos[0][0]=r_eerot[0];
    r_pos.carPos[0][1]=r_eerot[1];
    r_pos.carPos[0][2]=r_eerot[2];
	  
    //2列目
    r_pos.carPos[1][0]=r_eerot[3];
    r_pos.carPos[1][1]=r_eerot[4];
    r_pos.carPos[1][2]=r_eerot[5];
	  
    //3列目
    r_pos.carPos[2][0]=r_eerot[6];
    r_pos.carPos[2][1]=r_eerot[7];
    r_pos.carPos[2][2]=r_eerot[8];
	  
    //4列目
    r_pos.carPos[0][3]=r_x;//[mm]
    r_pos.carPos[1][3]=r_y;
    r_pos.carPos[2][3]=r_z;


    //1列目
    l_pos.carPos[0][0]=l_eerot[0];
    l_pos.carPos[0][1]=l_eerot[1];
    l_pos.carPos[0][2]=l_eerot[2];
	  
    //2列目
    l_pos.carPos[1][0]=l_eerot[3];
    l_pos.carPos[1][1]=l_eerot[4];
    l_pos.carPos[1][2]=l_eerot[5];
	  
    //3列目
    l_pos.carPos[2][0]=l_eerot[6];
    l_pos.carPos[2][1]=l_eerot[7];
    l_pos.carPos[2][2]=l_eerot[8];
	  
    //4列目
    l_pos.carPos[0][3]=l_x;//[mm]
    l_pos.carPos[1][3]=l_y;
    l_pos.carPos[2][3]=l_z;

    r_pos.elbow = 0;
    l_pos.elbow = 0;
	  
    d_returnid = m_DualManipulatorCommonInterface_Middle->moveLinearCartesianAbs(r_pos,l_pos);
    if(d_returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
	      
  }


  if(modesel == 3)
  {

    JARA_ARM_DUAL::ULONG rarm;
    JARA_ARM_DUAL::ULONG larm;
      
    std::cout << "Select : Gripper Open/Close" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Select Open/Close :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "1 : Open" << std::endl;
    std::cout << "2 : Close" << std::endl;
    std::cout << "------------------------------------" << std::endl;

    std::cout << ">>";
    std::cin >> childsel;
    std::cout << std::endl;

    if(childsel == 1)
    {
      std::cout << "Select : Open" << std::endl;

      rarm = 1;
      larm = 1;

      d_returnid =  m_DualManipulatorCommonInterface_Middle->openGripper(rarm,larm);
      if(d_returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    if(childsel == 2)
    {
      std::cout << "Select : Close"  << std::endl;

      rarm = 1;
      larm = 1;

      d_returnid =  m_DualManipulatorCommonInterface_Middle->closeGripper(rarm,larm);
      if(d_returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    
  }

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DualArmController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DualArmController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void DualArmController::TransRot(double eerot[],double roll,double pitch,double yaw)
{
  //ロールピッチヨー　ロボット工学（遠山茂樹著）p25
  double Rrotx;
  double Rroty;
  double Rrotz;
  Rrotx = roll * M_PI / 180;
  Rroty = pitch * M_PI / 180;
  Rrotz = yaw * M_PI / 180;

  eerot[0] = cos(Rroty)*cos(Rrotz);
  eerot[1] = cos(Rrotz)*sin(Rroty)*sin(Rrotx)-sin(Rrotz)*cos(Rrotx);
  eerot[2] = cos(Rrotz)*sin(Rroty)*cos(Rrotx)+sin(Rrotz)*sin(Rrotx);
  eerot[3] = sin(Rrotz)*cos(Rroty);
  eerot[4] = sin(Rrotz)*sin(Rroty)*sin(Rrotx)+cos(Rrotz)*cos(Rrotx);
  eerot[5] = sin(Rrotz)*sin(Rroty)*cos(Rrotx)-cos(Rrotz)*sin(Rrotx);
  eerot[6] = -sin(Rroty);
  eerot[7] = cos(Rroty)*sin(Rrotx);
  eerot[8] = cos(Rroty)*cos(Rrotx);
  
}

extern "C"
{
 
  void DualArmControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(dualarmcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<DualArmController>,
                             RTC::Delete<DualArmController>);
  }
  
};



// -*- C++ -*-
/*!
 * @file  SeedDualArmController.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "SeedDualArmController.h"
#include <math.h>

// Module specification
// <rtc-template block="module_spec">
static const char* seeddualarmcontroller_spec[] =
  {
    "implementation_id", "SeedDualArmController",
    "type_name",         "SeedDualArmController",
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
SeedDualArmController::SeedDualArmController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_DualManipulatorCommonInterface_MotionCommandsPort("DualManipulatorCommonInterface_MotionCommands"),
    m_DualManipulatorCommonInterface_CommonCommandsPort("DualManipulatorCommonInterface_CommonCommands")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SeedDualArmController::~SeedDualArmController()
{
}



RTC::ReturnCode_t SeedDualArmController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_DualManipulatorCommonInterface_MotionCommandsPort.registerConsumer("DualManipulatorCommonInterface_MotionCommands", "DualManipulatorCommonInterface::MotionCommands", m_DualManipulatorCommonInterface_MotionCommands);
  m_DualManipulatorCommonInterface_CommonCommandsPort.registerConsumer("DualManipulatorCommonInterface_CommonCommands", "DualManipulatorCommonInterface::CommonCommands", m_DualManipulatorCommonInterface_CommonCommands);
  
  // Set CORBA Service Ports
  addPort(m_DualManipulatorCommonInterface_MotionCommandsPort);
  addPort(m_DualManipulatorCommonInterface_CommonCommandsPort);

  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedDualArmController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedDualArmController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedDualArmController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SeedDualArmController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedDualArmController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedDualArmController::onExecute(RTC::UniqueId ec_id)
{
  int modesel;
  int childsel;
  
  std::cout << "------------------------------------" << std::endl;
  std::cout << "Please Select Mode :)" << std::endl;
  std::cout << "------------------------------------" << std::endl;
  std::cout << "1 : Arm Servo ON/OFF" << std::endl;
  std::cout << "2 : Gripper Servo ON/OFF" << std::endl;
  std::cout << "3 : Gripper and Arm Servo ON/OFF" << std::endl;
  std::cout << "4 : movePTPJointAbs" << std::endl;
  std::cout << "5 : movePTPJointRel" << std::endl;
  std::cout << "6 : moveLinearCartesianAbs" << std::endl;
  std::cout << "7 : moveLinearCartesianRel" << std::endl;
  std::cout << "8 : Gripper Open/Close" << std::endl;
  std::cout << "9 : Gripper Opening Controll" << std::endl;
  std::cout << "------------------------------------" << std::endl;

  std::cout << ">>";
  std::cin >> modesel;
  std::cout << std::endl;

  if(modesel == 1)
  {
    std::cout << "Select : Arm Servo ON/OFF" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Select ON/OFF :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "1 : ON" << std::endl;
    std::cout << "2 : OFF" << std::endl;
    std::cout << "------------------------------------" << std::endl;

    std::cout << ">>";
    std::cin >> childsel;
    std::cout << std::endl;

    if(childsel == 1)
    {
      std::cout << "Select : ON"  << std::endl;

      returnid =  m_DualManipulatorCommonInterface_CommonCommands->servoONArm();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    if(childsel == 2)
    {
      std::cout << "Select : OFF"  << std::endl;

      returnid =  m_DualManipulatorCommonInterface_CommonCommands->servoOFFArm();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    
  }
  
  if(modesel == 2)
  {
    std::cout << "Select : Gripper Servo ON/OFF"  << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Select ON/OFF :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "1 : ON" << std::endl;
    std::cout << "2 : OFF" << std::endl;
    std::cout << "------------------------------------" << std::endl;

    std::cout << ">>";
    std::cin >> childsel;
    std::cout << std::endl;

    if(childsel == 1)
    {
      std::cout << "Select : ON" << std::endl;

      returnid =  m_DualManipulatorCommonInterface_CommonCommands->servoONHand();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    if(childsel == 2)
    {
      std::cout << "Select : OFF" << std::endl;

      returnid =  m_DualManipulatorCommonInterface_CommonCommands->servoOFFHand();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    
  }
  
  if(modesel == 3)
  {
    std::cout << "Select : Gripper and Arm Servo ON/OFF" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Select ON/OFF :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "1 : ON" << std::endl;
    std::cout << "2 : OFF" << std::endl;
    std::cout << "------------------------------------" << std::endl;

    std::cout << ">>";
    std::cin >> childsel;
    std::cout << std::endl;

    if(childsel == 1)
    {
      std::cout << "Select : ON" << std::endl;

      returnid =  m_DualManipulatorCommonInterface_CommonCommands->servoON();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    if(childsel == 2)
    {
      std::cout << "Select : OFF"  << std::endl;

      returnid =  m_DualManipulatorCommonInterface_CommonCommands->servoOFF();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    
  }
  
  if(modesel == 4)
  {
    DualManipulatorCommonInterface::JointPos jointPoint;
    jointPoint.length(14);
    
    std::cout << "Select : movePTPJointAbs"  << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Enter Each Joint Angle :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Right Arm" << std::endl;
    for(int i=0; i<7; i++)
    {
      std::cout << "Joint " << i+1 << " Angle" << std::endl;
      std::cin >> jointPoint[i];
    }
    std::cout << "------------------------------------" << std::endl;
    std::cout << "left Arm" << std::endl;
    for(int i=0; i<7; i++)
    {
      std::cout << "Joint " << i+1 << " Angle" << std::endl;
      std::cin >>jointPoint[i+7];
    }

    returnid = m_DualManipulatorCommonInterface_MotionCommands->movePTPJointAbs(jointPoint);
    if(returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
   
  }

  if(modesel == 5)
  {
    DualManipulatorCommonInterface::JointPos jointPoint;
    jointPoint.length(14);
    
    std::cout << "Select : movePTPJointRel"  << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Enter Each Joint Angle :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Right Arm" << std::endl;
    for(int i=0; i<7; i++)
    {
      std::cout << "Joint " << i+1 << " Angle" << std::endl;
      std::cin >> jointPoint[i];
    }
    std::cout << "------------------------------------" << std::endl;
    std::cout << "left Arm" << std::endl;
    for(int i=0; i<7; i++)
    {
      std::cout << "Joint " << i+1 << " Angle" << std::endl;
      std::cin >>jointPoint[i+7];
    }

    returnid = m_DualManipulatorCommonInterface_MotionCommands->movePTPJointRel(jointPoint);
    if(returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
   
  }

  if(modesel == 6)
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
    
    DualManipulatorCommonInterface::CarPosWithElbow r_pos;
    DualManipulatorCommonInterface::CarPosWithElbow l_pos;
	  
    std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
    std::cout<< "roll : ";
    std::cin >> r_roll;
    std::cout << "pitch : ";
    std::cin >> r_pitch;
    std::cout << "yaw : ";
    std::cin >> r_yaw;
    std::cout << std::endl; 
	  
    std::cout << "座標を入力してください[mm]" << std::endl;
    std::cout << "x座標 : "; 
    std::cin >> r_x;
    std::cout << "y座標 : ";
    std::cin >> r_y;
    std::cout << "z座標 : ";
    std::cin >> r_z;
    std::cout << std::endl;

    std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
    std::cout<< "roll : ";
    std::cin >> l_roll;
    std::cout << "pitch : ";
    std::cin >> l_pitch;
    std::cout << "yaw : ";
    std::cin >> l_yaw;
    std::cout << std::endl; 
	  
    std::cout << "座標を入力してください[mm]" << std::endl;
    std::cout << "x座標 : "; 
    std::cin >> l_x;
    std::cout << "y座標 : ";
    std::cin >> l_y;
    std::cout << "z座標 : ";
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
	  
    returnid = m_DualManipulatorCommonInterface_MotionCommands->moveLinearCartesianAbs(r_pos,l_pos);
    if(returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
	      
  }


  if(modesel == 7)
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
    
    DualManipulatorCommonInterface::CarPosWithElbow r_pos;
    DualManipulatorCommonInterface::CarPosWithElbow l_pos;
	  
    std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
    std::cout<< "roll : ";
    std::cin >> r_roll;
    std::cout << "pitch : ";
    std::cin >> r_pitch;
    std::cout << "yaw : ";
    std::cin >> r_yaw;
    std::cout << std::endl; 
	  
    std::cout << "座標を入力してください[mm]" << std::endl;
    std::cout << "x座標 : "; 
    std::cin >> r_x;
    std::cout << "y座標 : ";
    std::cin >> r_y;
    std::cout << "z座標 : ";
    std::cin >> r_z;
    std::cout << std::endl;

    std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
    std::cout<< "roll : ";
    std::cin >> l_roll;
    std::cout << "pitch : ";
    std::cin >> l_pitch;
    std::cout << "yaw : ";
    std::cin >> l_yaw;
    std::cout << std::endl; 
	  
    std::cout << "座標を入力してください[mm]" << std::endl;
    std::cout << "x座標 : "; 
    std::cin >> l_x;
    std::cout << "y座標 : ";
    std::cin >> l_y;
    std::cout << "z座標 : ";
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
	  
    returnid = m_DualManipulatorCommonInterface_MotionCommands->moveLinearCartesianRel(r_pos,l_pos);
    if(returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
	      
  }


  if(modesel == 8)
  {
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

      returnid =  m_DualManipulatorCommonInterface_MotionCommands->openGripper();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    if(childsel == 2)
    {
      std::cout << "Select : Close"  << std::endl;

      returnid =  m_DualManipulatorCommonInterface_MotionCommands->closeGripper();
      if(returnid->id != 0)
      {
	std::cout << "------------------------------------" << std::endl;
        std::cout << "ERROR :(" << std::endl;
	std::cout << "------------------------------------" << std::endl;
      }
    }
    
  }

  if(modesel == 9)
  {
    DualManipulatorCommonInterface::DoubleSeq r_angle;
    DualManipulatorCommonInterface::DoubleSeq l_angle;

    r_angle.length(1);
    l_angle.length(1);
      
    std::cout << "Select : Gripper Opening Controll"  << modesel << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Please Enter Opening Controll Value :)" << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Right Arm" << std::endl;
    std::cin >> r_angle[0];
    std::cout << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "left Arm" << std::endl;
    std::cin >> l_angle[0];
    std::cout << std::endl;

    returnid =  m_DualManipulatorCommonInterface_MotionCommands->moveGripper(r_angle,l_angle);
    
    if(returnid->id != 0)
    {
      std::cout << "------------------------------------" << std::endl;
      std::cout << "ERROR :(" << std::endl;
      std::cout << "------------------------------------" << std::endl;
    }
    
  }

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedDualArmController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedDualArmController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedDualArmController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedDualArmController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedDualArmController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


void SeedDualArmController::TransRot(double eerot[],double roll,double pitch,double yaw)
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
 
  void SeedDualArmControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(seeddualarmcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<SeedDualArmController>,
                             RTC::Delete<SeedDualArmController>);
  }
  
};



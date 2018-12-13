#!/bin/sh

COM="gnome-terminal -e"
export HOST="localhost"

echo "rtm-naming"
sudo rtm-naming

cho "roscore"
$COM "roscore"

cd ../SEATML
$COM "eSEAT SEED-Noid_Bridge_command_r.seatml"
$COM "eSEAT SEED-Noid_Bridge_command_l.seatml"
$COM "eSEAT SEED-Noid_Bridge_state_r.seatml"
$COM "eSEAT SEED-Noid_Bridge_state_l.seatml"
cd ../script

$COM "../RTC/DualArmController/build/src/DualArmControllerComp -f ../RTC/DualArmController/rtc.conf"
$COM "../RTC/DualArmSimulation2.0/build/src/DualArmSimulationComp -f ../RTC/DualArmSimulation2.0/rtc.conf"

rtcon $HOST/DualArmController0.rtc:DualManipulatorCommonInterface_Common $HOST/DualArmSimulation0.rtc:DualManipulatorCommonInterface_Common
rtcon $HOST/DualArmController0.rtc:DualManipulatorCommonInterface_Middle $HOST/DualArmSimulation0.rtc:DualManipulatorCommonInterface_Middle
rtcon $HOST/DualArmSimulation0.rtc:r_JointAngle_out $HOST/SEED-Noid_Bridge_command_r.rtc:in
rtcon $HOST/DualArmSimulation0.rtc:l_JointAngle_out $HOST/SEED-Noid_Bridge_command_l.rtc:in
rtcon $HOST/DualArmSimulation0.rtc:r_JointAngle_in $HOST/SEED-Noid_Bridge_state_r.rtc:out
rtcon $HOST/DualArmSimulation0.rtc:l_JointAngle_in $HOST/SEED-Noid_Bridge_state_l.rtc:out

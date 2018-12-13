#!/bin/sh

COM="gnome-terminal -e"
export HOST="localhost"

sudo rtm-naming

$COM "../RTC/DualArmController/build/src/DualArmControllerComp -f ../RTC/DualArmController/rtc.conf"
$COM "../RTC/SeedUpperBody2.0/build/src/SeedUpperBodyComp -f ../RTC/SeedUpperBody2.0/rtc.conf"

rtconf $HOST/SeedUpeerBody0.rtc set port_name "/dev/ttyUSB0"


echo ""
echo "SEED-Noid上半身制御コンポーネントを接続します" 
echo ""

rtcon $HOST/DualArmController0.rtc:DualManipulatorCommonInterface_Common $HOST/SeedUpperBody0.rtc:DualManipulatorCommonInterface_Common
rtcon $HOST/DualArmController0.rtc:DualManipulatorCommonInterface_Middle $HOST/SeedUpperBody0.rtc:DualManipulatorCommonInterface_Middle

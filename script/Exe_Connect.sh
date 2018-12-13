#!/bin/sh

COM="gnome-terminal -e"
export HOST="localhost"

echo "ネーミングサーバーを実行します。"
sudo rtm-naming

$COM "../RTC/SeedDualArmController/build/src/SeedDualArmControllerComp -f ../RTC/SeedDualArmController/rtc.conf"
$COM "../RTC/SeedUpperBody/build/src/SeedUpperBodyComp -f ../RTC/SeedUpperBody/rtc.conf"

rtconf $HOST/SeedUpeerBodyRTC0.rtc set port_name "/dev/ttyUSB0"


echo ""
echo "SEED-Noid上半身制御コンポーネントを接続します" 
echo ""

rtcon $HOST/SeedUpperBody0.rtc:DualManipulatorCommonInterface_CommonCommands $HOST/SeedDualArmController0.rtc:DualManipulatorCommonInterface_CommonCommands
rtcon $HOST/SeedUpperBody0.rtc:DualManipulatorCommonInterface_MotionCommands $HOST/SeedDualArmController0.rtc:DualManipulatorCommonInterface_MotionCommands

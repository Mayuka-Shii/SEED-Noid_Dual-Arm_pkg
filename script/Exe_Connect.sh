#!/bin/sh

COM="gnome-terminal -e"
export HOST="localhost/rsdlab.host_cxt"

echo "ネーミングサーバーを実行します。"
sudo rtm-naming

echo ""
echo "SEED-Noid上半身制御コンポーネント群を起動します。"
echo "デフォルトではシリアルポートは /dev/ttyUSB0 に設定されます。"
echo "変更する場合RTSystemEditorを起動して変更して下さい"
echo ""

cd ../RTC/SeedDualArmController/
$COM "build/src/SeedDualArmControllerComp"
cd ../SeedUpperBody/
$COM "build/src/SeedUpperBodyComp"

rtconf $HOST/SeedUpeerBodyRTC0.rtc set port_name "/dev/ttyUSB0"


echo ""
echo "SEED-Noid上半身制御コンポーネントを接続します" 
echo ""

rtcon $HOST/SeedUpperBody0.rtc:DualManipulatorCommonInterface_CommonCommands $HOST/SeedDualArmController0.rtc:DualManipulatorCommonInterface_CommonCommands
rtcon $HOST/SeedUpperBody0.rtc:DualManipulatorCommonInterface_MotionCommands $HOST/SeedDualArmController0.rtc:DualManipulatorCommonInterface_MotionCommands

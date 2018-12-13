# SEED-Noid における双腕作業のための RTC 群

## 概要
当初の概要：THK社のSEED-Noidに対して,双腕ロボット制御機能共通インタフェース（以下、双腕共通I/F）を適用。  
現在の概要：双腕共通I/Fをより良いものとするため、双腕共通I/Fを拡張した双腕共通I/F2.0を提案し、THK社のSEED-Noidに対して,双腕共通I/F2.0を適用。  
実機制御のRTC群と、シミュレータ上のSEED-Noidを動かせるようなRTC群の開発を行った。

## 開発環境
言語：c++  
OS：Ubuntu16.04  
RTM：OpenRTM-aist-1.1.2-RELEASE  
ROS：ROS kinetic  
Gazebo：Gazebo7  
eSEAT：eSEAT2.5  

## 開発コンポーネント  
### [SeedUpperBody](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/tree/master/RTC/SeedUpperBody2.0)
双腕共通I/F2.0によってSEED-Noid実機の双腕を制御することができるRTC  

### [DualArmController](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/tree/master/RTC/DualArmController)
双腕共通I/F2.0を用いたサンプルコントローラRTC  
双腕共通I/F2.0の機能をCUIで呼び出すことが可能  

### [DualArmSimulation](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/tree/master/RTC/DualArmSimulation2.0)
双腕共通I/F2.0によってシミュレーション上のSEED-Noidを制御するためのメインとなるRTC  
ROSとのブリッジから現在関節角度の受け取り、目標関節角度を送信を行っている  

### [SEED-Noid_Bridge_state_r](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/blob/master/SEATML/SEED-Noid_Bridge_state_r.seatml)
### [SEED-Noid_Bridge_state_l](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/blob/master/SEATML/SEED-Noid_Bridge_state_l.seatml)
ROS側から送信された現在関節角度をRTM側に渡すブリッジRTC  
本RTCはeSEATによって動作する  

### [SEED-Noid_Bridge_command_r](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/blob/master/SEATML/SEED-Noid_Bridge_command_r.seatml)
### [SEED-Noid_Bridge_command_l](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/blob/master/SEATML/SEED-Noid_Bridge_command_l.seatml)
RTM側から送信された目標関節角度をROS側に渡すブリッジRTC  
本RTCはeSEATによって動作する  

## ドキュメント
### マニュアル
[SEED-Noid における双腕作業のための RTC 群マニュアル](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/blob/master/SEED-Noid_Dual-Arm_pkg_Manual.pdf)

### 仕様書
[双腕ロボット制御機能共通インタフェース2.0仕様書](https://github.com/Mayuka-Shii/SEED-Noid_Dual-Arm_pkg/blob/master/interface_doublearm_2.0.pdf)


## 今後の展望
RTMとROSのブリッジ作成マニュアル公開予定

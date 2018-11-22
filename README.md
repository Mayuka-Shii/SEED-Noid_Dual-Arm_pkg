# SEED-Noid における双腕作業のための RTC 群

## 概要
THK社のSEED-Noidに対して,双腕ロボット制御機能共通インタフェースを適用

## 開発環境
言語：c++  
OS：Ubuntu16.04  
RTM：OpenRTM-aist-1.1.2-RELEASE  

## 開発コンポーネント  
### SeedUpperBody
既存のSEED-Noid上半身制御RTCに双腕I/Fを追加したRTC  
参考：[https://github.com/rsdlab/SeedUpperBody-RTM-pkg/tree/master/RTC/SeedUpperBodyRTC](https://github.com/rsdlab/SeedUpperBody-RTM-pkg/tree/master/RTC/SeedUpperBodyRTC)

### SeedDualArmController
SEEDUpperBodyを双腕I/Fで操作するためのRTC

## マニュアル
[SEED-Noid における双腕作業のための RTC 群マニュアル]()

## 更新

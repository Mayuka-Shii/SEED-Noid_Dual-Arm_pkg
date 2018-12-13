#!bin/sh

export HOST="localhost"

rtdeact $HOST/DualArmSimulation0.rtc
rtdeact $HOST/DualArmController0.rtc
rtdeact $HOST/SEED-Noid_Bridge_command_r.rtc
rtdeact $HOST/SEED-Noid_Bridge_command_l.rtc
rtdeact $HOST/SEED-Noid_Bridge_state_r.rtc
rtdeact $HOST/SEED-Noid_Bridge_state_l.rtc

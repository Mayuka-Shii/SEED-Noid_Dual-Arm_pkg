#!bin/sh

export HOST="localhost"

rtact $HOST/DualArmSimulation0.rtc
rtact $HOST/DualArmController0.rtc
rtact $HOST/SEED-Noid_Bridge_command_r.rtc
rtact $HOST/SEED-Noid_Bridge_command_l.rtc
rtact $HOST/SEED-Noid_Bridge_state_r.rtc
rtact $HOST/SEED-Noid_Bridge_state_l.rtc

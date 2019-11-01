from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random



try:
#Insert dir of sumo tools
    sys.path.append('/usr/local/Cellar/sumo/1.3.1/share/sumo/tools')
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

#Insert dir of sumo-gui
sumoBinary = "/usr/local/Cellar/sumo/1.3.1/bin/sumo-gui"

#Insert dir of config file
sumoCmd = [sumoBinary, "-c", "simple_config/hello.sumocfg"]

import traci
import traci.constants as tc

traci.start(sumoCmd)
print("HEJ")
vehID="veh0"
traci.vehicle.subscribe(vehID, (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
print(traci.vehicle.getSubscriptionResults(vehID))
for step in range(20):
    print("step", step)
    traci.simulationStep()
    print(traci.vehicle.getSubscriptionResults(vehID))
    print(traci.vehicle.getSpeed("veh0"))

    if step==6:
        traci.vehicle.setSpeed(vehID,6)
    if step > 6 and traci.vehicle.getSpeed(vehID) == 6:
        traci.vehicle.setSpeed(vehID, -1)

traci.close()

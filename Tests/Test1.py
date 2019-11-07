from __future__ import absolute_import
from __future__ import print_function
import sys

""" Configuration code """
step_length = 0.1
run_config = "sumo-gui" # "sumo" or "sumo-gui"



try:
#Insert dir of sumo tools
    sys.path.append('/usr/local/Cellar/sumo/1.3.1/share/sumo/tools')
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

#Insert dir of sumo-gui
sumoBinary = "/usr/local/Cellar/sumo/1.3.1/bin/" + run_config

#Insert dir of config file
step_length = 0.1
sumoCmd = [sumoBinary, "-c", "TestNetworks/test.sumocfg", '--step-length', str(step_length)]


""" Main code """
import math
import traci
import traci.constants as tc
from SystemController import SystemController

traci.start(sumoCmd)
controller = SystemController(traci.simulation, step_length)
vehicle_stack = []

inc_lanes = ["-gneE3_0", "gneE4_0", "gneE2_0", "-gneE5_0"]
out_lanes = ["gneE3_0", "-gneE4_0", "-gneE2_0", "gneE5_0"]
inc_speed = 5
out_speed = 15

for step in range(10000):
    traci.simulationStep()
    for car in traci.simulation.getDepartedIDList():
        vehicle_stack.append(car)
        traci.vehicle.setSpeedMode(car, 00)
    for car in traci.simulation.getArrivedIDList():
        vehicle_stack.remove(car)

    if step != 0 and step % 10 == 0:
        for car in vehicle_stack:
            if traci.vehicle.getLaneID(car) in inc_lanes:
                traci.vehicle.setSpeed(car, inc_speed)
            elif traci.vehicle.getLaneID(car) in out_lanes:
                traci.vehicle.setSpeed(car, out_speed)






traci.close()

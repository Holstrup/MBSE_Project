from __future__ import absolute_import
from __future__ import print_function
import sys

""" Configuration code """

try:
#Insert dir of sumo tools
    sys.path.append('/usr/local/Cellar/sumo/1.3.1/share/sumo/tools')
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

#Insert dir of sumo-gui
sumoBinary = "/usr/local/Cellar/sumo/1.3.1/bin/sumo"

#Insert dir of config file
step_length = 0.1
sumoCmd = [sumoBinary, "-c", "networks/test.sumocfg", '--step-length', str(step_length)]


""" Main code """
import math
import traci
import traci.constants as tc
from SystemController import SystemController

traci.start(sumoCmd)
controller = SystemController(traci.simulation)

for step in range(10000):
    traci.simulationStep()

    if len(traci.simulation.getArrivedIDList()) > 0 or len(traci.simulation.getDepartedIDList()) > 0:
        controller.update_id_list()

    if step != 0 and step % 10 == 0:
        print("step", step)
        controller.update_state(step)





traci.close()

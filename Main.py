import sys
import ControlStrategy
from TrafficGenerator import *
import libraries.traci as traci
import libraries.sumolib as sumolib


class Main:

    def __init__(self):
        # init path vars
        self.sumoBinary = ""
        self.config_path = ""
        self.sumo_cmd = []
        # fill paths with local sumo installation
        self.locate_sumo_installation()
        # configure simulation parameters
        self.num_steps = 10000
        self.step_length = 0.1
        # set sumo command
        self.sumo_cmd = [self.sumoBinary, "-c", self.config_path, "--step-length", str(self.step_length), "--verbose"]
        # configure traffic density
        self.vehicle_appearance_probability = 0.001
        # init control strategy
        self.control_strategy = None
        # choose control strategy by ID:
        #   0: FIFO
        #   1: Right Hand Precedence
        #   2: Traffic Light
        #   3: Grid
        self.select_cs(2)
        # init traffic generator
        self.traffic_generator = TrafficGenerator(self.vehicle_appearance_probability,
                                                  getattr(self.control_strategy, 'routes'))

    def locate_sumo_installation(self):
        # replace with if-else statements to fit all users
        try:
            # Insert dir of sumo tools
            self.sumoBinary = "C:/Users/Bosse/Documents/00_DTU/01_Master/01_First_Semester/02223_Model-Based_Systems_Engineering/sumo-1.3.1/bin/sumo-gui.exe"
            self.config_path = "C:/Users/Bosse/Documents/00_DTU/01_Master/01_First_Semester/02223_Model-Based_Systems_Engineering/git/MBSE_Project/networks/test.sumocfg"
        except ImportError:
            sys.exit(
                "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

    def enable_log(self):
        self.sumo_cmd.append("--full-output")
        self.sumo_cmd.append("tmp_log.xml")

    def select_cs(self, idx):
        if idx == 0:
            self.control_strategy = ControlStrategy.FifoControl()
        elif idx == 1:
            self.control_strategy = ControlStrategy.RhpControl()
        elif idx == 2:
            self.control_strategy = ControlStrategy.TlControl()
        elif idx == 3:
            self.control_strategy = ControlStrategy.GridControl()
        else:
            print("ERR: Invalid control strategy index. Exiting...")
            sys.exit()

    def check_setup(self):
        if not self.control_strategy:
            print("ERR: Control strategy not configured. Exiting...")
            return False
        elif not 100 <= self.num_steps <= 100000:
            print("ERR: Number of simulation steps is out of range [100, 100 000]. Exiting...")
            return False
        # Todo: add check for traffic density. Depends on step length and vehicle appearance probability. We should
        #  establish a suitable range

    def run(self):
        traci.start(self.sumo_cmd)
        for step in range(self.num_steps):
            traci.simulationStep()
            self.traffic_generator.generate_traffic_flow()

# instantiate object
main = Main()
# uncomment to get log file:
# main.enable_log()
# run the simulation
main.run()
traci.close()

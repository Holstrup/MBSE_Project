import sys
import os
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
        self.num_steps = 6000
        self.step_length = 0.1
        # set sumo command
        self.sumo_cmd = [self.sumoBinary, "-c", self.config_path, "--step-length", str(self.step_length), "--verbose"]
        # configure traffic density
        self.vehicle_appearance_probability = 0.03
        # init control strategy
        self.control_strategy = None
        # choose control strategy by ID:
        #   0: FIFO
        #   1: Right Hand Precedence
        #   2: Traffic Light
        #   3: Grid
        #   4: None
        self.select_cs(3)
        # init traffic generator
        self.traffic_generator = TrafficGenerator(self.vehicle_appearance_probability,
                                                  getattr(self.control_strategy, 'routes'))

    def locate_sumo_installation(self):
        path_a = "/usr/local/Cellar/sumo/1.3.1/bin/sumo-gui"
        path_b = "C:/Users/Bosse/Documents/00_DTU/01_Master/01_First_Semester/02223_Model-Based_Systems_Engineering" \
                 "/sumo-1.3.1/bin/sumo-gui.exe"
        path_l = "/usr/local/Cellar/sumo/1.3.1/bin/sumo-gui"

        if os.path.exists(path_a):
            self.sumoBinary = path_a
        elif os.path.exists(path_b):
            self.sumoBinary = path_b
        elif os.path.exists(path_l):
            self.sumoBinary = path_l
        else:
            print("ERR: Could not find sumo gui")
            sys.exit()

        rel_config_path = "networks/test.sumocfg"
        self.config_path = os.path.join(os.path.dirname(__file__), rel_config_path)

    def enable_log(self):
        self.sumo_cmd.append("--full-output")
        self.sumo_cmd.append("data/" + str(self.vehicle_appearance_probability*10) + "_" + self.control_strategy.get_name() + ".xml")

    def select_cs(self, idx):
        if idx == 0:
            self.control_strategy = ControlStrategy.FifoControl()
        elif idx == 1:
            self.control_strategy = ControlStrategy.RhpControl()
        elif idx == 2:
            self.control_strategy = ControlStrategy.TlControl()
            self.control_strategy.set_step_length(self.step_length)
        elif idx == 3:
            self.control_strategy = ControlStrategy.GridControl()
        elif idx == 4:
            self.control_strategy = ControlStrategy.NoControl()
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
        elif not 0.01 <= self.vehicle_appearance_probability / self.step_length <= 0.5:
            print("ERR: Traffic flow settings not supported. The vehicle appearance probability divided by the step"
                  " length must be between 0.1 and 2.0. It is currently " + str(self.vehicle_appearance_probability /
                                                                                self.step_length))
            return False
        else:
            return True

    def run(self):
        if not self.check_setup():
            sys.exit()

        traci.start(self.sumo_cmd)
        for step in range(self.num_steps):
            traci.simulationStep()
            self.traffic_generator.generate_traffic_flow()

            self.control_strategy.control(step)


# instantiate object
main = Main()
# uncomment to get log file:
# main.enable_log()
# run the simulation
main.run()
traci.close()

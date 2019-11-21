import config_traci
from Control_Logic import ControlLogic2

import traci
import traci.constants as tc

traci.start(config_traci.sumoCmd, label="sim1")
sim1=traci.getConnection("sim1")
control = ControlLogic2(sim1)

#define all right turns
control.define_right_turn('-gneE5','-gneE2')
control.define_right_turn('gneE2','-gneE4')
control.define_right_turn('gneE4','gneE3')
control.define_right_turn('-gneE3','gneE5')


for step in range(10000):

    sim1.simulationStep()
    if sim1.vehicle.getIDCount()>0:

        #if any vehicles have departed in the given simulated step, add to vehicle list
        if sim1.simulation.getDepartedNumber() > 0:
            control.register_vehicle(sim1.simulation.getDepartedIDList())


        control.run_sim()







traci.close()


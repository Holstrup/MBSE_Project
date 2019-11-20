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

        #update lists
        control.update_vehicle_list()



        for road_id in control.get_watch_list():

            vehicle_que=control.watch_dict[road_id]

            if len(vehicle_que) >= 1:
                for vehicle in vehicle_que:

                    control.counter_dict[vehicle]+=1
                    other_vehicles = control.get_first_cars_in_que(road_id)
                    can_go=True

                    if len(other_vehicles) >= 1:
                        for other in other_vehicles:
                            if control.will_collide(vehicle,other):
                                if not control.is_prioritized(vehicle, other):
                                    can_go = False


                            #both cars are stopped
                            elif sim1.vehicle.getStopState(vehicle) == 1 and sim1.vehicle.getStopState(other) == 1:

                                if control.counter_dict[vehicle]<control.counter_dict[other]:

                                    can_go = False


                        if not can_go and control.vehicle_dict[vehicle] == True and control.can_stop(vehicle):

                            control.stop_at_junction(vehicle)
                        #If car is stopping but can go
                        elif control.vehicle_dict[vehicle] == False and can_go and sim1.vehicle.getStopState(vehicle) ==0:

                            control.cancel_stop(vehicle)

                            #control.remove_from_watch(road_id,vehicle)
                        elif sim1.vehicle.getStopState(vehicle) ==1 and can_go:

                            sim1.vehicle.resume(vehicle)
                            control.vehicle_dict[vehicle] = False

                    else:
                        if sim1.vehicle.getStopState(vehicle) ==0 and control.vehicle_dict[vehicle] ==False:
                            control.cancel_stop(vehicle)

                        elif sim1.vehicle.getStopState(vehicle) == 1:

                            sim1.vehicle.resume(vehicle)
                            control.vehicle_dict[vehicle] = True
                            control.counter_dict[vehicle] = 0








traci.close()


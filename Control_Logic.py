import math


class ControlLogic2:

    def __init__(self,trac):
        self.traci=trac
        self.vehicle_dict={}
        self.right_turn_dict={}
        self.inc_lanes = [ "gneE2", "gneE4","-gneE3","-gneE5"]
        self.watch_dict={"gneE2":[], "gneE4":[],"-gneE5":[],"-gneE3":[]}
        self.que=[]
        self.counter_dict={}

    def register_vehicle(self,departed_id_list):
        for id in departed_id_list:
            #register vehicle in dict with the priority based on the desired path

            #Car is moving
            self.vehicle_dict[id] = True
            #Counter how long car has been at stop
            self.counter_dict[id]=0

            #Set vehicle speedmode to 7
            self.traci.vehicle.setSpeedMode(id,7)

    def update_vehicle_list(self):
        vehicles_to_delete=[]
        for vehicle_id in self.vehicle_dict.keys():

            current_pos=self.traci.vehicle.getRoadID(vehicle_id)
            if current_pos not in self.inc_lanes:
                vehicles_to_delete.append(vehicle_id)

            #add to watch list if less than halfway to the junction or if que at lane
            elif self.dist_to_junction(vehicle_id)<50 and vehicle_id not in self.watch_dict[current_pos]:

                self.watch_dict[current_pos].append(vehicle_id)


        for vehicle in vehicles_to_delete:
            inc_pos=self.traci.vehicle.getRoute(vehicle)[0]

            del self.vehicle_dict[vehicle]
            self.watch_dict[inc_pos].remove(vehicle)

    def remove_from_watch(self,road_id,vehicle_id):
        self.watch_dict[road_id].remove(vehicle_id)

    def get_vehicles(self):
        return self.vehicle_dict.keys()

    def define_right_turn(self,in_pos,out_pos):
        self.right_turn_dict[in_pos]=out_pos

    #function to check if vehicle is turning right
    def turning_right(self,vehicle_id):
        in_pos=self.traci.vehicle.getRoute(vehicle_id)[0]
        out_pos = self.traci.vehicle.getRoute(vehicle_id)[1]
        if in_pos in self.right_turn_dict:
            if self.right_turn_dict[in_pos]==out_pos:
                return True

        #if either first or second if  True  then the car is not turning right
        return False

    def going_straight(self,vehicle_id):
        in_pos = self.traci.vehicle.getRoute(vehicle_id)[0]
        out_pos = self.traci.vehicle.getRoute(vehicle_id)[1]
        if (in_pos[0] == "-" and out_pos[0] == "-") or (in_pos[0]=="g" and out_pos[0])=="g":
            return True
        else:
            return False

    def is_crossing(self, vehicle_id):
        if self.turning_right(vehicle_id) or self.going_straight(vehicle_id):
            return False
        else:
            return True

    #priority if cars are across from each other
    def car_path_priority(self,vehicle):
        if self.turning_right(vehicle):
            return 1
        elif self.going_straight(vehicle):
            return 2
        elif self.is_crossing(vehicle):
            return 3

    def dist_to_junction(self, vehicle_id):
        vehicle_pos = self.traci.vehicle.getPosition(vehicle_id)
        junction_pos=(0,0)
        return math.sqrt((vehicle_pos[0] - junction_pos[0]) ** 2 + (vehicle_pos[1] - junction_pos[1]) ** 2)

    def time_to_junction(self, vehicle_id):

        dist = self.dist_to_junction(vehicle_id)
        speed = self.traci.vehicle.getSpeed(vehicle_id)
        return dist / (speed + 0.0001)

    def dist_to_stop(self,vehicle_id):
        speed=self.traci.vehicle.getSpeed(vehicle_id)
        dec=self.traci.vehicle.getDecel(vehicle_id)
        traveled_distance=speed**2/(2*dec)
        return traveled_distance

    def get_watch_list(self):
        return self.watch_dict.keys()


    '''def get_sorted_watch_list(self):
        #return list of [(key,value),(key2,value2)] where value is lowest
        return sorted(self.watch_dict.items(), key=lambda kv: kv[1])'''

    def is_to_the_right(self,vehicle1,vehicle2):
        vehicle1_pos = self.traci.vehicle.getRoadID(vehicle1)
        vehicle2_pos = self.traci.vehicle.getRoadID(vehicle2)
        index_vehicle1 = self.inc_lanes.index(vehicle1_pos)
        index_vehicle2=self.inc_lanes.index(vehicle2_pos)
        if index_vehicle1 + 1 == index_vehicle2 or (index_vehicle1 == 3 and index_vehicle2 == 0):
            return True
        else:
            return False

    def cars_to_the_right(self, vehicle):
        vehicle_pos = self.traci.vehicle.getRoadID(vehicle)
        index_vehicle = self.inc_lanes.index(vehicle_pos)

        #find pos to the right

        if index_vehicle == 3:
            pos_right = self.inc_lanes[0]
        else:
            pos_right = self.inc_lanes[index_vehicle+1]

        if len(self.watch_dict[pos_right]) >= 1:
            return True
        else:
            return False

    def is_across(self,vehicle1,vehicle2):
        vehicle1_pos = self.traci.vehicle.getRoadID(vehicle1)
        vehicle2_pos = self.traci.vehicle.getRoadID(vehicle2)

        index_vehicle1 = self.inc_lanes.index(vehicle1_pos)
        index_vehicle2 = self.inc_lanes.index(vehicle2_pos)

        to_the_left=(index_vehicle1 - 1 == index_vehicle2 or (index_vehicle1 == 0 and index_vehicle2 == 3))
        if not(self.is_to_the_right(vehicle1,vehicle2) and to_the_left):
            return True
        else:
            return False

    def is_to_the_left(self,vehicle1,vehicle2):
        vehicle1_pos = self.traci.vehicle.getRoadID(vehicle1)
        vehicle2_pos = self.traci.vehicle.getRoadID(vehicle2)

        index_vehicle1 = self.inc_lanes.index(vehicle1_pos)
        index_vehicle2 = self.inc_lanes.index(vehicle2_pos)

        return (index_vehicle1 - 1 == index_vehicle2 or (index_vehicle1 == 0 and index_vehicle2 == 3))

    def will_collide(self,vehicle1,vehicle2):
        time_to_turn=2
        #if 2 vehicles have less than x time between them
        time_to_intersection1=self.time_to_junction(vehicle1)
        time_to_intersection2 = self.time_to_junction(vehicle2)

        if abs(time_to_intersection1-time_to_intersection2)<time_to_turn:
            return True
        else:
            return False

    def form_vehicle_dict(self,vehicle_list):
        dict={}
        for vehicle in vehicle_list:
            dist_to_junc=self.dist_to_junction(vehicle)
            dict[vehicle]=[dist_to_junc,self.can_stop(vehicle)]
        return dict

    def is_prioritized(self,vehicle1,vehicle2):

        prioritized = True

        if self.cars_to_the_right(vehicle1):
            return False
        # checks if vehicle across is going straight or right
        elif self.is_across(vehicle1,vehicle2):
            vehicle1_prio=self.car_path_priority(vehicle1)
            vehicle2_prio=self.car_path_priority(vehicle2)
            if vehicle2_prio>vehicle1_prio:

                prioritized = False
            elif vehicle2_prio==vehicle1_prio and vehicle2_prio==3:
                veh1_dist=self.dist_to_junction(vehicle1)
                veh2_dist = self.dist_to_junction(vehicle2)
                if veh1_dist>veh2_dist:
                    prioritized = False
        # if vehicle 2 is in the intersection
        elif self.traci.vehicle.getRoadID(vehicle2) not in self.inc_lanes:
            prioritized = False

        return prioritized

    def can_stop(self,vehicle):

        braking_dist=self.dist_to_stop(vehicle)
        dist_to_end_of_road=self.dist_to_junction(vehicle)-25


        if braking_dist > dist_to_end_of_road:

            return False
        else:

            return True

    def stop_at_junction(self, vehicle_id):

        #from center of junction to end of road
        dist_from_center_to_end=7.5
        current_pos=self.traci.vehicle.getRoadID(vehicle_id)

        self.traci.vehicle.setStop(vehicle_id,current_pos,100-dist_from_center_to_end)
        self.vehicle_dict[vehicle_id] = False




    def cancel_stop(self,vehicle_id):
        #from center of junction to end of road

        current_pos=self.traci.vehicle.getRoadID(vehicle_id)
        self.traci.vehicle.setSpeed(vehicle_id,-1)
        #self.traci.vehicle.setStop(vehicle_id,current_pos,100-7.5,duration=0)
        self.vehicle_dict[vehicle_id]=True
        self.counter_dict[vehicle_id]=0


    def get_first_cars_in_que(self,road_id):
        qued_cars=[]
        for each_inc_road in self.inc_lanes:
            if each_inc_road != road_id and len(self.watch_dict[each_inc_road])>=1:
                qued_cars.append(self.watch_dict[each_inc_road][0])
                if len(self.watch_dict[each_inc_road])>=2:
                    qued_cars.append(self.watch_dict[each_inc_road][1])
        return qued_cars


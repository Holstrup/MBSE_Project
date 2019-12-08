import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

def read_from_file(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    vehicles = {}
    edges_data = {}

    for data in root:
        timestep = data.attrib["timestep"]
        for element in data:
            if element.tag == "vehicles":
                for instance in element:
                    id = instance.attrib["id"]
                    if id not in vehicles:
                        vehicles[id] = {"speed": [], "x": [], "y":[], "fuel": []}

                    vehicles[id]["speed"].append(instance.attrib["speed"])
                    vehicles[id]["x"].append(instance.attrib["x"])
                    vehicles[id]["y"].append(instance.attrib["y"])
                    vehicles[id]["fuel"].append(instance.attrib["fuel"])
            elif element.tag == "edges":
                for instance in element:
                    for lane in instance:
                        id = lane.attrib["id"]
                        meanspeed = lane.attrib["meanspeed"]
                        if id not in edges_data:
                            edges_data[id] = {"timestep": [], "meanspeed": []}
                        edges_data[id]["timestep"].append(timestep)
                        edges_data[id]["meanspeed"].append(meanspeed)


    return vehicles, edges_data, timestep



def read_net_file(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    edges = {}

    for data in root:
        if data.tag == "edge":
            for lane in data:
                id = lane.attrib["id"]
                speed = lane.attrib["speed"]
                shape = (lane.attrib["shape"]).split(" ")
                x_coor, y_coor = [], []
                for coordinate in shape:
                    coordinate = coordinate.split(",")
                    x_coor.append(float(coordinate[0]))
                    y_coor.append(float(coordinate[1]))

                edges[id] = {"speed": speed, "x": x_coor, "y": y_coor}
    return edges



def plot_lane_speed(x, y, z):
    x = np.array(x).astype(np.float)
    y = np.array(y).astype(np.float)
    z = np.array(z).astype(np.float)

    # Create a set of line segments so that we can color them individually
    # This creates the points as a N x 1 x 2 array so that we can stack points
    # together easily to get the segments. The segments array for line collection
    # needs to be (numlines) x (points per line) x 2 (for x and y)
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    fig, axs = plt.subplots()

    # Create a continuous norm to map from data points to colors
    lc = LineCollection(segments, cmap='viridis')
    # Set the values used for colormapping
    lc.set_array(z)
    lc.set_linewidth(2)
    line = axs.add_collection(lc)
    cbar = fig.colorbar(line, ax=axs)
    cbar.ax.set_ylabel('Speed [m/s]', rotation=270)


    axs.set_xlim(-50, 50)
    axs.set_ylim(-50, 50)
    plt.xlabel("X-direction")
    plt.ylabel("Y-direction")
    plt.show()





#edges = read_net_file("test01.net.xml")


def average_speed():
    speeds = []
    for vehicle in vehicledata.keys():
        v_speed = vehicledata[vehicle]["speed"]
        v_speed = list(map(float, v_speed))
        speeds.append(sum(v_speed) / len(v_speed))
    return (sum(speeds) / len(speeds)) * 3.6

def plot_edge(edges, edge_data, timestep=0):
    for edge_id in edges.keys():
        x, y = edges[edge_id]["x"], edges[edge_id]["y"]
        plt.plot(x, y, 'r-')
    plt.show()


# Safety Metric: (Cars involved in collisions) / (Total number of cars)
def safety(collisions, total_cars):
    return collisions / total_cars


# Efficiency Metric: (Cars Through Intersection) / (Minute)
def efficiency(vehicle_data, max_time):
    return len(vehicle_data.keys()) / (float(max_time)) * 60


# Greta Thunberg Metric: (Sum of car pollution) / (Number of cars)
def pollution(vehicle_data):
    total_fuel_averages = []
    for vehicle in vehicle_data.keys():
        fueldata = vehicle_data[vehicle]["fuel"]
        results = list(map(float, fueldata))
        total_fuel_averages += [sum(results) / 10]
    return sum(total_fuel_averages) / len(total_fuel_averages)


ControlLogic = ["Traffic Light", "Right Hand Precedence Control", "FIFO Control", "Grid Control"]
Prob = ["0.01", "0.05", "0.1", "0.15", "0.2", "0.3", "0.4"]

for probability in Prob:
    print("\nProbability: " + probability)
    vehicledata, edge_data, max_timestep = read_from_file("Data/" + probability + "_" + ControlLogic[3] + "NEW.xml")
    print("No. Vehicles : " + str(len(vehicledata.keys())))
    print("Cars / Minute   : " + str(efficiency(vehicledata, 600)))
    print("Average speed: " + str(average_speed()))
    print("Pollution   : " + str(pollution(vehicledata)))



def plot_speeds():
    for i, vehicle in enumerate(vehicledata.keys()):
        if i == 10: break
        v_speed = vehicledata[vehicle]["speed"]
        v_speed = list(map(float, v_speed))
        past_pointer = 0
        v_speed_sec = []
        for j in range(10, len(v_speed), 10):
            v_speed_sec.append(sum(v_speed[past_pointer:j]) / 10)
            past_pointer = j

        plt.scatter(range(0, len(v_speed_sec), 1), v_speed_sec)
        x1, x2, y1, y2 = plt.axis()
        plt.axis((x1, x2, 0, 20))
        if i == 9:
            print(v_speed_sec)
            plt.show()

def plot_acceleration():
    for i, vehicle in enumerate(vehicledata.keys()):
        if i == 10: break
        v_speed = vehicledata[vehicle]["speed"]
        v_speed = list(map(float, v_speed))
        past_pointer = 0
        v_speed_sec = []
        for i in range(10, len(v_speed), 10):
            v_speed_sec.append(sum(v_speed[past_pointer:i]) / 10)
            past_pointer = i
        v_acc_sec = []
        for i in range(1, len(v_speed_sec), 1):
            v_acc_sec.append(v_speed_sec[i] - v_speed_sec[i - 1])
        plt.scatter(range(0, len(v_acc_sec), 1), v_acc_sec)
        x1, x2, y1, y2 = plt.axis()
        plt.axis((x1, x2, -5, 5))
        plt.show()

#plot_acceleration()
#plot_speeds()

"""
for i, car in enumerate(vehicledata.keys()):
    x = vehicledata[car]["x"]
    y = vehicledata[car]["y"]
    z = vehicledata[car]["speed"]
    plot_lane_speed(x, y, z)
    if i == 10: break
"""
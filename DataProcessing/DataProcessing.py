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
                        vehicles[id] = {"speed": [], "x": [], "y":[]}

                    vehicles[id]["speed"].append(instance.attrib["speed"])
                    vehicles[id]["x"].append(instance.attrib["x"])
                    vehicles[id]["y"].append(instance.attrib["y"])
            elif element.tag == "edges":
                for instance in element:
                    for lane in instance:
                        id = lane.attrib["id"]
                        meanspeed = lane.attrib["meanspeed"]
                        if id not in edges_data:
                            edges_data[id] = {"timestep": [], "meanspeed": []}
                        edges_data[id]["timestep"].append(timestep)
                        edges_data[id]["meanspeed"].append(meanspeed)


    return vehicles, edges_data




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



edges = read_net_file("test01.net.xml")
vehicledata, edge_data = read_from_file("full_log")

def plot_edge(edges, edge_data, timestep=0):
    for edge_id in edges.keys():
        x, y = edges[edge_id]["x"], edges[edge_id]["y"]
        print(edges[edge_id]["speed"], edge_data[edge_id]["meanspeed"][80])
        print(float(edge_data[edge_id]["meanspeed"][80]) / float(edges[edge_id]["speed"]))
        plt.plot(x, y, 'r-')
    #plt.show()

plot_edge(edges, edge_data)




x = vehicledata["f06.1"]["x"]
y = vehicledata["f06.1"]["y"]
z = vehicledata["f06.1"]["speed"]
plot_lane_speed(x, y, z)



import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

def read_from_file(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    vehicles = {}

    for data in root:
        for element in data:
            if element.tag == "vehicles":
                for instance in element:
                    id = instance.attrib["id"]
                    if id not in vehicles:
                        vehicles[id] = {"speed": [], "x": [], "y":[]}

                    vehicles[id]["speed"].append(instance.attrib["speed"])
                    vehicles[id]["x"].append(instance.attrib["x"])
                    vehicles[id]["y"].append(instance.attrib["y"])
    return vehicles


vehicledata = read_from_file("full_log")


x = vehicledata["f06.1"]["x"]
y = vehicledata["f06.1"]["y"]
z = vehicledata["f06.1"]["speed"]



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
    fig.colorbar(line, ax=axs)

    axs.set_xlim(-50, 50)
    axs.set_ylim(-50, 50)
    plt.show()

plot_lane_speed(x,y,z)
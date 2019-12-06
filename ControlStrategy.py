class ControlStrategy:
    routes = ['du', 'dl', 'dr', 'ld', 'lr', 'lu', 'ul', 'ud', 'ur', 'ru', 'rl', 'rd']
    route_edge_dict = {'du': 'gneE2', 'dl': 'gneE2', 'dr': 'gneE2', 'ld': '-gneE5', 'lr': '-gneE5',
                       'lu': '-gneE5', 'ul': '-gneE3', 'ud': '-gneE3', 'ur': '-gneE3', 'ru': 'gneE4',
                       'rl': 'gneE4', 'rd': 'gneE4'}
    incoming_edges = ['gneE4', '-gneE5', 'gneE2', '-gneE3']
    edges_ud = ['gneE2', '-gneE3']
    edges_lr = ['gneE4', '-gneE5']

    def control(self):
        pass


class FifoControl(ControlStrategy):

    def __init__(self):
        self.name = "FIFO Control"
        self.id = 0

    def control(self):
        pass


class RhpControl(ControlStrategy):

    def __init__(self):
        self.name = "Right Hand Precedence Control"
        self.id = 1

    def control(self):
        pass


class TlControl(ControlStrategy):

    def __init__(self):
        self.name = "Traffic Light Control"
        self.id = 2

    def control(self):
        pass


class GridControl(ControlStrategy):

    def __init__(self):
        self.name = "Grid Control"
        self.id = 3

    def control(self):
        pass

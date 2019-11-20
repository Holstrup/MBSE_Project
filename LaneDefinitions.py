import numpy as np


down_up = np.array([[0, 1], [0, 1]]).astype(bool)
up_down = np.invert(down_up)

left_right = down_up.transpose()
right_left = up_down.transpose()

down_right = np.array([[0, 0],
                       [0, 1]]).astype(bool)

down_left = np.array([[1, 1],
                      [0, 1]]).astype(bool)

right_up = np.array([[0, 1],
                     [0, 0]]).astype(bool)

right_down = np.array([[1, 1],
                       [1, 0]]).astype(bool)

left_down = np.array([[0, 0],
                      [1, 0]]).astype(bool)

left_up = np.array([[0, 1],
                    [1, 1]]).astype(bool)

up_left = np.array([[1, 0],
                    [0, 0]]).astype(bool)
up_right = np.array([[1, 0],
                     [1, 1]]).astype(bool)


paths = {"down_up": down_up, "up_down": up_down,
         "left_right": left_right, "right_left": right_left,
         "down_right": down_right, "down_left": down_left,
         "right_up": right_up, "right_down": right_down,
         "left_down": left_down, "left_up": left_up,
         "up_left": up_left, "up_right": up_right}

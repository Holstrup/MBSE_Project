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


paths = {"du": down_up, "ud": up_down,
         "lr": left_right, "rl": right_left,
         "dr": down_right, "dl": down_left,
         "ru": right_up, "rd": right_down,
         "ld": left_down, "lu": left_up,
         "ul": up_left, "ur": up_right}

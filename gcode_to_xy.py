import matplotlib.pyplot as plt
import numpy as np

gcode = open("kaas.gcode", "r")

print("Gcode to XY")

def gcode_to_xy():
    x_array = []
    y_array = []
    start_position = 0
    end_position = 0
    x_coord = 0
    y_coord = 0
    for line in gcode:
        for pos, char in enumerate(line):
            if char == "X":
                x_coord = pos
            if char == "Y":
                y_coord = pos

        if line[0:3] == "G01":
            start_position = end_position
            end_position = (line[y_coord+1:-1])

        if line[0:3] == "G00":
            start_position = (line[x_coord + 1 : y_coord - 1])

        x_array.append(start_position)
        y_array.append(end_position)

    return x_array, y_array

x, y = gcode_to_xy()

x_n = np.array(x)
y_n = np.array(y)

gcode.close()

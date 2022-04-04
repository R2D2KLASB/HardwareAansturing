import matplotlib.pyplot as plt
import numpy as np


print("Gcode to XY")

def read_gcode(filename):

    relative_coords = []
    fixed_coords = []
    with open(f"{filename}", "r") as gcode:
        data = gcode.readlines()
        for i in data:
            tmp = []
            tmp_g = ''
            elements = i.split(' ')
            for e in elements:
                c = e[0]

                if c == 'X':
                    tmp.append(e.strip()[1:])
                if c == 'Y':
                    tmp.append(e.strip()[1:])
                if c == 'G' and len(elements) > 1:
                    tmp_g = e.strip()[1:]


            if len(tmp) > 0 and len(tmp_g) > 0:
                relative_coords.append((tmp_g, tmp))
            elif len(tmp) == 0 and len(tmp_g) > 0:
                relative_coords.append((tmp_g))

    return relative_coords


coords = read_gcode("kaas.gcode")

print(coords)

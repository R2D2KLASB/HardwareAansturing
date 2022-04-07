"""! @file
     @brief Python code to turn g-code into xy coordinates so the printer understands it.
"""


def readGcode(filename):
    """!
    @brief Read the g-code and set it to xy coords.
    @param coords this is where the coords are saved in.
    @details Reads the file with g-codes and checks for every line what the G number is and what the X and Y are. Then appends this split to an array.
    @return returns new array with shape: g-code number, X, Y.
    """

    coords = []
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
                coords.append((tmp_g, tmp))
            elif len(tmp) == 0 and len(tmp_g) > 0:
                coords.append((tmp_g))

    return coords


coords = readGcode("kaas.gcode")

print(coords)

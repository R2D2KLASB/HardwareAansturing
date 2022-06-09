## @package beeldverwerking.image.gcode
# Make G-Code from an vector image.
#
# Todo
# ===
# - Optimize gcode path for faster drawing.


from svg_to_gcode.svg_parser import parse_file
from svg_to_gcode.compiler import Compiler, interfaces
from svg_to_gcode import TOLERANCES
import re

TOLERANCES['approximation'] = 0.8

## Make from an svg image to Gcode.
# return gcode string.
def svgToGcode(path, name):
    # try:
        gcode_compiler = Compiler(interfaces.Gcode, movement_speed=100, cutting_speed=100, pass_depth=0)

        curves = parse_file(path + "/" + name) 

        gcode_compiler.append_curves(curves) 
        gcode_compiler.compile_to_file(path + "/" + "tmp.gcode", passes=1)
        
        with open(path + "/" + 'tmp.gcode', "r") as gcode:
            gcode = list(gcode)
            gcode = simplifier(gcode, 24000)

        return gcode
    # except:
    #     return False

## Remove not used lines, translate the gcode and scale with an given number.
# return gcode string. 
def simplifier(gcode, scale=False):
    tmp = []
    for line in gcode:
        if '-' not in line:
            tmp += [line]
    gcode = tmp
    if scale:
        scale = getScale(gcode, scale)
    down = False
    tmp_gcode = ''
    for line in gcode[2:]:
        if "M5" in line:
            down = True
        elif "G1" in line:
            for word in line.split(" "):
                if "G1" in word:
                    if down:
                        tmp_gcode += 'G0'
                        down = False
                    else:
                        tmp_gcode += word
                elif "F" in word:
                    continue
                elif "X" in word:
                    temp = re.compile("([a-zA-Z]+)([0-9]+)")
                    items = temp.match(word).groups()
                    tmp_gcode +=(items[0])
                    if scale:
                        tmp_gcode += str(float(items[1]) * scale).split('.')[0]
                    else:
                        tmp_gcode += str(float(items[1])).split('.')[0]
                elif "Y" in word:
                    temp = re.compile("([a-zA-Z]+)([0-9]+)")
                    items = temp.match(word).groups()
                    tmp_gcode +=(items[0])
                    if scale:
                        tmp_gcode += str(float(items[1]) * scale).split('.')[0]
                    else:
                        tmp_gcode += str(float(items[1])).split('.')[0]
                    tmp_gcode +=("")
                if "\n" not in word:
                    tmp_gcode +=(" ")
                elif "\n" in word:
                    tmp_gcode +=("\n")
    tmp_gcode += 'G28'
    return tmp_gcode

## Get the highest X and Y cordinates and calculate the scale factor for an given boundary size.
# return scale factor float
def getScale(gcode, maxXY):
    tmp = gcode
    x = 0
    y = 0
    for line in tmp:
        if "G" in line:
            for word in line.split(" "):
                if "X" in word:
                    temp = re.compile("([a-zA-Z]+)([0-9]+)")
                    items = temp.match(word).groups()
                    if float(items[1]) > x:
                        x = float(items[1])
                elif "Y" in word:
                    try:
                        temp = re.compile("([a-zA-Z]+)([0-9]+)")
                        items = temp.match(word).groups()
                        if float(items[1]) > y:
                            y = float(items[1])
                    except:
                        print('jammer')
    scale = maxXY / max(x, y)
    return scale

## todo
def Optimze(gcode):
    gcode = re.sub("[ ]{2,}"," ",gcode)
    gcode = re.sub(" \n", "\n", gcode)
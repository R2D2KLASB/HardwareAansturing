from editor import *
from gcode import *
import sys
from os.path import exists
import os


## Talker main
def main(args=None):

    # Command-line parameters
    par = sys.argv

    # Get package path
    rp = os.path.dirname(os.path.abspath(__file__))

    inputfile = par[1]
    outputfile = par[2]
    if exists(inputfile):
        extensions = ['jpg', 'jpeg', 'png']
        # Check if file is an image
        if inputfile.split('.')[-1] in extensions:
            with open(inputfile, "rb") as imageFile:
                edgeFile = imageToEdge(imageFile)
                svgFile = edgeToSVG(edgeFile, rp)
                gcode = svgToGcode(rp, 'tmp.svg')
                with open(outputfile, 'w') as outputFile:
                    outputFile.write(gcode)
        else:
            print("File not supported")        
    else:
        print("File doesn't excist")

if __name__ == '__main__':
    main()
    os.remove("tmp.gcode")
    os.remove("tmp.bmp")
    os.remove("tmp.svg")
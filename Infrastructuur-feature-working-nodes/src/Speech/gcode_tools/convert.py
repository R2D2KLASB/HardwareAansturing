import sys

with open(sys.argv[1], "r") as a_file, open("gcode_struct.txt", "a") as new_file:
    output = ""
    for idx, line in enumerate(a_file):
        if len(line.strip()) == 1:
            if idx > 4:
                output = output[:-2] + "},\n\n"
                output += f"//{line.strip()}\n\n"
                output += "{"
            else:
                output += f"//{line.strip()}\n\n"
                output += "{"
        elif line.strip()[0:3] == "G00":
            x = line.strip().find('X')
            y = line.strip().find('Y')
            if line.strip()[x+1] == '0' and line.strip()[y+1] == '0':
                continue
            output += "{"
            output += line.strip()[x+1:y-1]
            output += ", "
            output += line.strip()[y+1:len(line.strip())]
            output += ", \"G00\"},\n"
        elif line.strip()[0:3] == "G01":
            x = line.strip().find('X')
            y = line.strip().find('Y')
            output += "{"
            output += line.strip()[x+1:y-1]
            output += ", "
            output += line.strip()[y+1:len(line.strip())]
            output += ", \"G01\"},\n"

    output = output[:-2] + "},\n\n"
    new_file.write(output)
    new_file.close()

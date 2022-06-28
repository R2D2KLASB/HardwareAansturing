import sys

a_file = open(sys.argv[1], "r")

new_file = open("startGcode.txt", "a")

new_file.write(f"\n{sys.argv[1][:-4]}\n")

for line in a_file:
    new_file.write(f"{line.strip()}\n")

gcc := /usr/bin/g++ -std=c++17
rm := /usr/bin/rm
LIBS=-lsfml-graphics -lsfml-window -lsfml-system

.phony: clean clean_build remove_build

build: main.exe

main.exe: main.o ColorPicker.o ColorWheel.o ConfigState.o Game.o InputManager.o jsoncpp.o JsonManager.o main.o MainState.o plotter.o StateMachine.o VirtualPlotter.o
	$(gcc) -o main.exe main.o ColorPicker.o ColorWheel.o ConfigState.o Game.o InputManager.o jsoncpp.o JsonManager.o MainState.o plotter.o StateMachine.o VirtualPlotter.o $(LIBS)

ColorPicker.o: ColorPicker.cpp ColorWheel.hpp Game.hpp
	$(gcc) -c ColorPicker.cpp -o ColorPicker.o

ColorWheel.o: ColorWheel.cpp ColorWheel.hpp Game.hpp
	$(gcc) -c ColorWheel.cpp -o ColorWheel.o

ConfigState.o: ConfigState.cpp ConfigState.hpp ColorPicker.hpp Definitions.hpp Game.hpp MainState.hpp StateMachine.hpp
	$(gcc) -c ConfigState.cpp -o ConfigState.o

Game.o: Game.cpp Game.hpp ConfigState.hpp Definitions.hpp InputManager.hpp JsonManager.hpp StateMachine.hpp
	$(gcc) -c Game.cpp -o Game.o

InputManager.o: InputManager.cpp InputManager.hpp Definitions.hpp
	$(gcc) -c InputManager.cpp -o InputManager.o

jsoncpp.o: Library/JSON/jsoncpp.cpp Library/JSON/json/json.h Library/JSON/json/json-forwards.h
	$(gcc) -c Library/JSON/jsoncpp.cpp -o jsoncpp.o

JsonManager.o: JsonManager.cpp JsonManager.hpp Definitions.hpp Library/JSON/json/json.h Exceptions.hpp
	$(gcc) -c JsonManager.cpp -o JsonManager.o

main.o: main.cpp Definitions.hpp Game.hpp
	$(gcc) -c main.cpp -o main.o

MainState.o: MainState.cpp MainState.hpp Definitions.hpp Game.hpp VirtualPlotter.hpp StateMachine.hpp
	$(gcc) -c MainState.cpp -o MainState.o

plotter.o: plotter.cpp plotter.hpp DataTypes.hpp
	$(gcc) -c plotter.cpp -o plotter.o

StateMachine.o: StateMachine.cpp StateMachine.hpp GameState.hpp
	$(gcc) -c StateMachine.cpp -o StateMachine.o

VirtualPlotter.o: VirtualPlotter.cpp VirtualPlotter.hpp plotter.hpp
	$(gcc) -c VirtualPlotter.cpp -o VirtualPlotter.o

clean:
	$(rm) -f *.o
	$(rm) -f main.exe

clean_build: clean build

remove_build:
	$(rm) -f *.o

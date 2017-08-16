all: main

KdTree.o: KdTree.h KdTree.cpp
	g++ -c KdTree.cpp -O3 -std=c++11

PPM.o: PPM.h PPM.cpp
	g++ -c PPM.cpp -O3 -std=c++11

Object.o: Object.h Object.cpp
	g++ -c Object.cpp -O3 -std=c++11

main: main.cpp KdTree.o PPM.o Object.o 
	g++ main.cpp KdTree.o PPM.o Object.o -o main -O3 -std=c++11 `pkg-config --cflags --libs opencv`

clear:
	rm *.o

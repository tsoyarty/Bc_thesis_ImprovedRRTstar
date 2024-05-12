build: 
	g++ -I/usr/local/include/ompl-1.6 -I/usr/include/eigen3 -L/usr/local/lib main.cpp -o main -lompl

run:
	./main
 

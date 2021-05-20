all:
	g++ -c filter1.o filter1.cpp
	g++ -c transition.o -L../dependencies/lib -I../dependencies/include -ltransmission

	g++ -o transition transition.o filter1.o

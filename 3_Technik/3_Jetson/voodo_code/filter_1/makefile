all:
	g++ -c filter1.cpp
	g++ -c transition.cpp -L../dependencies/lib -I../dependencies/include -ltransmission

	g++ -o transition transition.o filter1.o

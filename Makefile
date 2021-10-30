CXX = clang++
CXXFLAGS  = -g -Wall -O3 -fopenmp=libomp -march=native -flto
LDFLAGS = -lm -flto

.PHONY : clean

default: main

BullyPath.o : BullyPath.hpp SpeedRangeSearch.hpp Trig.hpp BullyPath.cpp
	$(CXX) $(CXXFLAGS) -c BullyPath.cpp -o BullyPath.o

SpeedRangeSearch.o : Constants.hpp SpeedRangeSearch.hpp Trig.hpp SpeedRangeSearch.cpp
	$(CXX) $(CXXFLAGS) -c SpeedRangeSearch.cpp -o SpeedRangeSearch.o

Trig.o : Trig.hpp Trig.cpp
	$(CXX) $(CXXFLAGS) -c Trig.cpp -o Trig.o

main.o: BullyPath.hpp Constants.hpp SpeedRangeSearch.hpp Trig.hpp main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp -o main.o

main: BullyPath.o SpeedRangeSearch.o Trig.o main.o
	$(CXX) $(CXXFLAGS) -o main BullyPath.o SpeedRangeSearch.o Trig.o main.o $(LDFLAGS)

clean:
	*.o main
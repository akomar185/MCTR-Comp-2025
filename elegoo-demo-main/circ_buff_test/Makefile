CXX = g++
CFLAGS = -Wall -g

# $@ copies the target, "pagerank".
main: circular_buffer.o main.cpp
	$(CXX) $(CFLAGS) -o $@ $^

# $^ copies the dependency list, "list.h list.c"
circular_buffer.o: circular_buffer.cpp
	$(CXX) $(CFLAGS) -c $^


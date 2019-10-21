.PHONY: all clean

CC = g++
CPPFLAGS += -g -Wall -O3
CFLAGS += -std=c++11 -I.
LIBS = -lboost_system -lboost_random
PROC = pathfinders

all:$(PROC)
path_finders: main.cpp path_finder.cpp path_finder.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $? -o $@ $(LIBS)

clean:
	@rm -rf *.o *.dSYM a.out $(PROC)

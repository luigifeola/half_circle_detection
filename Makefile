CC = g++
CFLAGS = -g -Wall
SRCS = half_circle_detection.cpp
PROG = half_circle_detection

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

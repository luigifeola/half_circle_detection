CC = g++
CFLAGS = -g -Wall
SRCS = load_images.cpp
PROG = load_images

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

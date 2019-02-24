CC = g++
CFLAGS = -g -Wall
OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)
.RECIPEPREFIX +=

main: main.o makerosbag.o
    $(CC) $(CFLAGS) -o main main.o makerosbag.o $(LIBS)

catchmain:
    $(CC) $(CFLAGS) catchmain.cpp -c

catchtests:
    $(CC) $(CFLAGS) -o tests catchmain.o iotests.cpp makerosbag.cpp $(LIBS)

clean:
   -rm -f main *.o
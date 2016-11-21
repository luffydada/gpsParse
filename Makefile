.PHONY: all app lib clean

CROSS_COMPILE=/opt/arm-linux-gnueabihf/gcc-4.9/bin/arm-linux-gnueabihf-

CC=$(CROSS_COMPILE)gcc
CXX=$(CROSS_COMPILE)g++

CPPFLAGS:=-g -Wall -W -fPIC
CPPFLAGS+=-I/opt/arm-linux-gnueabihf/usr/include

LDFLAGS:= -L.
LDFLAGS+= -L/opt/arm-linux-gnueabihf/lib 
LDFLAGS+= -L/opt/arm-linux-gnueabihf/usr/lib
LDFLAGS+= -lpthread

OBJECTS_APP=main.o
OBJECTS_LIB=gpsParse.o
TARGET_APP=gpsParse
TARGET_LIB=libGpsParse.so

all:lib app
	
app:$(OBJECTS_APP)
	$(CXX) -o $(TARGET_APP) $^ -L./ -lGpsParse $(LDFLAGS) 

lib:$(OBJECTS_LIB)
	$(CXX) -shared -o $(TARGET_LIB) $^ $(LDFLAGS)

clean:
	-rm $(TARGET_APP) $(TARGET_LIB)
	-rm *.o

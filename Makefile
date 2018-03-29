CXXFLAGS = -O2 -g -std=c++11 -Wall -Wno-unused-result `pkg-config opencv --cflags `
OBJS = main.o 
LIBS = -pthread `pkg-config opencv --libs`
TARGET = main
$(TARGET):$(OBJS)
			$(CXX) -o $(TARGET) $(OBJS) $(LIBS)
all:$(TARGET)
	clean:
		rm -f $(OBJS) $(TARGET)


CXX=g++
PKGCONFIG=pkg-config
CXXFLAGS=$(shell $(PKGCONFIG) --cflags roscpp) $(shell $(PKGCONFIG) --cflags eigen3)
LIBS=$(shell $(PKGCONFIG) --libs roscpp) $(shell $(PKGCONFIG) --libs eigen3)
SRCS=phantomx.cpp inverse_kinematics.cpp gait_planning.cpp main.cpp
OBJS=$(SRCS:.cpp=.o)
TARGET=gait_planning

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS)  -o $(TARGET) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -g  -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)



#g++ inverse_kinematics.cpp gait_planning.cpp main.cpp -o inverse_kinematics `pkg-config --cflags --libs roscpp`

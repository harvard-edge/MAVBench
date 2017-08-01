GPU=1
CUDNN=1
AIRLIB_PATH=/home/ubuntu/AirSim/AirLib

CXX=g++

CFLAGS=-c -Wall -O3 -std=c++11 -DGPU=$(GPU) -DCUDNN=$(CUDNN)
INCLUDE:=`pkg-config --cflags opencv` `pkg-config --cflags eigen3` -I $(AIRLIB_PATH)/include -I darknet/src -I darknet/include
#~/libs/OpenTLD/src/3rdparty/cvblobs -I ~/libs/OpenTLD/src/3rdparty/libconfig -I ~/libs/OpenTLD/src/libopentld/imacq -I ~/libs/OpenTLD/src/libopentld/mftracker -I ~/libs/OpenTLD/src/libopentld/tld
LDFLAGS:=-L $(AIRLIB_PATH)/lib -lAirLib -L$(AIRLIB_PATH)/deps/MavLinkCom/lib -lMavLinkCom -L$(AIRLIB_PATH)/deps/rpclib/lib -lrpc -lpthread `pkg-config --libs opencv`
#~/libs/OpenTLD/build/lib -lopentld  -lconfig++  -lcvblobs `pkg-config --libs opencv`

ifeq ($(GPU), 1) 
INCLUDE+= -I /usr/local/cuda/include/
LDFLAGS+= -L/usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand
endif

ifeq ($(CUDNN), 1) 
LDFLAGS+= -lcudnn
endif

SOURCES=main.cpp misc.cpp error.cpp log__class.cpp drone.cpp objdetect.cpp track.cpp pid.cpp kcf/kcftracker.cpp kcf/fhog.cpp
#SOURCES=main.cpp log__class.cpp drone.cpp objdetect.cpp track.cpp pid.cpp kcf/kcftracker.cpp kcf/fhog.cpp

OBJDIR=build
OBJECTS=$(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.cpp=.o)))
DARKNET_OBJECTS=$(filter-out darknet/obj/darknet.o, $(wildcard darknet/obj/*.o))

EXECUTABLE=follow_the_leader

all: $(OBJDIR) $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(OBJECTS) $(DARKNET_OBJECTS) $(LDFLAGS) -o $@

$(OBJDIR):
	mkdir $(OBJDIR)

$(OBJDIR)/main.o: main.cpp drone.h objdetect.h track.h pid.h 
	$(CXX) $(CFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/objdetect.o: objdetect.cpp objdetect.h bounding_box.h
	$(CXX) $(CFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/track.o: track.cpp track.h bounding_box.h
	$(CXX) $(CFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/%.o: %.cpp %.h
	$(CXX) $(CFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/%.o: kcf/%.cpp $(wildcard kcf/*.hpp)
	$(CXX) $(CFLAGS) $(INCLUDE) $< -o $@

clean:
	rm -f $(EXECUTABLE)
	rm -rf $(OBJDIR)

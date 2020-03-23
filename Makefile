CXX = emcc
OUTPUT = test.js

SOURCES = src/test.cpp

LIBS = -lwebsocket.js
USE_WASM = -s WASM=1

all: $(SOURCES) $(OUTPUT)

$(OUTPUT): $(SOURCES)
	$(CXX)  $(SOURCES) -g4 -std=c++11 -o $(OUTPUT) $(LIBS) -O2 $(USE_WASM) -I /opt/ros/melodic/include -I include -s USE_BOOST_HEADERS=1

clean:
	rm -f $(OUTPUT)

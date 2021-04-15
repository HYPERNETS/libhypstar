MAJOR := 0
MINOR := 2
NAME := hypstar
VERSION := $(MAJOR).$(MINOR)

CC := g++
BUILD_DIR := build
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
INSTALL_DIR := /usr/local

C_SOURCES := $(wildcard src/*.cpp)
C_SOURCES += $(wildcard src/serial/*.cpp)
C_SOURCES += $(wildcard src/utils/*.cpp)
OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.cpp=.o)))

vpath %.cpp src:src/serial/:src/utils/
vpath %.h inc/:inc/serial/:inc/utils/

INCLUDES = -Iinc \
	-Iinc/serial \
	-Iinc/utils

CFLAGS := -std=gnu++11 -rdynamic -fPIC -O0 -g -Wall -Werror

lib: $(BUILD_DIR)/lib$(NAME).so

test_%: $(BUILD_DIR)/lib$(NAME).so
	@echo --------------------------
	@echo Building $@
	@echo --------------------------
	$(CC) -rdynamic $(INCLUDES) -L./$(BUILD_DIR) -Wl,-rpath=./$(BUILD_DIR) -o $(BUILD_DIR)/$@ test/$@.c -lhypstar -lrt 

	@echo --------------------------
	@echo Executing $@
	@echo --------------------------
	./$(BUILD_DIR)/$@
	

$(BUILD_DIR)/lib$(NAME).so: $(BUILD_DIR)/lib$(NAME).so.$(VERSION)
#	@echo ----- INFO: Linking lib 
	$(RM) $(BUILD_DIR)/lib$(NAME).so
	ln -s lib$(NAME).so.$(VERSION) $(BUILD_DIR)/lib$(NAME).so

$(BUILD_DIR)/lib$(NAME).so.$(VERSION): $(OBJECTS) 
	@echo ----- INFO: Building lib
	@echo C_SOURCES = $(C_SOURCES)
	@echo OBJECTS = $(OBJECTS)
	$(CC) -rdynamic -fPIC -lrt -shared -Wl,--export-dynamic -o $@ $(OBJECTS)

$(BUILD_DIR)/%.o : %.cpp | $(BUILD_DIR)
	@echo ----- INFO: Building file $<
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)
install: lib
	$(RM) /usr/lib/lib$(NAME).so*
	$(RM) $(INSTALL_DIR)/lib/lib$(NAME).so
	install -m 0644 $(BUILD_DIR)/lib$(NAME).so.$(VERSION) $(INSTALL_DIR)/lib/
	ln -s lib$(NAME).so.$(VERSION) /usr/local/lib/lib$(NAME).so
	install -m 0644 inc/hypstar.h $(INSTALL_DIR)/include
	install -m 0644 inc/hypstar_typedefs.hpp $(INSTALL_DIR)/include
	install -m 0644 inc/serial/libhypstar_linuxserial.h $(INSTALL_DIR)/include
	ldconfig
	

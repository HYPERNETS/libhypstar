DVER_MAJOR := 0
DVER_MINOR := 2
DVER_REVISION := 2
NAME := hypstar
VERSION := $(DVER_MAJOR).$(DVER_MINOR).$(DVER_REVISION)

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
LFLAGS := -rdynamic -fPIC -lrt -shared -Wl,--export-dynamic

$(shell git update-index -q --refresh)
ifneq ($(shell git status --porcelain),)
  COMMIT_HASH_MOD := -mod
endif
COMMIT_HASH := $(shell git rev-parse --short HEAD)$(COMMIT_HASH_MOD)
EXTRA_CFLAGS := -DDVER_MAJOR=$(DVER_MAJOR) -DDVER_MINOR=$(DVER_MINOR) -DDVER_REVISION=$(DVER_REVISION) -DDVER_HASH=\"$(COMMIT_HASH)\"

# If the first argument is "test_firmware_update"...
ifeq (test_firmware_update,$(firstword $(MAKECMDGOALS)))
  # use the rest as arguments for "test_firmware_update"
  RUN_ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
  # ...and turn them into do-nothing targets
  $(eval $(RUN_ARGS):;@:)
endif

lib: $(BUILD_DIR)/lib$(NAME).so

test_%: $(BUILD_DIR)/lib$(NAME).so
	@echo --------------------------
	@echo Building $@
	@echo --------------------------
	$(CC) $(EXTRA_CFLAGS) -rdynamic $(INCLUDES) -L./$(BUILD_DIR) -Wl,-rpath=./$(BUILD_DIR) -o $(BUILD_DIR)/$@ test/$@.c -lhypstar -lrt

	@echo --------------------------
	@echo Executing $@ $(RUN_ARGS)
	@echo --------------------------
	./$(BUILD_DIR)/$@ $(RUN_ARGS)
	

$(BUILD_DIR)/lib$(NAME).so: $(BUILD_DIR)/lib$(NAME).so.$(VERSION)
#	@echo ----- INFO: Linking lib 
	$(RM) $(BUILD_DIR)/lib$(NAME).so
	ln -s lib$(NAME).so.$(VERSION) $(BUILD_DIR)/lib$(NAME).so

$(BUILD_DIR)/lib$(NAME).so.$(VERSION): $(OBJECTS) 
	@echo ----- INFO: Building lib
	@echo C_SOURCES = $(C_SOURCES)
	@echo OBJECTS = $(OBJECTS)
	$(CC) $(EXTRA_CFLAGS) $(LFLAGS) -o $@ $(OBJECTS)

$(BUILD_DIR)/%.o : %.cpp | $(BUILD_DIR)
	@echo ----- INFO: Building file $<
	$(CC) $(EXTRA_CFLAGS) $(CFLAGS) $(INCLUDES) -c $< -o $@

.PHONY: clean install uninstall

clean:
	$(RM) -r $(BUILD_DIR)

install: lib
	$(RM) /usr/lib/lib$(NAME).so*
	$(RM) $(INSTALL_DIR)/lib/lib$(NAME).so
	$(RM) $(INSTALL_DIR)/lib/lib$(NAME).so.$(DVER_MAJOR)
	$(RM) $(INSTALL_DIR)/lib/lib$(NAME).so.$(DVER_MAJOR).$(DVER_MINOR)
	install -m 0644 $(BUILD_DIR)/lib$(NAME).so.$(VERSION) $(INSTALL_DIR)/lib/
	ln -s lib$(NAME).so.$(VERSION) $(INSTALL_DIR)/lib/lib$(NAME).so
	ln -s lib$(NAME).so.$(VERSION) $(INSTALL_DIR)/lib/lib$(NAME).so.$(DVER_MAJOR)
	ln -s lib$(NAME).so.$(VERSION) $(INSTALL_DIR)/lib/lib$(NAME).so.$(DVER_MAJOR).$(DVER_MINOR)
	install -m 0644 inc/hypstar.h $(INSTALL_DIR)/include
	install -m 0644 inc/hypstar_typedefs.hpp $(INSTALL_DIR)/include
	install -m 0644 inc/serial/libhypstar_linuxserial.h $(INSTALL_DIR)/include
	ldconfig

uninstall: 
	$(RM) /usr/lib/lib$(NAME).so*
	$(RM) $(INSTALL_DIR)/lib/lib$(NAME).so*
	$(RM) $(INSTALL_DIR)/include/hypstar*
	$(RM) $(INSTALL_DIR)/include/libhypstar*
	ldconfig

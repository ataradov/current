CFLAGS += -W -Wall --std=gnu11 -O3 -g3
CFLAGS += -D_GNU_SOURCE

SRCS = \
  gui.c \
  usb.c \
  os.c \
  os_thread.c \
  current.c \

HDRS = \
  gui.h \
  usb.h \
  os.h \
  os_thread.h \
  current.h \

LDFLAGS = -Llib

UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
  BIN = current
  CFLAGS += -DOS_LINUX
  CFLAGS += `pkg-config --cflags libusb-1.0`
  CFLAGS += -fno-diagnostics-show-caret
  LDFLAGS += -lraylib_lin -lm -lrt
  LDFLAGS += `pkg-config --libs libusb-1.0`
else
  BIN = current.exe
  CFLAGS += -DOS_WINDOWS
  CFLAGS += `pkg-config --cflags libusb-1.0`
  LDFLAGS += -Wl,--subsystem,console
  LIB_PATH = `pkg-config --variable=libdir libusb-1.0`
  LDFLAGS += -lraylib_win -lm -lopengl32 -lgdi32 -lwinmm
  LDFLAGS += $(LIB_PATH)/libusb-1.0.a
  LDFLAGS += $(LIB_PATH)/libwinpthread.a
endif

all: $(BIN)

$(BIN): $(SRCS) $(HDRS)
	gcc $(CFLAGS) $(SRCS) -o $(BIN) $(LDFLAGS)

gen_res:
	xxd -i res/icon.png > res/icon.h
	xxd -i res/arial_14.png > res/arial_14.h

clean:
	rm -fr $(BIN)



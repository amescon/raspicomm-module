obj-m += raspicommrs485.o

raspicommrs485-objs := module.o queue.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

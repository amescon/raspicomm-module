obj-m += raspicommrs485.o

raspicommrs485-objs := module.o queue.o

SRC = /home/mdk/raspicomm-module
LINUX_3_2_27 = /home/mdk/rpi/linux-rpi-3.2.27/
LINUX_3_6_11 = /home/mdk/rpi/linux-rpi-3.6.y/
LINUX_3_6_11_538 = /home/mdk/rpi/linux-rpi-3.6.y-538/
LINUX_3_10_19_600 = /home/mdk/rpi/linux-rpi-3.10.y/
LINUX_3_10_24_614 = /home/mdk/rpi/linux-rpi-3.10.24+614/
LINUX_3_10_25_622 = /home/mdk/rpi/linux-rpi-3.10.25+622/
LINUX_3_10_38_675 = /home/mdk/rpi/linux-rpi-3.10.38+675/
LINUX_3_12_22_691 = /home/mdk/rpi/linux-rpi-3.12.22+691/
LINUX_3_18_7_755 = /home/mdk/rpi/linux-rpi-3.18.7+755/

PREFIX = /home/mdk/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-

all:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_2_27) M=$(SRC) modules

3.2.27+:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_2_27) M=$(SRC) modules

3.6.11+:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_6_11) M=$(SRC) modules

3.6.11+538:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_6_11_538) M=$(SRC) modules

3.10.19+600:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_10_19_600) M=$(SRC) modules

3.10.24+614:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_10_24_614) M=$(SRC) modules

3.10.25+622:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_10_25_622) M=$(SRC) modules

3.10.38+675:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_10_38_675) M=$(SRC) modules

3.12.22+691:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_12_22_691) M=$(SRC) modules

3.18.7+755:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_18_7_755) M=$(SRC) modules

clean:
	$(MAKE) -C $(LINUX_3_2_27) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_6_11) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_6_11_538) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_10_19_600) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_10_24_614) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_10_25_622) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_10_38_675) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_12_22_691) M=$(SRC) clean
	$(MAKE) -C $(LINUX_3_18_7_755) M=$(SRC) clean
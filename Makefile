
PWD := $(shell pwd)  

.PHONY: build clean  

build:
	 $(MAKE) -C $(KDIR) M=$(PWD) modules  

clean:
	 rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c 


$(info Building with KERNELRELEASE = ${KERNELRELEASE}) 

KBUILD_CPPFLAGS += -std=gnu99
obj-m :=    cpufreq-sc8810.o

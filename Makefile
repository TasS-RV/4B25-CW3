include setup.conf


all: warp glaux

warp:
	mkdir -p build/ksdk1.1/work
	mkdir -p build/ksdk1.1/work/boards/Warp
	mkdir -p build/ksdk1.1/work/demos/Warp/src
	mkdir -p build/ksdk1.1/work/demos/Warp/armgcc/Warp
	cp -r tools/sdk/ksdk1.1.0/*					build/ksdk1.1/work
	cp src/boot/ksdk1.1.0/SEGGER*					build/ksdk1.1/work/demos/Warp/src/
	cp src/boot/ksdk1.1.0/config.h					build/ksdk1.1/work/demos/Warp/src/
	cp src/boot/ksdk1.1.0/boot.c					build/ksdk1.1/work/demos/Warp/src/
	cp src/boot/ksdk1.1.0/errstrs*					build/ksdk1.1/work/demos/Warp/src/
	cp src/boot/ksdk1.1.0/powermodes.c				build/ksdk1.1/work/demos/Warp/src/
	cp src/boot/ksdk1.1.0/warp.h					build/ksdk1.1/work/demos/Warp/src/
	
	cp src/boot/ksdk1.1.0/startup_MKL03Z4.S				build/ksdk1.1/work/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
	cp src/boot/ksdk1.1.0/gpio_pins.c				build/ksdk1.1/work/boards/Warp
	cp src/boot/ksdk1.1.0/gpio_pins.h				build/ksdk1.1/work/boards/Warp
	cp src/boot/ksdk1.1.0/config.h					build/ksdk1.1/work/boards/Warp
	
	cp src/boot/ksdk1.1.0/detect.*				build/ksdk1.1/work/demos/Warp/src/

	cp src/boot/ksdk1.1.0/CMakeLists-Warp.txt			build/ksdk1.1/work/demos/Warp/armgcc/Warp/CMakeLists.txt
	cp src/boot/ksdk1.1.0/devMMA8451Q.*				build/ksdk1.1/work/demos/Warp/src/
	cp src/boot/ksdk1.1.0/devRV8803C7.*				build/ksdk1.1/work/demos/Warp/src/


	cd build/ksdk1.1/work/lib/ksdk_platform_lib/armgcc/KL03Z4 && ./clean.sh; ./build_release.sh
	cd build/ksdk1.1/work/demos/Warp/armgcc/Warp && ./clean.sh; ./build_release.sh
	@echo "\n\nNow, run\n\n\tmake load-warp\n\n"


load-warp:
	$(JLINKPATH) -device MKL03Z32XXX4 -if SWD -speed 10000 -CommanderScript tools/scripts/warp.jlink.commands


connect-warp:
	$(JLINKPATH) -device MKL03Z32XXX4 -if SWD -speed 10000 -CommanderScript tools/scripts/connect.jlink.commands

connect-glaux:
	$(JLINKPATH) -device MKL03Z32XXX4 -if SWD -speed 10000 -CommanderScript tools/scripts/connect.jlink.commands

clean:
	rm -rf build/ksdk1.1/work

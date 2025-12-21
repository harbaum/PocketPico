BUILD_DIR = ./build

.PHONY: configure build clean fullclean

configure:
	@if [ -d "${BUILD_DIR}" ]; then echo "Project already configured. Maybe you need to call 'make clean'?" && false ; fi
	@mkdir ${BUILD_DIR}
	@cmake -B ${BUILD_DIR} -S .

build:
	@make -j4 -C ${BUILD_DIR}

clean:
	@make -C ${BUILD_DIR} clean

fullclean:
	@rm -rf ${BUILD_DIR}
	@echo "Build directory '${BUILD_DIR}' removed."

flash:
	@make -j4 -C ${BUILD_DIR}
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program build/PocketPico.elf verify reset exit"

reset:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c init -c reset -c exit

tetris:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/tetris.gb 0x10100000 verify reset exit"

mario:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/mario.gb 0x10100000 verify reset exit"

mario2:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/mario2.gb 0x10100000 verify reset exit"

zelda:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/zelda.gb 0x10100000 verify reset exit"

mario_n_tetris:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/mario.gb 0x10100000 verify reset exit"
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/tetris.gb 0x10180000 verify reset exit"

term:
	xterm -e cu -l /dev/ttyACM0 -s 115200

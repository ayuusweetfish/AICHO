mkdir build
cd build
PICO_TOOLCHAIN_PATH=~/.platformio/packages/toolchain-gccarmnoneeabi/bin cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make

# (cd build; make && ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000; init; reset halt; rp2040.core1 arp_reset assert 0; rp2040.core0 arp_reset assert 0; reset halt; program aicho_relay.elf verify; reset; exit"

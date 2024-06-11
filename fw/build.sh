mkdir build
cd build
PICO_TOOLCHAIN_PATH=~/.platformio/packages/toolchain-gccarmnoneeabi/bin cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make

# (cd build; make && cp aicho.uf2 /Volumes/RPI-RP2)
# (cd build; make && ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000; program aicho.elf verify; reset halt; rp2040.core1 arp_reset assert 0; rp2040.core0 arp_reset assert 0; exit")

mkdir build
cd build
PICO_TOOLCHAIN_PATH=~/.platformio/packages/toolchain-gccarmnoneeabi/bin cmake ..
make

# (cd build; make && cp aicho.uf2 /Volumes/RPI-RP2)

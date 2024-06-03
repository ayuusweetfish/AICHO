# ~/.platformio/packages/tool-openocd/bin/openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 1000"

cat >build/gdbinit <<EOF
define hook-quit
  set confirm off
end
define hook-run
  set confirm off
end
define hookpost-run
  set confirm on
end
set pagination off
target extended-remote localhost:3333
EOF

~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb build/aicho.elf -x build/gdbinit
rm build/gdbinit

# screen /dev/cu.usbmodem* 115200
# Detach: ^A D
# Exit: ^A K Y or ^A ^\ Y

cc uqoa_conv.c -O2 -o uqoa_conv -I../src
cc gen.c -O2 -o gen
~/Downloads/ffmpeg -i ~/Downloads/zq3jTYLUbiEf.128.mp3 -ar 48000 -to 60 -ac 1 -f s16le -acodec pcm_s16le - | ./uqoa_conv > zq3jTYLUbiEf.128.bin
./gen zq3jTYLUbiEf.128.bin auxdat.gdbinit

# source auxdat/auxdat.gdbinit

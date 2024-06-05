~/Downloads/ffmpeg -i ~/Downloads/zq3jTYLUbiEf.128.mp3 -ar 48000 -to 60 -ac 1 -f s16be -acodec pcm_s16be - | ~/Downloads/uqoa/uqoa_conv > zq3jTYLUbiEf.128.bin
./a.out zq3jTYLUbiEf.128.bin auxdat.gdbinit

# source auxdat/auxdat.gdbinit

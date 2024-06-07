cc uqoa_conv.c -O2 -o uqoa_conv -I../src
cc gen.c -O2 -o gen

binfiles=()
for name in Lorivox Lumisonic Harmonia Titanus; do
  for dir in In Ex; do
    ~/Downloads/ffmpeg -i ${name}_${dir}.wav -ar 48000 -to 60 -ac 1 -f s16le -acodec pcm_s16le - | ./uqoa_conv > ${name}_${dir}.bin
    binfiles+=(${name}_${dir}.bin)
  done
done
# ~/Downloads/ffmpeg -i ~/Downloads/zq3jTYLUbiEf.128.mp3 -ar 48000 -to 60 -ac 1 -f s16le -acodec pcm_s16le - | ./uqoa_conv > zq3jTYLUbiEf.128.bin

# ./gen zq3jTYLUbiEf.128.bin auxdat.gdbinit
./gen ${binfiles[@]} auxdat.gdbinit

# source auxdat/auxdat.gdbinit

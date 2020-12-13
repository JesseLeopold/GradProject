#!/bin/bash

# barf on error
set -e

# configure port, exit immediately, don't reset
picocom -qrX -b 115200 /dev/ttyACM0

# Prepare Output File
cat /dev/null > /home/jesse/workspace/autorun/out/fileHex.txt
cat /dev/null > /home/jesse/workspace/autorun/out/fileHexCropped.txt
cat /dev/null > /home/jesse/workspace/autorun/out/file.wav

# Convert the Audio File into a text-based Hex Dump Equivalent
echo "$ln xxd -p /home/respeaker/workspace/autorun/vep_aec_beamforming_node_out.wav"            | picocom -qrix 100 /dev/ttyACM0  &>> /home/jesse/workspace/autorun/out/fileHex.txt

# Delete first line of
sed '1d' /home/jesse/workspace/autorun/out/fileHex.txt >> /home/jesse/workspace/autorun/out/fileHexCropped.txt


# Regenerate the Audio File From the Hex Dump
xxd -r -p /home/jesse/workspace/autorun/out/fileHexCropped.txt > /home/jesse/workspace/autorun/out/file.wav


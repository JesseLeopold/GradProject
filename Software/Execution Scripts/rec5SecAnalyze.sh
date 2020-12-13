#!/bin/bash

# barf on error
set -e;

cd /home/jesse/workspace/autorun;

# configure port, exit immediately, don't reset
picocom -qrX -b 115200 /dev/ttyACM0;

# Run an Audio Recording Sequence, Record Feedback Locally
echo "$ln cd /home/respeaker/workspace/autorun; /home/respeaker/workspace/autorun/rec5sec.sh" | picocom -qrix 1000 /dev/ttyACM0;

sleep 6;

# Run a PocketSphinx Analysis Sequence
echo "$ln cd /home/respeaker/workspace/autorun; /home/respeaker/workspace/autorun/psAnalyze.sh" | picocom -qrix 1000 /dev/ttyACM0;

sleep 1;

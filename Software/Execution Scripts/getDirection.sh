#!/bin/bash

# Exit on error
set -e

# Obtain Detection of Arrival Data from audio recording log
cd /home/jesse/workspace/autorun/log
grep "DOA: " ./record.log | cut -c 5-

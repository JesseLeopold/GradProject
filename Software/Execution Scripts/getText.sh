#!/bin/bash

# Exit on error
set -e

# Collect the Speech to Text Data
cd /home/jesse/workspace/autorun/log
grep "Recognized:" ./analyze.log | cut -c 12-


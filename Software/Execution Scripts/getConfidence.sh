#!/bin/bash

# Exit on error
set -e

# Collect confidence data from lg
cd /home/jesse/workspace/autorun/log
grep "Confidence:" ./analyze.log | cut -c 13-

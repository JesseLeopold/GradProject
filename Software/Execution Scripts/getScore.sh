#!/bin/bash

# barf on error
set -e

# Retrieve Pocket Sphinx Analysis Score from log
cd /home/jesse/workspace/autorun/log
grep "Score:" ./analyze.log | cut -c 8-

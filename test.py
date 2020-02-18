#!/usr/bin/env python
from glob import glob
import os
files = glob("*.csv")
for file in files:
    os.remove(file)

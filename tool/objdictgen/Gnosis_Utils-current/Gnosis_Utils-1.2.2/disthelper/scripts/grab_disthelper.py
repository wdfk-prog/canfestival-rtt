import sys

# Enable scripts to run in place without disthelper installed

# Note that I ALWAYS want to find the copy that is in the
# directory above me. The user might have installed disthelper
# in site-packages, and it could be out of date.

import os
# get my directory
p = os.path.split(sys.argv[0])[0]
if len(p): p = os.path.abspath(p)
else: p = os.path.abspath('.')

# insert my directory into sys.path so I can grab other
# modules that live here
sys.path.insert(0,p)

# find disthelper/ parent directory & insert in sys.path
while not os.path.isdir(os.path.join(p,'disthelper')):
    p = os.path.split(p)[0]
        
sys.path.insert(0,p)

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random



try:
#Insert dir of sumo tools
    sys.path.append('/usr/local/Cellar/sumo/1.3.1/share/sumo/tools')

    from sumolib import checkBinary


except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
#Insert dir of sumo-gui
sumoBinary = "/usr/local/Cellar/sumo/1.3.1/bin/sumo-gui"

#Insert dir of config file
step_length=0.1
sumoCmd = [sumoBinary, "-c", "networks/test.sumocfg","--step-length",str(step_length)]


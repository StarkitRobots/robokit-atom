#sudo apt install v4l-utils

import os

def set_flags(flags, prefix = "v4l2-ctl --set-ctrl="):
	for k, v in flags.items():
		os.system(prefix + k + "=" + str(v))

flags = {"exposure_auto"                  : 1,
         "white_balance_temperature_auto" : 0,
         "exposure_auto_priority"         : 0,
         "backlight_compensation"         : 0,
         "focus_auto"                     : 0}

set_flags(flags)

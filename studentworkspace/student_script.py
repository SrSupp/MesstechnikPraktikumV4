#!/usr/bin/env python3
from messtechnik_helper import *

helene = MoveGroupHelper()
#main()
helene.add_tumor()

helene.set_led_blue(0)
helene.set_led_green(0)
helene.set_speed_scaler(1)
helene.move_ptp_home_pos()

print(helene.get_actual_pos())
helene.set_speed_scaler(0.3)
helene.set_led_green(255)
helene.move_ptp_abs_pos(0.289,0.0,0.30, *(pi, -pi/2, 0))

helene.sleep(1)

helene.probing_start()
helene.set_led_green(0)
helene.set_led_blue(255)
helene.set_speed_scaler(0.01)
helene.move_lin_rel_pos(0,0,-0.15,*(0,0,0))
helene.sleep(3)
helene.probing_end()

helene.set_led_green(255)
helene.set_led_blue(0)
helene.set_speed_scaler(0.1)
helene.move_lin_rel_pos(0,0,0.15,*(0,0,0))

helene.set_led_blue(0)
helene.set_led_green(0)
helene.set_speed_scaler(1)
helene.move_ptp_home_pos()
from messtechnik_helper import * #Load everything thats needed to control Helene
helene = MoveGroupHelper() #Create Helene Object
helene.add_tumor() 

#A Few helpful commands:
#helene.set_led_blue(value)       --       value can be an integer between 0 and 255, turns on the blue LED of Helene
#helene.set_led_green(value)       --       value can be an integer between 0 and 255, turns on the green LED of Helene
#helene.set_speed_scaler(value)     --       Sets each movement to the desired speed. Value is a float between 0 and 1
#print(helene.get_actual_pos())     --       prints the actual position of helene
#helene.sleep(time_in_s)            --       Pauses the code for the given time
#helene.move_ptp_home_pos()         --       let helene move to its home position via a ptp motion
#helene.move_ptp_abs_pos(x,y,z, *(pi, -pi/2, 0))  --  let helene move ptp to a given position. x,y and z are given in meter
#helene.move_lin_abs_pos(x,y,z, *(pi, -pi/2, 0))  --  let helene move lin to a given position. x,y and z are given in meter
#helene.move_lin_rel_pos(d_x,d_y,d_z,*(0,0,0))   -- let helene move linearly relative to its actual position. For example if d_z is -0.15 Helene will drive 15cm down. 
#helene.move_ptp_rel_pos(d_x,d_y,d_z,*(0,0,0))   -- let helene move ptp relative to its actual position. For example if d_z is -0.15 Helene will drive 15cm down. 
#helene.probing_start()   --    Please use this command before the puncture into the Tumor. It signals to the data logger when the measurement data should be saved.
#helene.probing_end()     --    After the puncture please use this command. It signals to the data logger when the measurement data should no longer be saved.
#A more detailed instruction in given in the script!

#<-----------your code below---------->
helene.set_led_blue(0)
helene.set_led_green(0)
helene.set_speed_scaler(0.5)
helene.move_ptp_home_pos()
print(helene.get_actual_pos())
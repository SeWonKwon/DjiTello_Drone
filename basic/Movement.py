from djitellopy import tello

from time import sleep

me = tello.Tello()

me.connect()

print(me.get_battery())

me.takeoff()

# send_rc_control( left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity):

me.send_rc_control(0, 50, 0, 0)

print(me.getsend_read_command('height?'))

sleep(2) # 2 second

me.send_rc_control(0, 0, 0, 30)

sleep(2)

me.send_rc_control(0, 0, 0, 0)

me.land()
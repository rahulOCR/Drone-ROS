import rospy
from obs.functions import *
from obs.colours import *


def main():
    rospy.init_node("drone_controller", anonymous=True)
    drone = Drone()
    drone.set_mode('GUIDED')
    drone.wait4connect()
    drone.wait4start()


    drone.initialize_local_frame()
    ''' Takeoff Altitude'''
    drone.takeoff(10)
    rate = rospy.Rate(3)

    '''Square waypoints with alt = 5m side = 6.5m'''
    # goals = [[0, 0, 5, 0], [6.5, 0, 5, -90], [6.5, 6.5, 5, 0], [0, 6.5, 5, 90], [0, 0, 5, 180], [0, 0, 5, 0]]

    '''Triangle Waypoints with alt= 10 m , side of 10 m '''
    goals = [[0, 0, 10, 0], [8.6602, 5, 10, -60], [0, 10 , 10 , 60], [0, 0, 10, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

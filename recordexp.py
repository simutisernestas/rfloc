import rospy
from sensor_msgs.msg import Range
from getdata import read_measurements

def talker():
    pub = rospy.Publisher('chatter', Range, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    msg = Range()
    while not rospy.is_shutdown():
        devices = read_measurements(dummy=False)
        # rospy.loginfo(f"Number of devices: {len(devices)}")
        # rospy.loginfo(f"Devices: {str(devices)}")
        for (device,meas) in devices.items():
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = device
            msg.range = meas["Range"]
            pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
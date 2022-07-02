class F1tenth:
    def __init__(self):
        rospy.init_node("auto_driver")

        scan_topic = '/scan'
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

        drive_topic = 'auto_drive'
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

def main():
    
    scan_topic = '/scan'
    scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

    drive_topic = 'auto_drive'
    drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    rospy.spin()

if __name__ == "__main__":
    main()
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import QuaternionStamped, PoseWithCovarianceStamped, PoseStamped

def callback(msg):
    pub_msg = PoseWithCovarianceStamped()
    pub_msg.header = msg.header
    
    # Set position to 0
    pub_msg.pose.pose.position.x = 0.0
    pub_msg.pose.pose.position.y = 0.0
    pub_msg.pose.pose.position.z = 0.0
    
    # Copy orientation
    pub_msg.pose.pose.orientation = msg.quaternion
    
    # Set covariance (diagonal elements to small value, others 0)
    # The EKF will ignore position if config is set correctly, but we provide valid covariance anyway
    cov = [0.0] * 36
    # Orientation covariance (indices 21, 28, 35 for roll, pitch, yaw variance)
    # But flattening is: 0-5 (row 0), 6-11 (row 1)...
    # indices: 0(x), 7(y), 14(z), 21(roll), 28(pitch), 35(yaw)
    for i in range(0, 36, 7):
        cov[i] = 1e-6
        
    pub_msg.pose.covariance = cov
    
    pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('star_tracker_converter')
    
    # Publish PoseWithCovarianceStamped
    pub = rospy.Publisher('/star_tracker/pose', PoseWithCovarianceStamped, queue_size=10)
    
    # Subscribe to QuaternionStamped
    sub = rospy.Subscriber('/star_tracker/attitude', QuaternionStamped, callback)
    
    rospy.loginfo("Star Tracker Converter Node Started. Waiting for topic /star_tracker/attitude...")
    rospy.spin()

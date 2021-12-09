#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2

def callback(msg):

    print(msg.width * msg.height)
    data=""
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
        data = data + str(point[0])+","+str(point[1])+","+str(point[2])+"\n"
    
    f = open("data.txt","w")
    f.write(data)
    f.close()
    

def listener():

    rospy.init_node('deformationGraph',anonymous=True)
    rospy.Subscriber("/graph",PointCloud2,callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
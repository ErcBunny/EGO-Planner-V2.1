import numpy as np
import open3d as o3d
import rospy
import std_msgs
from sensor_msgs.msg import PointCloud2, PointField

if __name__ == "__main__":
    rospy.init_node("cmu_env_pcd_publisher")
    global_point_cloud_pub = rospy.Publisher(
        rospy.get_param("~global_pcd_publish_topic"), PointCloud2, queue_size=1
    )

    # get pcd
    pcd = o3d.io.read_point_cloud(rospy.get_param("~scene_id"))
    points = np.asarray(pcd.points)

    # create pcd msg
    msg = PointCloud2()

    msg.header = std_msgs.msg.Header()
    msg.header.frame_id = rospy.get_param("~global_pcd_reference_frame")
    msg.header.stamp = rospy.Time.now()

    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]

    msg.is_bigendian = False
    msg.is_dense = True

    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width

    msg.data = np.asarray(points, np.float32).tobytes()

    # publish cloud at 1 hz
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        global_point_cloud_pub.publish(msg)
        rate.sleep()

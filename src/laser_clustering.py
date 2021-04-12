#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan, PointCloud
import laser_geometry.laser_geometry as lg
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose, Point

import math

import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
import matplotlib.pyplot as plt
#from sklearn.datasets import make_blobs

import roslib; roslib.load_manifest('laser_assembler')
from laser_assembler.srv import *

import tf2_ros
import tf2_py as tf2
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import cv2 as cv



class laserClustering(object):

    def __init__(self):

        laser_topic = rospy.get_param('~input_scan_topic', 'scanfront')
        self.dist_from_obs = rospy.get_param('~dist_from_obs', 0.1)
        self.bandwidth = rospy.get_param('~cluster_bandwidth', 0.5)

        rospy.loginfo("---laser clustering initiated!---")
        rospy.loginfo(" - scan topic: %s", laser_topic)
        rospy.loginfo(" - dist from obstacles: %.2f m", self.dist_from_obs)
        rospy.loginfo(" - cluster bandwith: %.2f m\n", self.bandwidth)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.lp = lg.LaserProjection()

        #self.point_lists = np.array()

        self.Freespace = 0
        self.Unknown = -1
        self.Occupied = 100

        self.bandwidth = 0.8
        self.ms = MeanShift(bandwidth=self.bandwidth, bin_seeding=True, min_bin_freq=3, cluster_all=False) #min_bin_freq=1

        self.use_static_map = False

        if self.use_static_map:
            self.static_map = OccupancyGrid
            rospy.loginfo("Waiting to receive the static map...")
            rospy.wait_for_service('static_map')
            try:
                get_map = rospy.ServiceProxy('static_map', GetMap)
                self.static_map = get_map().map
                self.dist_trans = self.computeDistanceTransform()
                rospy.loginfo("I got the static map")

            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s", e)

        self.pc_pub = rospy.Publisher('my_pc2', PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher('obstacles', MarkerArray, queue_size=1)
        self.marker2_pub = rospy.Publisher('points', MarkerArray, queue_size=1)
        rospy.Subscriber(laser_topic, LaserScan, self.laser_cb2, queue_size=1)


    def computeDistanceTransform(self):
        size = self.static_map.info.height, self.static_map.info.width, 1
        map_image = np.zeros(size, dtype=np.uint8)

        for i in range(len(self.static_map.data)-1):
            if(self.static_map.data[i]==self.Freespace):
                cell = self.indexToCell(i)
                map_image[cell[0], cell[1]] = 255
        try:
            dist_trans = cv.distanceTransform(map_image, cv.DIST_L1, 3)
        except:
            rospy.logerr("Error obtaining the distance transform!!!")
        return dist_trans



    def indexToCell(self, idx):
        index = idx
        if (index >= (self.static_map.info.width * self.static_map.info.height)):
            index = (self.static_map.info.width * self.static_map.info.height) - 1;

        x = index % self.static_map.info.width
        y = index // self.static_map.info.width
        cell = [x, y]
        return cell

    def pointToCell(self, p):
        # We account for no rotation of the map
        x = math.floor((p.point.x - self.static_map.info.origin.position.x) / self.static_map.info.resolution);
        y = math.floor((p.point.y - self.static_map.info.origin.position.y) / self.static_map.info.resolution);
        return x, y

    def pointToIndex(self, p):

        # We account for no rotation of the map
        #x = math.floor((p.point.x - self.static_map.info.origin.position.x) / self.static_map.info.resolution);
        #y = math.floor((p.point.y - self.static_map.info.origin.position.y) / self.static_map.info.resolution);
        x,y = self.pointToCell(p)

        # Cell to index
        if (x >= self.static_map.info.width):
            x = (self.static_map.info.width - 1)

        if (y >= self.static_map.info.height):
            y = (self.static_map.info.height - 1)
        # Get the index in the array
        index = x + y * self.static_map.info.width
        return index



    def valid_nhood4(self, idx):
        # get 4-connected neighbourhood indexes, check for edge of map
        if (idx > self.static_map.info.width * self.static_map.info.height - 1):
            return False

        if self.static_map.data[idx] == self.Occupied or self.static_map.data[idx] == self.Unknown:
            return False

        if (idx % self.static_map.info.width > 0):
            if self.static_map.data[idx-1] == self.Occupied or self.static_map.data[idx-1] == self.Unknown:
                return False
        if ((idx % self.static_map.info.width) < self.static_map.info.width - 1):
            if self.static_map.data[idx+1] == self.Occupied or self.static_map.data[idx+1] == self.Unknown:
                return False
        if (idx >= self.static_map.info.width):
            if self.static_map.data[idx - self.static_map.info.width] == self.Occupied or self.static_map.data[idx - self.static_map.info.width] == self.Unknown:
                return False
        if (idx < self.static_map.info.width * (self.static_map.info.height - 1)):
            if self.static_map.data[idx + self.static_map.info.width] == self.Occupied or self.static_map.data[idx + self.static_map.info.width] == self.Unknown:
                return False

        return True



    def valid_nhood8(self, idx):
        # get 8-connected neighbourhood indexes, check for edge of map
        if self.valid_nhood4(idx) == False:
            return False

        if (idx % self.static_map.info.width > 0 and idx >= self.static_map.info.width):
            if self.static_map.data[idx - 1 - self.static_map.info.width] == self.Occupied or self.static_map.data[idx - 1 - self.static_map.info.width] == self.Unknown:
                return False
        if (idx % self.static_map.info.width > 0 and idx < self.static_map.info.width * (self.static_map.info.height - 1)):
            if self.static_map.data[idx - 1 + self.static_map.info.width] == self.Occupied or self.static_map.data[idx - 1 + self.static_map.info.width] == self.Unknown:
                return False
        if (idx % self.static_map.info.width < self.static_map.info.width - 1 and idx >= self.static_map.info.width):
            if self.static_map.data[idx + 1 - self.static_map.info.width] == self.Occupied or self.static_map.data[idx + 1 - self.static_map.info.width] == self.Unknown:
                return False
        if (idx % self.static_map.info.width < self.static_map.info.width - 1 and idx < self.static_map.info.width * (self.static_map.info.height - 1)):
            if self.static_map.data[idx + 1 + self.static_map.info.width] == self.Occupied or self.static_map.data[idx + 1 + self.static_map.info.width] == self.Unknown:
                return False

        return True


    def publish_points(self, points):
        ma = MarkerArray()
#        m = Marker()
#        m.header.stamp = rospy.Time()
#        m.header.frame_id = points[0].header.frame_id
#        m.type = 2 # 8-points, 7-sphere list
#        m.action = 0 #0-add
#        m.ns = 'points'
#        m.scale.x = 0.1
#        m.scale.y = 0.1
#        m.scale.z = 0.1
#        m.color.r = 0.0
#        m.color.g = 0.0
#        m.color.b = 1.0
#        m.color.a = 1.0
#        m.frame_locked = False
#        #m.lifetime
#        m.pose = Pose()
#        m.pose.orientation.w = 1.0
        #point = Point()
        #point.z = 0.1
        i = 1
        #print("\npoints: %d" % len(points))
        for p in points:
            #point.x = p.point.x
            #point.y = p.point.y
            #m.points.append(point)
            m = Marker()
            m.header.stamp = rospy.Time()
            m.header.frame_id = points[0].header.frame_id
            m.type = 2 # 8-points, 7-sphere list
            m.action = 0 #0-add
            m.ns = 'points'
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.frame_locked = False
            m.lifetime = rospy.Duration.from_sec(0.25)
            m.pose = Pose()
            m.pose.orientation.w = 1.0
            m.pose.position.x = p.point.x
            m.pose.position.y = p.point.y
            m.id = i
            #print(i)
            i = i+1
            ma.markers.append(m)

        #print("size of m.points: %d" % len(m.points))
        self.marker2_pub.publish(ma)


    def publish_centers(self, points, frame):
        ma = MarkerArray()
        i = 1
        for p in points:
            m = Marker()
            m.header.stamp = rospy.Time()
            m.header.frame_id = frame
            m.type = 3 # 8-points, 7-sphere list
            m.action = 0 #0-add
            m.ns = 'centers'
            m.scale.x = 0.35
            m.scale.y = 0.35
            m.scale.z = 1.5
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.frame_locked = False
            m.lifetime = rospy.Duration.from_sec(0.25)
            m.pose = Pose()
            m.pose.orientation.w = 1.0
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.id = i
            i = i+1
            ma.markers.append(m)
        self.marker_pub.publish(ma)


    def laser_cb2(self, msg):
        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = self.lp.projectLaser(msg)

        try:
            trans = self.tf_buffer.lookup_transform('odom', msg.header.frame_id,
                                                      msg.header.stamp,
                                                      rospy.Duration(0.01))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        try:
            pc2_msg = do_transform_cloud(pc2_msg, trans)
        except:
            rospy.logerr("Could not transform point cloud")

        points = np.asarray(pc2.read_points_list(pc2_msg, field_names = ("x", "y"), skip_nans=False))
        rospy.loginfo(" Range: %d", len(msg.ranges))
        rospy.loginfo(" scanned points size: %d", len(points))






    def laser_cb(self, msg):

        # convert the message of type LaserScan to a PointCloud2
        pc2_msg = self.lp.projectLaser(msg)

        #self.pc_pub.publish(pc2_msg)

        # what is faster? -> 2!
        # 1) list
        #point_list = np.asarray(list(pc2.read_points(pc2_msg, field_names = ("x", "y"), skip_nans=True)))
        # or 2) - a list of the individual points which is less efficient
        #print(pc2.read_points_list(pc2_msg, field_names = ("x", "y"), skip_nans=True))
        point_list = np.asarray(pc2.read_points_list(pc2_msg, field_names = ("x", "y"), skip_nans=True))
        #print(point_list)

        p = PointStamped()
        p.header.frame_id = pc2_msg.header.frame_id
        p.header.stamp = rospy.Time()
        filtered_pl = []
        point_stamped = []
        for i in point_list:
            p.point.x = i[0]
            p.point.y = i[1]
            try:
                pt = self.tf_buffer.transform(p, 'map')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("Could not transform point to map: %s"%e)
                return
            x,y = self.pointToCell(pt)
            distance = self.dist_trans[x, y] * self.static_map.info.resolution
            if distance > self.dist_from_obs:
                filtered_pl.append(i)
                point_stamped.append(pt)


        filtered_pl = np.asarray(filtered_pl)

        if len(point_stamped) > 0:
            self.publish_points(point_stamped)
        # The following bandwidth can be automatically detected using
        #bandwidth = estimate_bandwidth(point_list, quantile=0.2, n_samples=500)

        #ms = MeanShift(bandwidth=bandwidth) #bin_seeding=True
        if len(filtered_pl) > 0:
            try:
                self.ms.fit(filtered_pl)
                labels = self.ms.labels_  #with the labels I know the points for each cluster
                cluster_centers = self.ms.cluster_centers_
                self.publish_centers(cluster_centers, pc2_msg.header.frame_id)
            except:
                rospy.logwarn("No cluster obtained..")


        #self.ms = MeanShift(bandwidth=self.bandwidth, min_bin_freq=2, seeds=cluster_centers) #bin_seeding is ignored if seeds are given
        #print("mumber of cluster centers: %d" % len(cluster_centers))
        #print("centers:")
        #print(cluster_centers)

        #labels_unique = np.unique(labels)
        #n_clusters_ = len(labels_unique)
        #print("number of estimated clusters : %d" % n_clusters_)

        #P = ms.predict(point_list)

        #print("P:")
        #print(P)


        #plt.scatter(point_list[:,0], point_list[:,1], marker=".", picker=True) #c=colors
        #plt.scatter(cluster_centers[:,0], cluster_centers[:,1], marker="o", c='red')
        #plt.plot(point_list[:,0], point_list[:,1], marker=".")
        #plt.plot(point_list, marker=".")
        #plt.title(f'Estimated number of clusters = {n_clusters_}')
        #plt.xlabel('X')
        #plt.ylabel('Y')
        #plt.show()



if __name__ == '__main__':
    rospy.init_node('clusterLaser', anonymous=True)
    lc = laserClustering()
    rospy.spin()

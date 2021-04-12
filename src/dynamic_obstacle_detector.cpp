
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "obstacle_kf.hpp"

#include <math.h> /* isinf, sqrt */
#include <vector>

class DynamicObstacleDetector {

public:                                                   
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  sensor_msgs::PointCloud pre_cloud_;
  ros::Subscriber scan_sub_;
  ros::Publisher obs_pub_, dyn_obs_pub_, points_pub_;
  std::string input_scan_topic_;
  std::string odom_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  laser_geometry::LaserProjection projector_;


  std::vector<ObstacleKF> tracked_obstacles_;


  int scan_buffer_size_;
  float thres_point_dist_; 
  int thresh_min_points_;
  int thresh_max_points_;
  float min_vel_tracked_;
  float max_vel_tracked_;

  //for the KF
  float track_distance_;
  float track_timeout_;
  int obstacle_count_;


  //TODO: publish trajectory of trackedObs


  struct Point {
    int id;
    float x;
    float y;
    Point()
    {
      id = -1;
      x = 0.0;
      y = 0.0;
    }
  };

  struct Obstacle
  {
    int id;
    ros::Time t;
    Point center;
    float width;
    int seen;
    bool matched;
    std::vector<Point> points;
    Obstacle()
    {
      t=ros::Time(0);
      id=-1;
      width = 0.0;
      seen = 1;
      matched = false;
    }

  };

  struct TrackedObstacle
  {
    //Obstacle obstacle;
    int id;
    std::vector<Point> traj;
    std::vector<ros::Time> time;
  };

  //std::vector<Obstacle> prev_obs_;

  std::vector<std::vector<Point>> points_;
  std::vector<std::vector<Obstacle>> obstacles_;



  DynamicObstacleDetector() : nh_("~"), n_(), tf_listener_(tf_buffer_) {

    nh_.param("input_scan_topic", input_scan_topic_, std::string("scan"));
    nh_.param("odom_frame", odom_frame_, std::string("odom"));
    nh_.param("scan_buffer_size", scan_buffer_size_, 3);
    nh_.param("cluster_max_distance_points", thres_point_dist_, float(0.6));
    nh_.param("cluster_min_points", thresh_min_points_, 3);
    nh_.param("cluster_max_points", thresh_max_points_, 35);
    nh_.param("min_vel_tracked", min_vel_tracked_, float(0.35));
    nh_.param("max_vel_tracked", max_vel_tracked_, float(2.0));

    nh_.param("max_tracked_distance", track_distance_, float(0.55)); 
    nh_.param("max_tracked_sec", track_timeout_, float(1.0));


    obstacle_count_ = 0;

    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>(
        input_scan_topic_.c_str(), 1, &DynamicObstacleDetector::scan_cb, this);

    obs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/static", 0);
    dyn_obs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/dynamic_obstacles/dynamic", 1);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("/dynamic_obstacles/points", 0);
    
  }


  ~DynamicObstacleDetector() {}


  void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg) 
  {
    std::vector<Point> points;
    float angle_min = msg->angle_min;
    float angle = angle_min + (msg->angle_increment / 2.0);

    // Transform scan to a set of points in odom_frame_
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = msg->header.frame_id;
    ps.header.stamp = ros::Time(0);
    for (unsigned int i = 0; i < msg->ranges.size(); i++) 
    {
      if (!isinf(msg->ranges[i]) && !isnan(msg->ranges[i]))
      {
        Point p;
        p.id = i + 1;
        ps.point.x = msg->ranges[i] * cos(angle);
        ps.point.y = msg->ranges[i] * sin(angle);
        ps.point.z = 0.0;
        geometry_msgs::PointStamped psn;
        try {
          psn = tf_buffer_.transform(ps, odom_frame_);
        } catch (tf2::TransformException &ex) {
          ROS_WARN("Could NOT transform point to %s: %s", odom_frame_.c_str(), ex.what());
          return;
        }
        p.x = psn.point.x;
        p.y = psn.point.y;
        points.push_back(p);
      }
      angle = angle + msg->angle_increment;
    }

    if(!points.empty())
    {
      //Publish the points in RViz
      publish_points(points);
      //Find obstacles candidates in the point set
      std::vector<Obstacle> obs = findObs(points, msg->header.stamp);
      //printf("found %i obstacles!\n", (int)obs.size());
      publish_obs(obs, std::string("obstacles"), 2, obs_pub_, 0.1);
      //Track with the KFs
      trackMovingObstaclesKF(obs);
    }

  }

  

  void trackMovingObstaclesKF(std::vector<Obstacle> &obs)
  {
    //printf("\nNew obstacles size: %i\n\n", (int)obs.size());
    // Vector with flags to consider when a detection have been used
		std::vector<bool> used(obs.size(), false);

    double posDev = 0.2; //0.1; //laser


    // Update tracked obstacles with the detections according to min distance
		int k;
		double x1, y1, x2, y2, dist, minDist;
		for(int i=0; i<(int)tracked_obstacles_.size(); i++)
		{
			//if(tracked_obstacles_[i].updated)
			//	continue;
				
			// Search the closest detection to every person tracked
			k = 0;
			x1 = tracked_obstacles_[i].x(0,0);
			y1 = tracked_obstacles_[i].x(1,0);
			minDist = 1000000.0;
			for(int j=0; j<(int)obs.size(); j++)
			{	
				if(!used[j])
				{
					x2 = obs[j].center.x;
					y2 = obs[j].center.y;
					dist = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
					if(dist < minDist)
					{
						k = j;
						minDist = dist;
					}
				}
			}
			
			// Check if the distant is too high
			if(minDist < track_distance_)
			{
				// Predict the position of the obstacle to this instant
				tracked_obstacles_[i].predict((ros::Time::now()-tracked_obstacles_[i].tStamp).toSec());
			
				// Update the filter
				tracked_obstacles_[i].update(obs[k].center.x, obs[k].center.y, posDev, ros::Time::now());
			
				// Mark the detection as used
				used[k] = true;
				//std::cout << "Done" << std::endl;
        //std::cout << "Updating obstacle " << i << " with detection " << k << std::endl;
			}
		}

    // Add new persons to the list
		for(int i=0; i<(int)obs.size(); i++)
		{
			if(!used[i]) 
			{
				//std::cout << "New obstacle added to the list: " << tracked_obstacles_.size() << std::endl;
				ObstacleKF obstacle;
				obstacle.init(obstacle_count_++, obs[i].center.x, obs[i].center.y, 0, 0);
				tracked_obstacles_.push_back(obstacle);
        //std::cout << "New obstacle added! size: " << tracked_obstacles_.size() << std::endl;
				
				used[i] = true;
			}
		}
		
		// Remove too old estimations without update and static obstacles
		std::vector<ObstacleKF> temp;
		for(int i=0; i<(int)tracked_obstacles_.size(); i++)
		{

      //printf("obstacle %i, linvel: %.3f\n", i, linvel);
			if((ros::Time::now()-tracked_obstacles_[i].tStamp).toSec() < track_timeout_) //&& linvel >= min_vel_tracked_
				temp.push_back(tracked_obstacles_[i]);
		}

		tracked_obstacles_.clear();
		tracked_obstacles_ = temp;
		
		// Publish dynamic obstacles
		publishDynObsMarker();
  
  }



  void publishDynObsMarker()
	{
		visualization_msgs::MarkerArray obsMarkers;
		visualization_msgs::Marker marker;

		// Get marker color
		float r, g, b;
		r = 0.0;
		g = 1.0;
		b = 0.0;
			
		marker.header.frame_id = odom_frame_;
		marker.header.stamp = ros::Time::now();
		marker.ns = ros::this_node::getName();
		for(int i=0; i<(int)tracked_obstacles_.size(); i++)
		{
			if(tracked_obstacles_[i].updated)
			{
        double time = (ros::Time::now()-tracked_obstacles_[i].tStamp).toSec();
        double vx = tracked_obstacles_[i].x(2,0);
        double vy = tracked_obstacles_[i].x(3,0);
        double linvel = sqrt((vx*vx)+(vy*vy));
        double x_arrow = vx * time;
        double y_arrow = vy * time;
        double yaw = atan2(vy, vx);
        //printf("Tracked obs %i, linvel: %.3f\n", tracked_obstacles_[i].id, linvel);
        //printf("tracked ob %i linvel: %.3f, min_vel_tracked: %.3f\n", tracked_obstacles_[i].id, linvel, min_vel_tracked_);
      
        if(linvel >= min_vel_tracked_)
        {
          // Cylinder
          marker.id = 20*tracked_obstacles_[i].id;
          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = tracked_obstacles_[i].x(0,0);
          marker.pose.position.y = tracked_obstacles_[i].x(1,0);
          marker.pose.position.z = 0.5;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 0.25;
          marker.scale.y = 0.25;
          marker.scale.z = 1.0;
          marker.color.a = 1.0; 
          marker.color.r = r;
          marker.color.g = g;
          marker.color.b = b;
          marker.lifetime = ros::Duration(0.1);
          obsMarkers.markers.push_back(marker);

          // Arrow
          marker.id = 20*tracked_obstacles_[i].id+1;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = tracked_obstacles_[i].x(0,0);
          marker.pose.position.y = tracked_obstacles_[i].x(1,0);
          marker.pose.position.z = 0.5;
          //marker.points.push_back(marker.pose.position);
          //geometry_msgs::Point end;
          //end.x = x_arrow;
          //end.y = y_arrow;
          //end.z = marker.pose.position.z;
          //marker.points.push_back(end);
          marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          //marker.pose.orientation.x = 0.0;
          //marker.pose.orientation.y = 0.0;
          //marker.pose.orientation.z = 0.0;
          //marker.pose.orientation.w = 1.0;
          marker.scale.x = 1.2 * (linvel/max_vel_tracked_);
          marker.scale.y = 0.15;
          marker.scale.z = 0.15;
          obsMarkers.markers.push_back(marker);
        }
         
			}
			// else
			// {
			// 	// Delete body marker
			// 	marker.id = tracked_obstacles_[i].id;
			// 	marker.type = visualization_msgs::Marker::CYLINDER;
			// 	marker.action = visualization_msgs::Marker::DELETE;
			// 	obsMarkers.markers.push_back(marker);
			// }
		}
		// publish marker:
		dyn_obs_pub_.publish(obsMarkers);
	}




  std::vector<Obstacle> findMovingObs3()
  {
    //use the obs of the first scan as an initial set of tracked obs
    std::vector<TrackedObstacle> tobs;
    int id = 1;
    for(unsigned int i=0; i<obstacles_[0].size(); i++)
    {
      TrackedObstacle to;
      to.id = id;
      to.traj.push_back(obstacles_[0][i].center);
      to.time.push_back(obstacles_[0][i].t);
      tobs.push_back(to);
      id++;
    }
    //printf("initial set of obstacles: %i\n", (int)tobs.size());

    //Look for the obstacles in the next scan and update the trajectory
    float min_total_dist = 0.0;
    float max_total_dist = 0.0;
    for(unsigned int c=1; c<scan_buffer_size_; c++)
    {
      float dt = (obstacles_[c][0].t - obstacles_[c-1][0].t).toSec();
      float max_dist_step = max_vel_tracked_ * dt;
      float min_dist_step = min_vel_tracked_ * dt;
      min_total_dist += min_dist_step;
      max_total_dist += max_dist_step;
      for(unsigned int i=0; i < tobs.size(); i++)
      {
        TrackedObstacle* candidate = &tobs[i];
        Obstacle min;
        float min_dist=1000;
        for(unsigned int j=0; j < obstacles_[c].size(); j++)
        {
            float d = dist(candidate->traj[c-1], obstacles_[c][j].center);
            if(d < min_dist)
            {
              min_dist = d;
              min = obstacles_[c][j];
            }
        }
        //printf("TrackedOb %i with ob %i. min_dist: %.2f\n", candidate->id, min.id, min_dist);
        if(min_dist > min_dist_step && min_dist < max_dist_step) 
        {
          candidate->traj.push_back(min.center);
          candidate->time.push_back(obstacles_[c][0].t);
        }
      }
    }
    std::vector<Obstacle> movingobs;
    for(TrackedObstacle tracked : tobs)
    {
      
      //if not tracked in all (or nearly all) the scans, discard it
      if(tracked.traj.size() < scan_buffer_size_-1)
      {
        continue;
      }
      //printf("\nTracked ob: %i. traj size: %i\n", tracked.id, (int)tracked.traj.size());
      //printf("\tdist: %.3f", dist(tracked.traj[0] , tracked.traj[tracked.traj.size()-1]));

      //check the trajectory is not noise
      if(dist(tracked.traj[0] , tracked.traj[tracked.traj.size()-1]) >= min_total_dist)
      {
        Obstacle o;
        o.id = tracked.id;
        o.center = tracked.traj[tracked.traj.size()-1];
        movingobs.push_back(o);
      } 
    }
    return movingobs;
  }




  std::vector<Obstacle> findObs(const std::vector<Point>& points, ros::Time t)
  {
    std::vector<Obstacle> obs;
    unsigned int id = 1;
    unsigned int i = 0;
    while(i<points.size()-1)
    {

      Obstacle o;
        
      //Joint the points
      o.points.push_back(points[i]);
      while((i+1) < points.size() && dist(points[i], points[i+1]) <= thres_point_dist_)
      {
        o.points.push_back(points[i+1]);
        i++;
      }
      //printf("Found a candidate group of %i points.", (int)o.points.size());

      //Create the obstacle if the conditions are fulfilled
      if((int)o.points.size() > thresh_min_points_ && (int)o.points.size() < thresh_max_points_)
      {
        //printf("Found a candidate group of %i points.", (int)o.points.size());
        //printf(" --> ACCEPTED!\n");
        //Create obstacle
        o.id = id;
        o.t = t;
        o.width = dist(o.points[0], o.points[o.points.size()-1]);
        o.center.id = id;
        id++;
        o.seen = 1;
        for(Point p : o.points)
        {
          o.center.x += p.x;
          o.center.y += p.y;
        }
        o.center.x /= (float)o.points.size();
        o.center.y /= (float)o.points.size();
        obs.push_back(o);
        
      } 
      else{
        //printf(" --> DISCARDED!\n");
      }
      i++;
      //remove the point candidates
      o.points.clear();
    }
    return obs;
  }




  float dist(const Point &p1, const Point &p2)
  {
    //printf("p1x:%.2f, p2x:%.2f, p1y:%.2f, p2y:%.2f, -Dist: %.3f\n",p1.x, p2.x, p1.y, p2.y, std::hypotf((p1.x - p2.x), (p1.y - p2.y)));
    return std::hypotf((p1.x - p2.x), (p1.y - p2.y));
  }


  void publish_obs(const std::vector<Obstacle> &obs, std::string namespc, int color, ros::Publisher &pub, double time)
  {
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = odom_frame_;
    m.header.stamp = ros::Time();
    m.ns = namespc;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.4;
    switch (color)
    {
      case 0:
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        break;
      case 1:
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        break;
      case 2:
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        break;
      default:
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        break;
    }
    m.color.a = 1.0; 
    m.lifetime = ros::Duration(time);
    for(Obstacle o : obs)
    {
      m.id = o.id;
      m.pose.position.x = o.center.x;
      m.pose.position.y = o.center.y;
      m.pose.position.z = 0.3;
      ma.markers.push_back(m);
    }
    pub.publish(ma);
  }


  void publish_points(const std::vector<Point> &points)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = odom_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = "dyn_points";
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;
    m.scale.y = 0.03;
    m.scale.z = 0.03;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0; 
    m.id = points[0].id;
    m.lifetime = ros::Duration(0.1);
    for(Point p : points)
    {
      //if(p.id == -1)
      //  continue;
      geometry_msgs::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.03;
      m.points.push_back(pt);
    }
    points_pub_.publish(m);
  }

  // void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg) {
  //   // sensor_msgs::LaserScan new_scan;
  //   sensor_msgs::PointCloud new_cloud;

  //   try {
  //     geometry_msgs::TransformStamped transformStamped =
  //         tf_buffer_.lookupTransform(
  //             msg->header.frame_id, "odom",
  //             msg->header.stamp + ros::Duration().fromSec(msg->ranges.size() *
  //                                                         msg->time_increment),
  //             ros::Duration(1.0));
  //   } catch (tf2::TransformException &ex) {
  //     ROS_WARN("Could NOT transform between %s and odom: %s",
  //              msg->header.frame_id.c_str(), ex.what());
  //     return;
  //   }

  //   sensor_msgs::PointCloud2 cloud;
  //   projector_.transformLaserScanToPointCloud("odom", *msg, cloud, tf_buffer_);

  //   sensor_msgs::convertPointCloud2ToPointCloud(cloud, new_cloud);
    
  //   if (initiated_) {
  //     // Compare pre_cloud_ and new_cloud in odom frame.
  //     printf("eoeoeooeheheheheh");
  //     new_cloud = compareCloud(pre_cloud_, new_cloud);
  //     cloud_pub_.publish(new_cloud);
  //   }
  //   // pre_scan_ = new_scan;
  //   pre_cloud_ = new_cloud;
  //   initiated_ = true;
  // }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "dynamic_obstacle_detector");
  DynamicObstacleDetector node;
  ros::spin();
  return 0;
}

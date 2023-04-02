#ifndef _FRONTIER_DETETCTOR_H_
#define _FRONTIER_DETECTOR_H_

#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "frontier_exploration/PointArray.h"
#include "frontier_exploration/get_centroids.h"
#include "std_srvs/Empty.h"

namespace FrontierDetector{

    struct Header_param{

        std::string frame_id = "map";
        ros::Time stamp = ros::Time::now(); 
    };

    class FrontierDetector{
        private:
            float OBSTABLE_INFLATION;  // Inflate obstacles in meters
            float MapRevolution;  // map revolution (m/cell)
            nav_msgs::OccupancyGrid raw_map;
            frontier_exploration::PointArray Frontier;
            frontier_exploration::PointArray Centroids;
            frontier_exploration::get_centroids::Request req;
            frontier_exploration::get_centroids::Response res;
            visualization_msgs::Marker frontier_vis;
            visualization_msgs::Marker centroid_vis;
            std::vector<geometry_msgs::Point> raw_centroids;

            std::vector<geometry_msgs::Point> pointGroup; // Frontier group
            std::vector<geometry_msgs::Point> frontierClose; // checked over frontier
            Header_param header;

            bool CheckCollision(const nav_msgs::OccupancyGrid& map, 
                                geometry_msgs::Point& start, geometry_msgs::Point& end);
        public:
            std::vector<geometry_msgs::Point> centroids;
            std::vector<geometry_msgs::Point> frontier;
            nav_msgs::OccupancyGrid inflated_map;
            
            ros::Publisher frontierMarker_;
            ros::Publisher centroidMarker_;
            ros::Publisher FrontierPub_;
            ros::Publisher CentroidsPub_;
            ros::Publisher inflatedMapPub_;

            ros::Subscriber MapSub_;

            ros::ServiceServer CentriodServer_;
            ros::ServiceClient CentriodClient_;

            FrontierDetector(ros::NodeHandle& nh);
            ~FrontierDetector();
            int GridValue(nav_msgs::OccupancyGrid &map,geometry_msgs::Point& x1);   // Get specific grid value
            int CheckNeibor(nav_msgs::OccupancyGrid &inflated_map,long& index); // Check neibor cell

            bool InflateMap(nav_msgs::OccupancyGrid& raw_map, nav_msgs::OccupancyGrid& inflated_map); // Inflate grid map
            bool ComputeFrontier(nav_msgs::OccupancyGrid& Inflated_map);    // Get all frontiers in current map
            bool ComputeCentroids(nav_msgs::OccupancyGrid &inflated_map,std::vector<geometry_msgs::Point>& frontiers);  // Get the centroids from frontiers
            void InitDetector();    // Initialization
            void InitVis(); // For display fronteir and centroids
            void Visualization();
            void Grouping(nav_msgs::OccupancyGrid& inflated_map,geometry_msgs::Point& point);   // Grouping current frontiers
            void mapCallback(const nav_msgs::OccupancyGridConstPtr& raw_map);
            bool centroidCallback(frontier_exploration::get_centroids::Request &req,
                                  frontier_exploration::get_centroids::Response &res);

            std::vector<geometry_msgs::Point> Sort(nav_msgs::OccupancyGrid& inflated_map,std::vector<geometry_msgs::Point>& pts);   // Sorting according to the index

    };

}

#endif
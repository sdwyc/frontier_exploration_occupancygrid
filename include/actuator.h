#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <tf/transform_listener.h>
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "frontier_exploration/PointArray.h"
const double PI=3.1415926535897932385;
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace Actuator{
    struct Header_param{

        std::string frame_id = "map";
        ros::Time stamp = ros::Time::now(); 
    };

    struct RobotPose{
        geometry_msgs::Point Position;
        double Yaw;
    };

    struct CurrentGoal{
        string id = "0";
        ros::Time stamp;
    };
    // Actuator class function: Publish a current goal selected from frontiers
    class Actuator{
        private:
            string CmdTopic;
            string RobotBase;
            float GoalTolerance;   // Paramter: Abandon next goal close to explored point
            float ObstacleTolerance; // Paramter: Close to obstacle no rotate
            float RotateSpeed;
            long iteration;

            Header_param header;
            vector<geometry_msgs::Point> centroids; // The centroids of all frontiers
            geometry_msgs::Point Home;  // The mapping origin
            visualization_msgs::Marker GoalMarker;
            visualization_msgs::Marker HomeMarker;
            visualization_msgs::Marker path_vis;
            nav_msgs::OccupancyGrid inflated_map;
            nav_msgs::OccupancyGrid raw_map;

            geometry_msgs::Twist RotSpeed;
            RobotPose robotPose;    // Robot current pose through TF

            move_base_msgs::MoveBaseGoal MoveGoal;  
            
            ros::Publisher GoalPub;
            ros::Publisher CancelgoalPub;
            ros::Publisher cmdPub;
            ros::Publisher goal_vis;
            ros::Publisher home_vis;
            ros::Publisher path_pub;
            ros::Subscriber centroidsSub;
            ros::Subscriber InflatedMapSub_;
            ros::Subscriber RawMapSub_;
            tf::TransformListener PoseListner;

        public:
            MoveBaseClient ac;
            geometry_msgs::Point Goal;
            std::vector<geometry_msgs::Point> GoalClose;  // restore traveled or unavailable points
                                                          // Avoid all centriods in GoalClose
            int GoHomeFlag; // The flag of robot return home
            Actuator(ros::NodeHandle& nh);
            Actuator();
            ~Actuator();
            
            geometry_msgs::Point SelectGoal(std::vector<geometry_msgs::Point>& centroids);
            void ObtainPose();  // Obtain robot current pose and yaw
            void MoveToGoal();  // Going to the given goal through action
            void CancelGoal();  // Cancel current goal
            void Rotation(float angle);  // Robot will robot certain angle after reached the goal
            void ReturnHome();  // After exploration, robot will go back origin point
            void Visualization();   // Visualize all markers
            void VisInit(); // Marker Initialization
            void ActuatorInit();    // Initialization of all vars
            void AddToClose(geometry_msgs::Point& goal);    // Add current goal to the close_list set
            void centroidCallback(const frontier_exploration::PointArrayConstPtr& msg);
            // void inflatedmapCallback(const nav_msgs::OccupancyGridConstPtr& inflateMap);
            void mapCallback(const nav_msgs::OccupancyGridConstPtr& RawMap);    // Raw map CB

            int CheckCollision(const nav_msgs::OccupancyGrid& map, 
                                       geometry_msgs::Point& start, geometry_msgs::Point& end); // Lazy collision checking method

    };
}

#endif
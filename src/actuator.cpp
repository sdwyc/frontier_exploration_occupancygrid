#include <time.h>
#include <math.h>
#include "actuator.h"

using namespace std;

void Actuator::Actuator::centroidCallback(const frontier_exploration::PointArrayConstPtr& msg){
    centroids = msg->points;
    Visualization();
}

void Actuator::Actuator::mapCallback(const nav_msgs::OccupancyGridConstPtr& RawMap){
    raw_map = *RawMap;
}

void Actuator::Actuator::Rotation(float angle){

    ObtainPose(); // Obtain robot's current pose
    // When close to obstacle, robot cannot rotate
    // Frame transformation: map -> image
    int world_x = (robotPose.Position.x+fabs(raw_map.info.origin.position.x))/raw_map.info.resolution;
    int world_y = (robotPose.Position.y+fabs(raw_map.info.origin.position.y))/raw_map.info.resolution;
    int CheckXmin = int(world_x-round(ObstacleTolerance/raw_map.info.resolution)) , CheckXmax = int(world_x+round(ObstacleTolerance/raw_map.info.resolution));
    int CheckYmin = int(world_y-round(ObstacleTolerance/raw_map.info.resolution)) , CheckYmax = int(world_y+round(ObstacleTolerance/raw_map.info.resolution));
    // Check Neibors
    for(int x=CheckXmin;x<=CheckXmax;x++){
        for(int y=CheckYmin;y<=CheckYmax;y++){
            if(x<0 || y<0 || x>raw_map.info.width || y>raw_map.info.height){continue;}
            if(raw_map.data[x+(y*raw_map.info.width)] >=70){
                cout << "Position close to obstacle. Cannot rotate" << endl;
                return;
            }
        }
    }

    double rotated_angle = 0.0; // Rotated angles 
    time_t initTime, currTime;
    time_t duration = 0L;
    time_t limitation = 15L;  // Rotation time within 15 second
    time(&initTime);

    while((rotated_angle < angle) && (ros::ok()) && bool(duration<limitation)){
        time(&currTime);
        duration = currTime-initTime;
        double old_yaw = robotPose.Yaw;
        cmdPub.publish(RotSpeed);
        ObtainPose();
        ros::spinOnce();
        double dYaw = robotPose.Yaw - old_yaw;
        if((robotPose.Yaw>=0 && robotPose.Yaw<350) && (old_yaw>=350 && old_yaw< 360)){
            dYaw = robotPose.Yaw + (360.0-old_yaw);
        }
        rotated_angle += dYaw;
    }
    duration = 0;
}

void Actuator::Actuator::CancelGoal(){
    ac.cancelGoal();
}

void Actuator::Actuator::ReturnHome(){

        Goal = Home;
        MoveToGoal();  
}

void Actuator::Actuator::MoveToGoal(){
    // Movebase action method
    MoveGoal.target_pose.pose.position = Goal;
    ac.sendGoal(MoveGoal);
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        cout << "Reached the goal!" << endl;
    }
    
}

void Actuator::Actuator::Visualization(){
    GoalMarker.points.clear();
    HomeMarker.points.clear();
    GoalMarker.pose.position = Goal;
    HomeMarker.pose.position = Home;
    // GoalMarker.points.push_back(Home);
    goal_vis.publish(GoalMarker);
    home_vis.publish(HomeMarker);
}

void Actuator::Actuator::VisInit(){
    //! Goal is pink
    GoalMarker.header.frame_id=header.frame_id;
    GoalMarker.header.stamp = header.stamp;
    GoalMarker.ns = "goal";
    GoalMarker.id = 4;
    GoalMarker.lifetime = ros::Duration();
    GoalMarker.type=GoalMarker.SPHERE;
    GoalMarker.action = GoalMarker.ADD;
    GoalMarker.color.a=1.0;
    GoalMarker.color.r=0.79;
    GoalMarker.color.g=0.06;   //203, 15, 121
    GoalMarker.color.b=0.47;
    GoalMarker.scale.x=0.3;
    GoalMarker.scale.y=0.3;
    GoalMarker.scale.z=0.3;
    GoalMarker.pose.orientation.w=1.0;
    GoalMarker.points.clear();

    //* Home is green
    HomeMarker.header.frame_id=header.frame_id;
    HomeMarker.header.stamp = header.stamp;
    HomeMarker.ns = "home";
    HomeMarker.id = 5;
    HomeMarker.lifetime = ros::Duration();
    HomeMarker.type=HomeMarker.SPHERE;
    HomeMarker.action = HomeMarker.ADD;
    HomeMarker.color.a=1.0;
    HomeMarker.color.r=0.0;
    HomeMarker.color.g=1.0;   //203, 15, 121
    HomeMarker.color.b=0.0;
    HomeMarker.scale.x=0.3;
    HomeMarker.scale.y=0.3;
    HomeMarker.scale.z=0.3;
    HomeMarker.pose.orientation.w=1.0;
    HomeMarker.points.clear();
}

void Actuator::Actuator::ObtainPose(){
    tf::StampedTransform transform;
    tf::Quaternion cur_rotation;
    PoseListner.waitForTransform("map",RobotBase,ros::Time::now(),ros::Duration(0.5));
    int  temp=0;
    
    while (temp==0 && ros::ok())
    {
        try
            {
                temp=1;
                PoseListner.lookupTransform("map",RobotBase,ros::Time(0), transform);
            }
        catch (tf::TransformException& ex)
            {
                temp=0;
                cout << "Cannot Obtain robot pose!!" << endl;
                ros::Duration(0.1).sleep();
            }
        robotPose.Position.x = transform.getOrigin().x();
        robotPose.Position.y = transform.getOrigin().y();
        robotPose.Position.z = 0.0;
        cur_rotation = transform.getRotation();
        double Yaw = tf::getYaw(cur_rotation);
        if(Yaw < 0){
            Yaw = 2*PI - fabs(Yaw);
        }
        robotPose.Yaw = 180*Yaw/PI;  // Angle in Degree
        // cout << "rosbot yaw: " << robotPose.Yaw << endl;
    }
}

void Actuator::Actuator::ActuatorInit(){
    string cmd_topic = "cmd_vel";
    string base_frame = "base_link";  // Default value
    ros::param::param<std::string>("~/cmd_topic",CmdTopic,cmd_topic);
    ros::param::param<std::string>("~/robot_base_frame",RobotBase,base_frame);
    ros::param::param<float>("~/goal_tolerance",GoalTolerance,0.2);        // m
    ros::param::param<float>("~/obstacle_tolerance",ObstacleTolerance,0.5);  // m
    ros::param::param<float>("~/rotate_speed",RotateSpeed,0.5);  // m

    iteration = 0;
    GoHomeFlag = 0;
    centroids.clear();
    GoalClose.clear();
    RotSpeed.linear.x = 0.0;
    RotSpeed.linear.y = 0.0;
    RotSpeed.linear.z = 0.0;
    RotSpeed.angular.x = 0.0;
    RotSpeed.angular.y = 0.0;
    RotSpeed.angular.z = RotateSpeed;
    
    // MoveGoal.target_pose.header.frame_id = header.frame_id;
    MoveGoal.target_pose.header.frame_id = "inflated_map";
    MoveGoal.target_pose.header.stamp = header.stamp;
    MoveGoal.target_pose.pose.position.z = 0.0;
    MoveGoal.target_pose.pose.orientation.w = 1.0;
}

void Actuator::Actuator::AddToClose(geometry_msgs::Point& goal){
    // Assure current goal not in close
    if(find(GoalClose.begin(),GoalClose.end(),goal) == GoalClose.end()){
        GoalClose.push_back(goal);
    }
}

geometry_msgs::Point Actuator::Actuator::SelectGoal(std::vector<geometry_msgs::Point>& centroids){
    ObtainPose();
    int index;
    int count = 0;  // For ensure whether all centroids are in GoalClose
    double shortest=10000;
    double temp;
    if(centroids.size()==0){cout << "No centroids!  No goal!" << endl;return Home;}
    else{
        for(int i=0;i<centroids.size();i++){
            if(GoalClose.size() != 0){
                for(int n=0;n<GoalClose.size();n++){
                    float Distance = sqrt(pow((centroids[i].x-GoalClose[n].x),2)+pow((centroids[i].y-GoalClose[n].y),2));
                    if(Distance < GoalTolerance && Distance > 0.0001){
                        GoalClose.push_back(centroids[i]);  // Abandon centroid close to explored goal
                        break;
                    }
                }
                // Abandon explored goal
                if(find(GoalClose.begin(),GoalClose.end(),centroids[i])!=GoalClose.end()){count++;continue;}
            }
            
            temp = sqrt(pow((robotPose.Position.x-centroids[i].x),2)+pow((robotPose.Position.y-centroids[i].y),2));
            int collision = CheckCollision(raw_map,robotPose.Position,centroids[i]); // Collision times
            auto sigmoid = [collision]{return 2/(1+exp(-0.3*collision))-1;}; // Add obstacle punishment factor 
            temp = temp * (1+sigmoid()); // Cost function. The shorest goal are priored
            if(temp < shortest){
                shortest = temp;
                index = i;
            }
        }
        if(count == centroids.size()){GoHomeFlag = 1;return Home;} // Go home
        cout << "GoalClose List size: " << GoalClose.size() << endl;
        Goal = centroids[index];
        iteration++;
        cout << "/************************/" << endl;
        cout << "Iteration: " << iteration << ": \nGoal: "
            << Goal.x << "," << Goal.y << "\nDistance: " << shortest << endl;
        return Goal;
    }
}

int Actuator::Actuator::CheckCollision(const nav_msgs::OccupancyGrid& map, 
                                       geometry_msgs::Point& start, geometry_msgs::Point& end){
    float length = sqrt(pow((start.x-end.x),2)+pow((start.y-end.y),2));
    float COS_THETA = (end.x - start.x)/length;
    float SIN_THETA = (end.y - start.y)/length;
    float resolution = map.info.resolution;
    float STEP = resolution;
    int count=0;

    float x_check = start.x;
    float y_check = start.y;
    // Segment checking
    while (fabs(x_check-end.x)>STEP && fabs(y_check-end.y)>STEP){
        int x_check_world=(x_check+fabs(map.info.origin.position.x))/map.info.resolution;
        int y_check_world=(y_check+fabs(map.info.origin.position.y))/map.info.resolution;
        if(map.data[x_check_world+(y_check_world*map.info.width)] > 65){
            count++;
        }
        x_check+=STEP*COS_THETA;
        y_check+=STEP*SIN_THETA;
    }
    return count;
}

Actuator::Actuator::Actuator(ros::NodeHandle& nh):
GoalPub(nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal",1000)),
CancelgoalPub(nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1000)),
goal_vis(nh.advertise<visualization_msgs::Marker>("goal_vis",1000)),
home_vis(nh.advertise<visualization_msgs::Marker>("home_vis",1000)),
path_pub(nh.advertise<visualization_msgs::Marker>("path_vis",1000)),
centroidsSub(nh.subscribe("centroids",10,&Actuator::centroidCallback,this)),
RawMapSub_(nh.subscribe("map",10,&Actuator::mapCallback,this)),
ac("move_base",true)
{
    ActuatorInit();
    VisInit();
    ObtainPose();
    Home = robotPose.Position;
    Goal = Home;
    cout << "Home pose: " << Home.x << "," << Home.y << endl;
    cmdPub=nh.advertise<geometry_msgs::Twist>(CmdTopic,1000);
}

Actuator::Actuator::~Actuator(){
    GoalMarker.points.clear();
    GoalClose.clear();
    centroids.clear();
    VisInit();
}
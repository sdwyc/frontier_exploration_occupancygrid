#include "frontier_detector.h"
#include "actuator.h"


int FrontierDetector::FrontierDetector::GridValue(nav_msgs::OccupancyGrid &map,geometry_msgs::Point& x1){
    float resolution=map.info.resolution;
    int width = map.info.width;
    float Xoriginx=map.info.origin.position.x,
        Xoriginy=map.info.origin.position.y;
    int index;
    index=int((x1.y+std::fabs(Xoriginy))/map.info.resolution)*map.info.width+int((x1.x+std::fabs(Xoriginx))/map.info.resolution);
    int out = map.data[index];
    return out;
}

int  FrontierDetector::FrontierDetector::CheckNeibor(nav_msgs::OccupancyGrid &inflated_map,long& index){
    // Get image x/y
    int x = index%inflated_map.info.width,
        y = index/inflated_map.info.width;
    int flag=0;
    for(int i=x-1;i<x+2;i++)
    {
        if(flag==1){break;}
        for(int j=y-1;j<y+2;j++)
        {
            if(i==x && j==y){continue;}
            if(i-1<0 || i+1>inflated_map.info.width || j-1<0 || j+1>inflated_map.info.height){continue;}
            if(inflated_map.data[j*inflated_map.info.width+i] >= 35 && inflated_map.data[j*inflated_map.info.width+i] < 65
               || inflated_map.data[j*inflated_map.info.width+i] == -1)
            {
                // if neibor cell is unknown cell, flag is true
                flag=1;
                break;
            }
        }
    }
    return flag;

}

void FrontierDetector::FrontierDetector::InitVis(){
    //? frontier is blue
    frontier_vis.header.frame_id = header.frame_id;
    frontier_vis.header.stamp = header.stamp;
    frontier_vis.ns = "frontier";
    frontier_vis.id = 0;
    frontier_vis.lifetime = ros::Duration();
    frontier_vis.type=frontier_vis.POINTS;
    frontier_vis.action = frontier_vis.ADD;
    frontier_vis.color.a=0.7;
    frontier_vis.color.r=0.0;
    frontier_vis.color.g=0.0;
    frontier_vis.color.b=1.0;
    frontier_vis.scale.x=MapRevolution;
    frontier_vis.scale.y=MapRevolution;
    frontier_vis.scale.z=0.0;
    frontier_vis.pose.orientation.w=1.0;
    frontier_vis.points.clear();

    //! centroids are red
    centroid_vis.header.frame_id=header.frame_id;
    centroid_vis.header.stamp = header.stamp;
    centroid_vis.ns = "centroids";
    centroid_vis.id = 1;
    centroid_vis.lifetime = ros::Duration();
    centroid_vis.type=centroid_vis.POINTS;
    centroid_vis.action = centroid_vis.ADD;
    centroid_vis.color.a=1.0;
    centroid_vis.color.r=1.0;
    centroid_vis.color.g=0.0;
    centroid_vis.color.b=0.0;
    centroid_vis.scale.x=MapRevolution*3;
    centroid_vis.scale.y=MapRevolution*3;
    centroid_vis.scale.z=0.0;
    centroid_vis.pose.orientation.w=1.0;
    centroid_vis.points.clear();
}

void FrontierDetector::FrontierDetector::mapCallback(const nav_msgs::OccupancyGridConstPtr& raw_map){
    FrontierDetector::raw_map.header.frame_id = "inflated_map";
    FrontierDetector::raw_map.header.stamp = header.stamp;
    FrontierDetector::raw_map.info = raw_map->info;
    FrontierDetector::raw_map.data = raw_map->data;
    // Inflate grid map
    InflateMap(FrontierDetector::raw_map,FrontierDetector::inflated_map);
    // Get all frontiers
    ComputeFrontier(inflated_map);
    Frontier.points = frontier;
    Centroids.points = centroids;
    FrontierPub_.publish(Frontier);
    if(Centroids.points.size()!=0){CentroidsPub_.publish(Centroids);}
    // std::cout << "Found frontier cells: " << FrontierDetector::frontier.size() << std::endl
    //           << "Found frontier: " << FrontierDetector::centroids.size() << std::endl;
    Visualization();
}

bool FrontierDetector::FrontierDetector::centroidCallback(frontier_exploration::get_centroids::Request &req,
                                                          frontier_exploration::get_centroids::Response &res){
    ComputeCentroids(inflated_map,frontier);
    res.centroids = centroids;
    std::cout << "Centroids computing completely!" << std::endl;
    return true;
}

void FrontierDetector::FrontierDetector::Visualization(){
    if(FrontierDetector::frontier.empty()){
        std::cout << "Frontier no found! Waiting......" << std::endl;
    }
    if(FrontierDetector::centroids.empty()){
        std::cout << "Computing the goal! Waiting....." << std::endl;
    }
    if(frontier.size()==0 || centroids.size()==0){  // Avoid node died if no found frontier or centroids
        FrontierDetector::frontierMarker_.publish(FrontierDetector::frontier_vis);
        FrontierDetector::centroidMarker_.publish(FrontierDetector::centroid_vis);        
    }
    else{
        FrontierDetector::frontier_vis.points.clear();
        FrontierDetector::centroid_vis.points.clear();
        FrontierDetector::frontier_vis.points = FrontierDetector::frontier;
        FrontierDetector::centroid_vis.points = FrontierDetector::centroids;
        FrontierDetector::frontierMarker_.publish(FrontierDetector::frontier_vis);
        FrontierDetector::centroidMarker_.publish(FrontierDetector::centroid_vis);
    }
}

void FrontierDetector::FrontierDetector::InitDetector(){

    FrontierDetector::frontier.clear();
    FrontierDetector::centroids.clear();
    InitVis();
    inflated_map.data.clear();

    ros::param::param<float>("~/obstacle_inflation",this->OBSTABLE_INFLATION,0.3);  //* Deliver param by param server
    ros::param::param<float>("~/map_revolution",this->MapRevolution,0.1);
}

bool FrontierDetector::FrontierDetector::InflateMap(nav_msgs::OccupancyGrid& raw_map, nav_msgs::OccupancyGrid& inflated_map){
    inflated_map = raw_map;
    int dilate_amount = round(OBSTABLE_INFLATION / raw_map.info.resolution);
    for (int x = 0; x < raw_map.info.width; x++) {
        for (int y = 0; y < raw_map.info.height; y++) {
            if (raw_map.data[raw_map.info.width * y + x] < 65 && raw_map.data[raw_map.info.width * y + x] >= 0) {
                inflated_map.data[raw_map.info.width * y + x] = 0;   // Free cell continue
                continue;
            }
            if (raw_map.data[raw_map.info.width * y + x] == -1){ continue; }
            for (int i = -dilate_amount; i <= dilate_amount; i++) {
                for (int j = -dilate_amount; j <= dilate_amount; j++) {

                    int x_d = x + i;
                    int y_d = y + j;

                    if (x_d < 0 || x_d > raw_map.info.width - 1 || y_d < 0 || y_d > raw_map.info.height - 1) {
                        continue;
                    }
                    inflated_map.data[raw_map.info.width * y_d + x_d] = 100;    // Inflate map with obstacle cells
                }
            }
        }
    }
    inflatedMapPub_.publish(inflated_map);
}

bool FrontierDetector::FrontierDetector::ComputeFrontier(nav_msgs::OccupancyGrid& inflated_map){
    FrontierDetector::frontier.clear();
    geometry_msgs::Point pt;
    geometry_msgs::Point p;
    if(!inflated_map.data.empty())
    {
        for(long n=0;n<inflated_map.data.size();n++){
            // If cell is free cells and neibors exist unknown cells, then the current cell is frontier cells
            if(inflated_map.data[n]>=0&& inflated_map.data[n]<35 && CheckNeibor(inflated_map,n)==1){
                int n_x = n%inflated_map.info.width,
                    n_y = n/inflated_map.info.width;
                p.x=(n_x+0.5)*inflated_map.info.resolution-std::fabs(inflated_map.info.origin.position.x);  // world frame -> map frame
                p.y=(n_y+0.5)*inflated_map.info.resolution-std::fabs(inflated_map.info.origin.position.y);  // 0.5 is compensation value
                p.z=0.0;
                FrontierDetector::frontier.push_back(p);
            }
        }   
    }
    else {ROS_INFO("map data isn't received!");return false;}
    return true;
}

bool FrontierDetector::FrontierDetector::ComputeCentroids(nav_msgs::OccupancyGrid &inflated_map,std::vector<geometry_msgs::Point>& frontiers){
    std::vector<geometry_msgs::Point> groupClose;
    geometry_msgs::Point temp;
    FrontierDetector::centroids.clear();
    FrontierDetector::raw_centroids.clear();
    pointGroup.clear();
    if (frontiers.size()==0){
        std::cout << "Cannot find any frontiers! Checking!!" << std::endl;
        return false;
    }
    for(int i=0;i<frontiers.size();i++)
    {
        if(frontierClose.size()>1){
            if(std::find(frontierClose.begin(),frontierClose.end(),frontiers[i])!=frontierClose.end()){continue;}
        }
        pointGroup.push_back(frontiers[i]);
        frontierClose.push_back(frontiers[i]);
        // Find frontier groups, every group is disconnected from each other
        Grouping(inflated_map,frontiers[i]);
        pointGroup = Sort(inflated_map, pointGroup); // Sort based on map-image index
        if(pointGroup.size()<=6) // Avoid too short frontier
        {
            pointGroup.clear();
            continue;
        }
        int index =int(ceil(pointGroup.size()/2));

        FrontierDetector::raw_centroids.push_back(pointGroup[index]);
        pointGroup.clear();
    }
    frontierClose.clear();
    
    std::vector<int> pop_index;
    // Raw centroid filter: too close and non-collision centroid pairs will remove one
    if(raw_centroids.size()>=2){
        for(int m=0;m<raw_centroids.size();m++){
            for(int n=raw_centroids.size();n>m;n--){
                if(!pop_index.empty()){
                    if(std::find(pop_index.begin(),pop_index.end(),n) != pop_index.end()
                    || std::find(pop_index.begin(),pop_index.end(),m) != pop_index.end())
                    {
                        continue;
                    }
                }
                float distance=sqrt(pow((raw_centroids[m].x-raw_centroids[n].x),2)+pow((raw_centroids[m].y-raw_centroids[n].y),2));
                if(distance < 3 && CheckCollision(raw_map,raw_centroids[m],raw_centroids[n])){
                    if(!pop_index.empty()){
                        if(std::find(pop_index.begin(),pop_index.end(),n) == pop_index.end()){
                            pop_index.push_back(n);
                        }
                    }
                    else{ pop_index.push_back(n); }
                }
            }
        }
        for(int num=0;num<raw_centroids.size();num++){
            if(std::find(pop_index.begin(),pop_index.end(),num) == pop_index.end()){
                centroids.push_back(raw_centroids[num]);
            }
        }
        std::cout << "pop_index: " << pop_index.size()<< std::endl;
        std::cout << "raw_centroids: " << raw_centroids.size()<< std::endl;
        std::cout << "centroids: " << centroids.size()<< std::endl;
        raw_centroids.clear();
        pop_index.clear();
    }
    return true;
}
// Lazy collision check
bool FrontierDetector::FrontierDetector::CheckCollision(const nav_msgs::OccupancyGrid& map, 
                                geometry_msgs::Point& start, geometry_msgs::Point& end){
    float length = sqrt(pow((start.x-end.x),2)+pow((start.y-end.y),2));
    float COS_THETA = (end.x - start.x)/length;
    float SIN_THETA = (end.y - start.y)/length;
    float resolution = map.info.resolution;
    float STEP = resolution;
    int count=0;

    float x_check = start.x;
    float y_check = start.y;
    while (fabs(x_check-end.x)>STEP && fabs(y_check-end.y)>STEP){
        int x_check_world=(x_check+fabs(map.info.origin.position.x))/map.info.resolution;
        int y_check_world=(y_check+fabs(map.info.origin.position.y))/map.info.resolution;
        if(map.data[x_check_world+(y_check_world*map.info.width)] >= 70){
            count++;
        }
        x_check+=STEP*COS_THETA;
        y_check+=STEP*SIN_THETA;
        if(count>2){ return false; }
    }
    return true;
}

void FrontierDetector::FrontierDetector::Grouping(nav_msgs::OccupancyGrid& inflated_map,geometry_msgs::Point& point){
    int out=0;
    geometry_msgs::Point temp;
    geometry_msgs::Point worldPoint;
    // Transform the point to world(image) frame
    worldPoint.x = (point.x+std::fabs(inflated_map.info.origin.position.x))/inflated_map.info.resolution-0.5;
    worldPoint.y = (point.y+std::fabs(inflated_map.info.origin.position.y))/inflated_map.info.resolution-0.5;
    worldPoint.z = 0.0;

    if(!frontier.empty()){
        for(float i=worldPoint.x-1;i<=worldPoint.x+1;i++){
            if(out==1){break;}
            for(float j=worldPoint.y-1;j<=worldPoint.y+1;j++){
                int index;
                if(i==worldPoint.x && j==worldPoint.y){continue;}
                temp.x = (i+0.5)*inflated_map.info.resolution-std::fabs(inflated_map.info.origin.position.x);
                temp.y = (j+0.5)*inflated_map.info.resolution-std::fabs(inflated_map.info.origin.position.y);
                temp.z = 0.0;    //*  temp is in map frame

                if(std::find(frontier.begin(),frontier.end(),temp)!=frontier.end()){
                    if(frontierClose.size()>1){
                        if(std::find(frontierClose.begin(),frontierClose.end(),temp)!=frontierClose.end()){continue;}
                    }
                    auto itera = std::find(frontier.begin(),frontier.end(),temp);
                    index = std::distance(frontier.begin(),itera);  // obtain index of fronteir
                    pointGroup.push_back(frontier[index]);
                    frontierClose.push_back(frontier[index]);
                    Grouping(inflated_map,frontier[index]); // recursive structure
                    out=1;  // Until all frontier is iterated, out=1 break the loop
                    break;
                }
            }
        }
    }
}

std::vector<geometry_msgs::Point> FrontierDetector::FrontierDetector::Sort(nav_msgs::OccupancyGrid& inflated_map,std::vector<geometry_msgs::Point>& pts){
    std::vector<geometry_msgs::Point> outcome;
    outcome = pts;
    geometry_msgs::Point p;
    if(!pts.size()==0){
        p = pts[0];
        for(int i=0;i<outcome.size()-1;i++)
        {
            for(int j=0;j<outcome.size()-1-i;j++)
            {
                // Sort based on image index
                if((outcome[j].y*inflated_map.info.width+outcome[j].x) < (outcome[j+1].y*inflated_map.info.width+outcome[j+1].x))
                {
                    p = outcome[j];
                    outcome[j] = outcome[j+1];
                    outcome[j+1] = p;
                }
            }
        }
    }
    else{std::cout << "Group is empty!!" << std::endl;}
    return outcome;
}

FrontierDetector::FrontierDetector::FrontierDetector(ros::NodeHandle& nh):
    frontierMarker_(nh.advertise<visualization_msgs::Marker>("frontier_vis",1000)),
    centroidMarker_(nh.advertise<visualization_msgs::Marker>("centroid_vis",1000)),
    FrontierPub_(nh.advertise<frontier_exploration::PointArray>("frontier",1000)),
    CentroidsPub_(nh.advertise<frontier_exploration::PointArray>("centroids",1000)),
    inflatedMapPub_(nh.advertise<nav_msgs::OccupancyGrid>("inflated_map",1000)),
    CentriodClient_(nh.serviceClient<std_srvs::Empty>("get_centroids")),
    CentriodServer_(nh.advertiseService("get_centriods",&FrontierDetector::centroidCallback,this)),
    MapSub_(nh.subscribe("map",10,&FrontierDetector::mapCallback,this))
{
    InitDetector();
    InitVis();
    std::cout << "Planner is Ready!!!" << std::endl;
}

FrontierDetector::FrontierDetector::~FrontierDetector(){
    FrontierDetector::frontier.clear();
    FrontierDetector::centroids.clear();
    InitVis();
    inflated_map.data.clear();
}
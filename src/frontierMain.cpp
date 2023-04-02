#include <ros/ros.h>
#include <vector>
#include <time.h>
#include "frontier_detector.h"
#include "actuator.h"

int main (int argc, char **argv){
    ros::init(argc, argv, "frontier_planner");
    ros::NodeHandle nh;
    time_t startTime, currTime, Duration;
    time_t Limit = 180; // The max time limitation for single goal navigation
    int changeFlag = 0; // if current goal is too close to obstacle, give up the goal immediately
    std::cout << "------------Exploration Starting------------" << std::endl;
    // Create the frontier_detector and actuator
    FrontierDetector::FrontierDetector frontier_detector(nh);
    Actuator::Actuator actuator(nh);
    // Rotate 360 degree for initialized environment
    actuator.Rotation(360.0);
    //! -------------------- Main Loop --------------------------------- !//
    while(nh.ok()){
        ros::spinOnce();
        // Get all centroids
        frontier_detector.ComputeCentroids(frontier_detector.inflated_map,frontier_detector.frontier);
        // Select a goal according to the cost value
        actuator.SelectGoal(frontier_detector.centroids);
        std::cout << "Found frontier cells: " << frontier_detector.frontier.size() << std::endl
                  << "Found frontier: " << frontier_detector.centroids.size() << std::endl;
        // Navigate to the goal
        actuator.MoveToGoal();
        time(&startTime);
        Duration = 0;
        // If goal is lost, abandon, or aborted, over time, then selecting the next goal
        while(nh.ok() && actuator.ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED
              && actuator.ac.getState() != actionlib::SimpleClientGoalState::LOST
              && actuator.ac.getState() != actionlib::SimpleClientGoalState::ABORTED
              && bool(Duration < Limit)){
            ros::spinOnce();
            if(frontier_detector.GridValue(frontier_detector.inflated_map,actuator.Goal)>=65){
                changeFlag = 1;
                actuator.ac.cancelAllGoals();
                std::cout << "Goal's close to obstacle. Changed Goal!!" << std::endl;
                break;
            }
            time(&currTime);
            Duration = currTime - startTime;  // Avoid spending too much time for one goal
        }
        if(Duration >= Limit){std::cout << "Overtime. Changed Goal!!" << std::endl;}
        actuator.AddToClose(actuator.Goal); // Add to closeList for avoiding go to the explored goal
        if(changeFlag != 1 && Duration < Limit){
            std::cout << "Reached the goal!" << std::endl; actuator.Rotation(0.0);
            }
        else{changeFlag = 0;}
        
        if(frontier_detector.frontier.size()==0 || actuator.GoHomeFlag==1){ // For homing
            actuator.ReturnHome();
            while(nh.ok() && actuator.ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                std::cout << "Exploration finished! Returning home.." << std::endl;
                ros::Duration(5).sleep();
                ros::spinOnce();
            }
            nh.shutdown();
        }
    }
    return 0;
}

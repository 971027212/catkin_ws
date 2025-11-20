#include <ros/ros.h>
#include <vector>
#include <time.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <cmath>

#include "frontier_detector.h"
#include "actuator.h"

namespace {

struct ExplorationSample{
    int iteration;
    double coverage;
    double distance;
};

double ComputeCoverage(const nav_msgs::OccupancyGrid& map){
    if(map.data.empty()){
        return 0.0;
    }
    double known = 0.0;
    for(const auto cell : map.data){
        if(cell != -1){
            known += 1.0;
        }
    }
    return known / static_cast<double>(map.data.size());
}

void WriteMetricsToCsv(const std::string& path,
                       const std::vector<ExplorationSample>& history){
    if(history.empty()){
        ROS_WARN("No exploration metrics collected, skip writing CSV.");
        return;
    }
    std::ofstream ofs(path.c_str());
    if(!ofs.is_open()){
        ROS_ERROR("Failed to open metrics output path: %s", path.c_str());
        return;
    }
    ofs << "iteration,coverage,distance\n";
    ofs << std::fixed << std::setprecision(4);
    for(const auto& sample : history){
        ofs << sample.iteration << ','
            << sample.coverage << ','
            << sample.distance << "\n";
    }
    ofs.close();
    ROS_INFO("Saved exploration metrics to %s", path.c_str());
}



} // namespace

int main (int argc, char **argv){
    ros::init(argc, argv, "frontier_planner");
    ros::NodeHandle nh;

    time_t startTime, currTime, Duration;
    time_t Limit = 180; // The max time limitation for single goal navigation
    int changeFlag = 0; // if current goal is too close to obstacle, give up the goal immediately

    // Active-SLAM 相关参数
    double coverage_threshold = 0.9;
    int max_iterations = 500;
    std::string metrics_output_path = "/tmp/frontier_metrics.csv";
    ros::param::param<double>("~/coverage_threshold", coverage_threshold, coverage_threshold);
    ros::param::param<int>("~/max_iterations", max_iterations, max_iterations);
    ros::param::param<std::string>("~/metrics_output_path", metrics_output_path, metrics_output_path);

    std::vector<ExplorationSample> metrics_history;
    double cumulative_distance = 0.0;

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
        frontier_detector.ComputeCentroids(frontier_detector.inflated_map,
                                           frontier_detector.frontier);

        // Select a goal according to the cost value (Active-SLAM utility)
        actuator.SelectGoal(frontier_detector.centroids);

        std::cout << "Found frontier cells: " << frontier_detector.frontier.size() << std::endl
                  << "Found frontier: " << frontier_detector.centroids.size() << std::endl;

        // Navigate to the goal
        actuator.MoveToGoal();
        time(&startTime);
        Duration = 0;

        // If goal is lost, abandon, or aborted, over time, then selecting the next goal
        while(nh.ok() &&
              actuator.ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
              actuator.ac.getState() != actionlib::SimpleClientGoalState::LOST &&
              actuator.ac.getState() != actionlib::SimpleClientGoalState::ABORTED &&
              bool(Duration < Limit)){
            ros::spinOnce();
            if(frontier_detector.GridValue(frontier_detector.inflated_map,actuator.Goal) >= 65){
                changeFlag = 1;
                actuator.ac.cancelAllGoals();
                std::cout << "Goal's close to obstacle. Changed Goal!!" << std::endl;
                break;
            }
            time(&currTime);
            Duration = currTime - startTime;  // Avoid spending too much time for one goal
        }

        if(Duration >= Limit){
            std::cout << "Overtime. Changed Goal!!" << std::endl;
        }

        // Add to closeList for avoiding go to the explored goal
        actuator.AddToClose(actuator.Goal);

        if(changeFlag != 1 && Duration < Limit){
            std::cout << "Reached the goal!" << std::endl;
            actuator.Rotation(0.0);
            actuator.MarkGoalReached();

            // ---- Active-SLAM: 记录覆盖率与路径长度 ----
            double coverage = ComputeCoverage(frontier_detector.inflated_map);
            const double path_length = actuator.GetLastPlanLength();
            if(std::isfinite(path_length)){
                cumulative_distance += path_length;
            }
            const int iteration_id = static_cast<int>(metrics_history.size()) + 1;
            metrics_history.push_back({iteration_id, coverage, cumulative_distance});
            WriteMetricsToCsv(metrics_output_path, metrics_history);
            ROS_INFO("Iteration %d - coverage %.3f, cumulative distance %.3f",
                     iteration_id, coverage, cumulative_distance);

            // 终止条件：覆盖率或迭代次数
            if((coverage_threshold > 0.0 && coverage >= coverage_threshold) ||
               (max_iterations > 0 && iteration_id >= max_iterations)){
                actuator.GoHomeFlag = 1;
            }
        }
        else{
            changeFlag = 0;
        }

        // For homing
        if(frontier_detector.frontier.size() == 0 || actuator.GoHomeFlag == 1){
            actuator.PublishExplorationSummary();
            actuator.ReturnHome();
            while(nh.ok() && actuator.ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){

                std::cout << "Exploration finished! Returning home.." << std::endl;
                ros::Duration(5).sleep();
                ros::spinOnce();
            }

            nh.shutdown();

        }
    }

    // 写出探索指标 CSV
    WriteMetricsToCsv(metrics_output_path, metrics_history);

    return 0;
}


#include <time.h>
#include <cmath>
#include <math.h>
#include <limits>
#include <algorithm>
#include "actuator.h"

using namespace std;

void Actuator::Actuator::centroidCallback(const frontier_exploration::PointArrayConstPtr& msg){
    centroids = msg->points;
    Visualization();
}

void Actuator::Actuator::mapCallback(const nav_msgs::OccupancyGridConstPtr& RawMap){
    raw_map = *RawMap;
}

bool Actuator::Actuator::worldToMap(double wx, double wy, int& mx, int& my) const{
    if(raw_map.info.resolution <= 0.0 || raw_map.data.empty()){
        return false;
    }
    const double origin_x = raw_map.info.origin.position.x;
    const double origin_y = raw_map.info.origin.position.y;
    mx = static_cast<int>((wx - origin_x)/raw_map.info.resolution);
    my = static_cast<int>((wy - origin_y)/raw_map.info.resolution);
    if(mx < 0 || my < 0){
        return false;
    }
    const int width = static_cast<int>(raw_map.info.width);
    const int height = static_cast<int>(raw_map.info.height);
    if(mx >= width || my >= height){
        return false;
    }
    return true;
}

Actuator::Actuator::InfoMetrics Actuator::Actuator::computeInfoAndUncertainty(const geometry_msgs::Point& viewpoint){
    InfoMetrics metrics{0.0, 0.0};
    if(raw_map.data.empty() || info_radius_ <= 0.0){
        return metrics;
    }
    const double angle_increment = info_angle_step_deg_ * PI / 180.0;
    if(angle_increment <= 0.0){
        return metrics;
    }
    const double max_range = info_radius_;
    const double step = std::max(raw_map.info.resolution, 0.01f);
    const int width = static_cast<int>(raw_map.info.width);
    std::unordered_set<int> visited;
    double variance_sum = 0.0;
    size_t variance_samples = 0;

    for(double angle = 0.0; angle < 2 * PI; angle += angle_increment){
        double ray_length = 0.0;
        while(ray_length <= max_range){
            const double wx = viewpoint.x + ray_length * cos(angle);
            const double wy = viewpoint.y + ray_length * sin(angle);
            int mx = 0;
            int my = 0;
            if(!worldToMap(wx, wy, mx, my)){
                break;
            }
            const int idx = my * width + mx;
            if(!visited.insert(idx).second){
                // already considered in previous ray
                ray_length += step;
                continue;
            }
            const int8_t value = raw_map.data[idx];
            double probability = 0.5; // unknown cell
            if(value >= 0){
                probability = static_cast<double>(value) / 100.0;
            }
            variance_sum += probability * (1.0 - probability);
            variance_samples++;
            if(value == -1){
                metrics.gain += 1.0;
            }
            if(value >= 65){
                break; // obstacle encountered
            }
            ray_length += step;
        }
    }
    if(variance_samples > 0){
        metrics.uncertainty = variance_sum / static_cast<double>(variance_samples);
    }
    return metrics;
}

double Actuator::Actuator::computePathCost(const geometry_msgs::Point& viewpoint,
                                           nav_msgs::Path& plan)
{
    if (make_plan_client.exists()) {
        nav_msgs::GetPlan srv;
        srv.request.start.header.frame_id = header.frame_id;
        srv.request.start.header.stamp = ros::Time::now();
        srv.request.start.pose.position = robotPose.Position;
        const double yaw_rad = robotPose.Yaw * PI / 180.0;
        srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rad);

        srv.request.goal = srv.request.start;
        srv.request.goal.pose.position = viewpoint;
        srv.request.tolerance = plan_tolerance_;

        if (make_plan_client.call(srv) && !srv.response.plan.poses.empty()) {
            plan = srv.response.plan;

            double cost = 0.0;
            for (size_t i = 1; i < plan.poses.size(); ++i) {
                const geometry_msgs::Point& prev = plan.poses[i-1].pose.position;
                const geometry_msgs::Point& curr = plan.poses[i].pose.position;
                cost += hypot(curr.x - prev.x, curr.y - prev.y);
            }
            return cost;
        }
    }

    plan.poses.clear();

    double cost = hypot(viewpoint.x - robotPose.Position.x,
                        viewpoint.y - robotPose.Position.y);

    geometry_msgs::Point start = robotPose.Position;
    geometry_msgs::Point end = viewpoint;
    int collision = CheckCollision(raw_map, start, end);
    auto sigmoid = [collision](){
        return 2.0 / (1.0 + std::exp(-0.3 * collision)) - 1.0;
    };
    cost = cost * (1.0 + sigmoid());

    return cost;
}
       

double Actuator::Actuator::computeLoopScore(const nav_msgs::Path& plan) const{
    if(plan.poses.empty() || raw_map.data.empty()){
        return 0.0;
    }
    size_t known_cells = 0;
    size_t total_cells = 0;
    const int width = static_cast<int>(raw_map.info.width);
    for(const auto& pose_stamped : plan.poses){
        int mx = 0;
        int my = 0;
        if(!worldToMap(pose_stamped.pose.position.x, pose_stamped.pose.position.y, mx, my)){
            continue;
        }
        const int idx = my * width + mx;
        if(idx < 0 || idx >= static_cast<int>(raw_map.data.size())){
            continue;
        }
        total_cells++;
        if(raw_map.data[idx] != -1){
            known_cells++;
        }
    }
    if(total_cells == 0){
        return 0.0;
    }
    return static_cast<double>(known_cells) / static_cast<double>(total_cells);
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
    ros::Rate rate(rotation_ctrl_rate_);
    time(&initTime);

    while((rotated_angle < angle) && (ros::ok()) && bool(duration<limitation)){
        time(&currTime);
        duration = currTime-initTime;
        double old_yaw = robotPose.Yaw;
        cmdPub.publish(RotSpeed);
        rate.sleep();
        ObtainPose();
        ros::spinOnce();
        double dYaw = robotPose.Yaw - old_yaw;
        if((robotPose.Yaw>=0 && robotPose.Yaw<350) && (old_yaw>=350 && old_yaw< 360)){
            dYaw = robotPose.Yaw + (360.0-old_yaw);
        }
        rotated_angle += dYaw;
    }
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;
    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;
    cmdPub.publish(stop_cmd);
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
    
    ObtainPose();
    header.stamp = ros::Time::now();
    MoveGoal.target_pose.header.frame_id = header.frame_id;
    MoveGoal.target_pose.header.stamp = header.stamp;
    // Movebase action method
    MoveGoal.target_pose.pose.position = Goal;
    // Orient the robot so that it initially faces the next goal. This avoids the
    // robot starting every trajectory facing the world x-axis which often causes
    // the local planner to fight the heading error and results in the drifting
    // behaviour that was observed when switching to a new goal.
    const double yaw = atan2(Goal.y - robotPose.Position.y,
                             Goal.x - robotPose.Position.x);
    MoveGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

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

void Actuator::Actuator::MarkGoalReached(){
    exploration_path_.push_back(Goal);
}

void Actuator::Actuator::PublishExplorationSummary(){
    if(summary_published_ || exploration_path_.size() < 2){
        return;
    }
    path_vis.header.stamp = ros::Time::now();
    path_vis.points = exploration_path_;
    path_pub.publish(path_vis);
    summary_published_ = true;
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


    path_vis.header.frame_id = header.frame_id;
    path_vis.header.stamp = header.stamp;
    path_vis.ns = "exploration_path";
    path_vis.id = 6;
    path_vis.lifetime = ros::Duration();
    path_vis.type = visualization_msgs::Marker::LINE_STRIP;
    path_vis.action = visualization_msgs::Marker::ADD;
    path_vis.pose.orientation.w = 1.0;
    path_vis.scale.x = 0.05;
    path_vis.color.a = 1.0;
    path_vis.color.r = 0.0;
    path_vis.color.g = 0.6;
    path_vis.color.b = 1.0;
    path_vis.points.clear();
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
                PoseListner.lookupTransform("map", RobotBase, ros::Time::now() -ros::Duration(0.1), transform);
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
    ros::param::param<float>("~/rotate_speed",RotateSpeed,1.0);  // rad/s
    ros::param::param<double>("~/rotation_control_rate",rotation_ctrl_rate_,30.0);
    ros::param::param<double>("~/alpha_gain",alpha_gain_,0.0);
    ros::param::param<double>("~/gamma_uncertainty",gamma_uncertainty_,0.0);
    ros::param::param<double>("~/delta_loop",delta_loop_,0.0);
    ros::param::param<double>("~/beta_cost",beta_cost_,1.0);
    ros::param::param<double>("~/info_radius",info_radius_,3.0);
    ros::param::param<double>("~/info_angle_step_deg",info_angle_step_deg_,5.0);
    ros::param::param<double>("~/plan_tolerance",plan_tolerance_,0.05);

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
    
    //MoveGoal.target_pose.header.frame_id = header.frame_id;
    //MoveGoal.target_pose.header.frame_id = "inflated_map";
    MoveGoal.target_pose.header.frame_id = header.frame_id;
    MoveGoal.target_pose.header.stamp = header.stamp;
    MoveGoal.target_pose.pose.position.z = 0.0;
    MoveGoal.target_pose.pose.orientation.w = 1.0;

    last_plan_length_ = 0.0;
    last_plan_.poses.clear();
    last_plan_.header.frame_id = header.frame_id;
}

void Actuator::Actuator::AddToClose(geometry_msgs::Point& goal){
    // Assure current goal not in close
    if(find(GoalClose.begin(),GoalClose.end(),goal) == GoalClose.end()){
        GoalClose.push_back(goal);
    }
}

geometry_msgs::Point Actuator::Actuator::SelectGoal(std::vector<geometry_msgs::Point>& centroids){
    ObtainPose();
    struct Candidate{
        geometry_msgs::Point point;
        double gain;
        double uncertainty;
        double loop;
        double cost;
        nav_msgs::Path path;
    };
    std::vector<Candidate> candidates;
    const size_t total_centroids = centroids.size();
    if(total_centroids == 0){
        cout << "No centroids!  No goal!" << endl;
        GoHomeFlag = 1;
        return Home;
    }
    size_t close_count = 0;
    for(size_t i=0;i<centroids.size();i++){
        if(!GoalClose.empty()){
            for(size_t n=0;n<GoalClose.size();n++){
                float Distance = sqrt(pow((centroids[i].x-GoalClose[n].x),2)+pow((centroids[i].y-GoalClose[n].y),2));
                if(Distance < GoalTolerance && Distance > 0.0001){
                    GoalClose.push_back(centroids[i]);
                    break;
                }
            }
            if(find(GoalClose.begin(),GoalClose.end(),centroids[i])!=GoalClose.end()){
                close_count++;
                continue;
            }
        }
        nav_msgs::Path plan;
        const InfoMetrics info_metrics = computeInfoAndUncertainty(centroids[i]);
        const double cost = computePathCost(centroids[i], plan);
        if(!std::isfinite(cost)){
            continue;
        }
        const double loop = computeLoopScore(plan);
        candidates.push_back({centroids[i], info_metrics.gain, info_metrics.uncertainty, loop, cost, plan});
    }

    if(close_count == total_centroids || candidates.empty()){
        GoHomeFlag = 1;
        return Home;
    }

    double min_gain = std::numeric_limits<double>::max();
    double max_gain = std::numeric_limits<double>::lowest();
    double min_uncertainty = std::numeric_limits<double>::max();
    double max_uncertainty = std::numeric_limits<double>::lowest();
    double min_loop = std::numeric_limits<double>::max();
    double max_loop = std::numeric_limits<double>::lowest();
    double min_cost = std::numeric_limits<double>::max();
    double max_cost = std::numeric_limits<double>::lowest();

    for(const auto& candidate : candidates){
        min_gain = std::min(min_gain, candidate.gain);
        max_gain = std::max(max_gain, candidate.gain);
        min_uncertainty = std::min(min_uncertainty, candidate.uncertainty);
        max_uncertainty = std::max(max_uncertainty, candidate.uncertainty);
        min_loop = std::min(min_loop, candidate.loop);
        max_loop = std::max(max_loop, candidate.loop);
        min_cost = std::min(min_cost, candidate.cost);
        max_cost = std::max(max_cost, candidate.cost);
    }

    auto normalize = [](double value, double min_value, double max_value){
        if(!std::isfinite(value) || (max_value - min_value) <= 1e-6){
            return 0.0;
        }
        return (value - min_value) / (max_value - min_value);
    };

    double best_utility = -std::numeric_limits<double>::infinity();
    Candidate best_candidate = candidates.front();
    for(const auto& candidate : candidates){
        const double gain_norm = normalize(candidate.gain, min_gain, max_gain);
        const double uncertainty_norm = normalize(candidate.uncertainty, min_uncertainty, max_uncertainty);
        const double loop_norm = normalize(candidate.loop, min_loop, max_loop);
        const double cost_norm = normalize(candidate.cost, min_cost, max_cost);
        const double utility = alpha_gain_ * gain_norm +
                               gamma_uncertainty_ * uncertainty_norm +
                               delta_loop_ * loop_norm -
                               beta_cost_ * cost_norm;
        if(utility > best_utility){
            best_utility = utility;
            best_candidate = candidate;
        }
    }

    cout << "GoalClose List size: " << GoalClose.size() << endl;
    Goal = best_candidate.point;
    last_plan_ = best_candidate.path;
    last_plan_length_ = best_candidate.cost;
    iteration++;
    cout << "/************************/" << endl;
    cout << "Iteration: " << iteration << ": \nGoal: "
        << Goal.x << "," << Goal.y << "\nUtility: " << best_utility
        << "\nGain: " << best_candidate.gain
        << "\nUncertainty: " << best_candidate.uncertainty
        << "\nLoop score: " << best_candidate.loop
        << "\nCost: " << best_candidate.cost << endl;
    return Goal;
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
    summary_published_ = false;
    exploration_path_.clear();
    exploration_path_.push_back(Home);
    cout << "Home pose: " << Home.x << "," << Home.y << endl;
    cmdPub=nh.advertise<geometry_msgs::Twist>(CmdTopic,1000);
    make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    make_plan_client.waitForExistence(ros::Duration(5.0));
}

Actuator::Actuator::~Actuator(){
    GoalMarker.points.clear();
    GoalClose.clear();
    centroids.clear();
    VisInit();
}

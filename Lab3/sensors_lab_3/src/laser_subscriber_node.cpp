#include <laser_subscriber_node.h>

using namespace std;

// Subscriber callback functions

void LaserSubscriberNode::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Got a laser scan message with %zd range measurements", msg->ranges.size());
    laser_scan_->clear();

    for(int i = 0; i < msg->ranges.size(); i++){
        double angle = msg->angle_min + msg->angle_increment * (double)i + angle_correction_;
        double distance = 0.0;
        if(msg->ranges[i] > 0.0)
            distance = distance_linear_correction_ * msg->ranges[i] + distance_offset_correction_;

        LaserScanPoint point(distance, angle);
        laser_scan_->push_back(point);
    }

    if(calibration_state_ == CalibrationState::CalibrationRun)
        runCalibration();
}

void LaserSubscriberNode::marker_pub_callback(const ros::TimerEvent &te) {
    ROS_INFO("Publishing markers");
    visualization_msgs::Marker marker = createMarker();

    std::vector<LaserScanPoint>::iterator scan_point = this->laser_scan_->begin();
    for( ; scan_point != this->laser_scan_->end(); scan_point++) {
        if(scan_point->getDistance() > 0.0){
            geometry_msgs::Point point;
            point.x = scan_point->getX();
            point.y = scan_point->getY();
            point.z = 0.0;
            marker.points.push_back(point);
        }
    }
    
    marker_pub_.publish(marker);
}

// Service callback functions

bool LaserSubscriberNode::save_data_callback(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res ) {
    ROS_INFO("Saving data to %s", output_file_name_.c_str());
    
    ofstream output_fstream;
    output_fstream.open(output_file_name_.c_str(), ofstream::out | ofstream::trunc);

    std::vector<LaserScanPoint>::iterator scan_point = this->laser_scan_->begin();
    for( ; scan_point != this->laser_scan_->end(); scan_point++) {
        output_fstream << scan_point->getX() << "," << scan_point->getY() << endl;
    }

    output_fstream.close();
}

bool LaserSubscriberNode::save_calibration_data_callback(msgs::LrfCalibration::Request  &req,
        msgs::LrfCalibration::Response &res ) {

    this->number_of_measurements_ = req.measurements;
    this->field_of_view_ = req.fov;
    if(number_of_measurements_ > 0 && field_of_view_ > 0.0 && field_of_view_ < 360.0 && req.filename != ""){
        this->calibration_filename_ = req.filename + "-" + to_string(number_of_measurements_) + "-" + to_string(field_of_view_) + ".txt";

        ROS_INFO(
            "Initializing calibration filename = %s, number of measurments = %d, field of view = %.2f", 
            calibration_filename_.c_str(),
            number_of_measurements_,
            field_of_view_
        );

        this->calibration_scans_->clear();
        this->calibration_state_ = CalibrationState::CalibrationRun;
        
        runCalibration();
        res.result = true;
    }
    else{   
        ROS_DEBUG(
            "Something is wrong! filename = %s, number of measurments = %d, field of view = %.2f", 
            req.filename.c_str(),
            number_of_measurements_,
            field_of_view_
        );
        res.result = false;
    }
    
}

// Calibration functions

void LaserSubscriberNode::runCalibration(){
    
    if(number_of_measurements_ > 0){
        ROS_INFO("Calibrating number of measurements left %d", number_of_measurements_);
        number_of_measurements_--;
        saveLaserScan();
    }
    else{
        saveCalibration();
        calibration_state_ = CalibrationState::CalibrationIdle;
    } 
}

void LaserSubscriberNode::saveLaserScan(){
    vector<LaserScanPoint> scan;

    double angle_limits = this->field_of_view_ * (3.14/360.0);
    std::vector<LaserScanPoint>::iterator scan_point = this->laser_scan_->begin();
    for( ; scan_point != this->laser_scan_->end(); scan_point++) {
        if(scan_point->getAngle() <= angle_limits && scan_point->getAngle() >= -angle_limits)
            scan.push_back(LaserScanPoint(scan_point->getDistance(), scan_point->getAngle()));
    }

    this->calibration_scans_->addScan(scan);
}

void LaserSubscriberNode::saveCalibration(){ 
    ROS_INFO("Saving calibration data to %s", calibration_filename_.c_str());
    
    ofstream output_fstream;
    ios_base::openmode file_mode;

    output_fstream.open(calibration_filename_.c_str(), ofstream::out | ofstream::trunc);

    for(int i = 0; i < calibration_scans_->size(); i++){
        double mean, std_deviation;
        int number_of_measurements = calibration_scans_->getStatistics(i, &mean, &std_deviation);
        output_fstream << to_string(calibration_scans_->getAngle(i)) << "," << to_string(number_of_measurements) << ",";
        output_fstream << to_string(mean) << "," << to_string(std_deviation) << endl;
    }

    ROS_INFO("Closing file.");
    output_fstream.close();
}

// Marker functions

visualization_msgs::Marker LaserSubscriberNode::createMarker() {
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "laser_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    
    return marker;
}

LaserSubscriberNode::LaserSubscriberNode() {
    nh_ = ros::NodeHandle("~");
    n_ = ros::NodeHandle();

    nh_.param<std::string>("laser_topic",laser_topic_,"/scan");
    nh_.param<std::string>("marker_topic",marker_topic_,"viz");
    nh_.param<std::string>("outfile",output_file_name_,"test.m");

    this->laser_scan_ = new vector<LaserScanPoint>(720);

    this->number_of_measurements_ = 0;
    this->field_of_view_ = 0.0;
    this->calibration_filename_ = "";
    this->calibration_state_ = CalibrationState::CalibrationIdle;
    this->calibration_scans_ = new LaserCalibrationScans();
}

void LaserSubscriberNode::subscribeAndAdvertise() {
    //advertise publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker> (marker_topic_ ,10);
    
    //subscribe to laser messages
    laser_sub_ = n_.subscribe(laser_topic_, 100, &LaserSubscriberNode::laser_scan_callback, this);
    
    //advertise service
    save_data_srv_ = nh_.advertiseService("save_data", &LaserSubscriberNode::save_data_callback, this);
    save_calibration_data_srv_ = nh_.advertiseService(
        "save_calibration", 
        &LaserSubscriberNode::save_calibration_data_callback, 
        this
    );

    //setup timer
    marker_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &LaserSubscriberNode::marker_pub_callback, this);
}

LaserSubscriberNode::~LaserSubscriberNode() {
    delete laser_scan_;
    delete calibration_scans_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_sub_node");

    LaserSubscriberNode laser_node;
    ROS_INFO("Laser Subscriber node created");
    laser_node.subscribeAndAdvertise();
    ROS_INFO("Laser Subscriber node initialized");

    ros::spin();

    return 0;
}


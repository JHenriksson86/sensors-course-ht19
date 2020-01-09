#include <sensors_lab3_2018/laser_subscriber_node.h>

using namespace std;

bool LaserSubscriberNode::save_calibration_data_callback(msgs::LrfCalibration::Request  &req,
        msgs::LrfCalibration::Response &res ) {
    std::string filename = req.filename;
    ROS_INFO("Saving calibration data to %s", filename.c_str());
    
    ofstream output_fstream;
    output_fstream.open(filename.c_str(), ofstream::out | ofstream::trunc);

    const double angle_limits = (req.fov / 2.0) * (3.14/180.0);
    
    std::vector<LaserScanPoint>::iterator scan_point = this->laser_scan_->begin();
    for( ; scan_point != this->laser_scan_->end(); scan_point++) {
        if(scan_point->getAngle() <= angle_limits && scan_point->getAngle() >= -angle_limits)
            output_fstream << scan_point->getAngle() << "," << scan_point->getDistance() << endl;
    }

    output_fstream.close();
    res.result = true;
}

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

void LaserSubscriberNode::marker_pub_callback(const ros::TimerEvent &te) {
    ROS_INFO("Publishing markers");
    visualization_msgs::Marker marker = createMarker();

    std::vector<LaserScanPoint>::iterator scan_point = this->laser_scan_->begin();
    for( ; scan_point != this->laser_scan_->end(); scan_point++) {
        geometry_msgs::Point point;
        point.x = scan_point->getX();
        point.y = scan_point->getY();
        point.z = 0.0;
        marker.points.push_back(point);
    }
    
    marker_pub_.publish(marker);
}

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

void LaserSubscriberNode::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Got a laser scan message with %zd range measurements", msg->ranges.size());
    
    this->laser_scan_->clear();
    for(int i = 0; i < msg->ranges.size(); i++){
        double range_angle = msg->angle_min + msg->angle_increment * (double)i;
        
        LaserScanPoint point(msg->ranges[i], range_angle);
        laser_scan_->push_back(point);
    }
    
}

LaserSubscriberNode::LaserSubscriberNode() {
    nh_ = ros::NodeHandle("~");
    n_ = ros::NodeHandle();

    nh_.param<std::string>("laser_topic",laser_topic_,"/scan");
    nh_.param<std::string>("marker_topic",marker_topic_,"viz");
    nh_.param<std::string>("outfile",output_file_name_,"test.m");

    this->laser_scan_ = new vector<LaserScanPoint>(720);
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
    delete this->laser_scan_;
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


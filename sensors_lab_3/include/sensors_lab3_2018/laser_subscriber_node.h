#ifndef LAB3_LASER_SUBSCRIBER_NODE_H //include guards
#define LAB3_LASER_SUBSCRIBER_NODE_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <msgs/LrfCalibration.h>

#include <vector>
#include <cmath>
#include <fstream>

class LaserScanPoint {
    double distance_; 
    double angle_;
    double x_;
    double y_;

    public:

    LaserScanPoint() {
        this->distance_ = 0.0;
        this->angle_ = 0.0;
        this->x_ = 0.0;
        this->y_ = 0.0;
    }

    LaserScanPoint(double distance, double angle) {
        this->setPolarCoordinates(distance, angle);
    }

    void setPolarCoordinates(double distance, double angle) {
        this->distance_ = distance;
        this->angle_ = angle;
        this->x_ = distance * std::cos(angle);
        this->y_ = distance * std::cos(angle); 
    }

    void setCartesianCoordinates(double x, double y) { 
        this->x_ = x;
        this->y_ = y;
        this->distance_ = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        this->distance_ = std::atan2(y, x); 
    }
    
    double getDistance() const { return this->distance_; }

    double getAngle() const { return this->angle_; } 

    double getX() const { return this->x_; }

    double getY() const { return this->y_; }

    ~LaserScanPoint(){}
};

class LaserSubscriberNode {
    //members
    private:
        //node pointing to the global namespace /
        ros::NodeHandle n_;

        //node pointing to the local namespace ~
        ros::NodeHandle nh_;

        //subscriber to laser messages
        ros::Subscriber laser_sub_;

        //publishes a marker at a fixed frequency
        ros::Publisher marker_pub_;

        //service server for saving data to a file
        ros::ServiceServer save_data_srv_;

        //service server for saving calibration data
        ros::ServiceServer save_calibration_data_srv_;

        //timer that trigers marker publishing
        ros::Timer marker_pub_timer_;

        //parameters to be loaded from the param server
        std::string laser_topic_, marker_topic_, output_file_name_;

        //message to be published at every iteration
        visualization_msgs::Marker marker_msg_;

        //Vector containing transformed laser scan points
        std::vector<LaserScanPoint>* laser_scan_;


    //functions
    private:
        //callback for service server
        bool save_data_callback(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &res );

        bool save_calibration_data_callback(msgs::LrfCalibration::Request  &req,
                msgs::LrfCalibration::Response &res );

        //callback for publishing timer
        void marker_pub_callback(const ros::TimerEvent &te);

        //callback for laser message
        void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

        visualization_msgs::Marker createMarker();

    public:
        //default constructor
        LaserSubscriberNode();

        //subscribes to topics and advertises publishers
        void subscribeAndAdvertise();

        ~LaserSubscriberNode();

};

#endif

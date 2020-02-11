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
#include <string>
#include <exception>

class LaserScanPoint {
    double  distance_; 
    double  angle_;
    double  x_;
    double  y_;
    int     numberOfZeroMeasurements;

    public:

    LaserScanPoint() {
        this->distance_ = 0.0;
        this->angle_ = 0.0;
        this->x_ = 0.0;
        this->y_ = 0.0;
        this->numberOfZeroMeasurements = 0;
    }

    LaserScanPoint(double distance, double angle) {
        this->setPolarCoordinates(distance, angle);
    }

    void updateLaserScanPoint(double distance, double angle){
        if(distance > 0.0 || numberOfZeroMeasurements > 5){
            this->setPolarCoordinates(distance, angle);
            numberOfZeroMeasurements = 0;
        }
        else{
            numberOfZeroMeasurements++;
        }
    }

    void setPolarCoordinates(double distance, double angle) {
        this->distance_ = distance;
        this->angle_ = angle;
        this->x_ = distance * std::cos(angle);
        this->y_ = distance * std::sin(angle); 
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

class LaserCalibrationScans {
    private:

    std::vector<double>*                angles_;
    std::vector<std::vector<double>>*   scans_;

    public:

    LaserCalibrationScans(){
        this->scans_ = new std::vector<std::vector<double>>();
        this->angles_ = new std::vector<double>();
    }

    void addScan(const std::vector<LaserScanPoint>& laser_scan){
        std::vector<double> temp_scan;
        
        if(this->angles_->empty())
            initAngles(laser_scan);    

        if(this->angles_->size() != laser_scan.size()){
            ROS_INFO("Scan size missmatch! Angles size = %d and scan size = %d", (int)this->angles_->size(), (int)laser_scan.size());
            throw new std::bad_exception();
        }

        for(int i = 0; i < laser_scan.size(); i++){
            if(angles_->at(i) == laser_scan[i].getAngle())
                temp_scan.push_back(laser_scan[i].getDistance());
            else{
                ROS_INFO("Failed to add laserscan, because angle miss match");
                throw new std::bad_exception();    
            }
        }

        scans_->push_back(temp_scan);
    }

    int getStatistics(
        const unsigned int n, 
        double* out_mean, 
        double* out_std_deviation = nullptr) const
    {

        std::vector<double> distances;
        for(int i = 0; i < this->scans_->size(); i++) {
            double distance = this->scans_->at(i).at(n);
            if(distance > 0.0){
                distances.push_back(distance);
            }
        }

        double mean = this->getSum(distances) / (double)distances.size();

        if(out_std_deviation != nullptr){
            double squared_sum = 0.0;
            for(auto distance : distances) {
                squared_sum += std::pow(distance - mean, 2);
            }
            *out_std_deviation = std::sqrt(squared_sum / mean); 
        }

        *out_mean = mean;
        return distances.size();
    }

    double getAngle(const unsigned int n) const {
        return angles_->at(n);
    }

    void clear(){
        this->angles_->clear();
        this->scans_->clear();
    }

    int size() const {
        return this->angles_->size();
    }

    ~LaserCalibrationScans(){
        delete angles_;
        delete scans_;
    }

    private:

    void initAngles(const std::vector<LaserScanPoint>& laser_scan){
        for(auto scan : laser_scan)
            this->angles_->push_back(scan.getAngle());
    }

    double getSum(const std::vector<double>& scans) const {
        double sum = 0.0;
        for(auto scan : scans)
            sum += scan;
        return sum;
    }
};

class LaserSubscriberNode {
    //members
    private:

        enum CalibrationState{
            CalibrationRun,
            CalibrationIdle
        };

        //node pointing to the global namespace /
        ros::NodeHandle                 n_;

        //node pointing to the local namespace ~
        ros::NodeHandle                 nh_;

        //subscriber to laser messages
        ros::Subscriber                 laser_sub_;

        //publishes a marker at a fixed frequency
        ros::Publisher                  marker_pub_;

        //service server for saving data to a file
        ros::ServiceServer              save_data_srv_;

        //service server for saving calibration data
        ros::ServiceServer              save_calibration_data_srv_;

        //timer that trigers marker publishing
        ros::Timer                      marker_pub_timer_;

        //parameters to be loaded from the param server
        std::string                     laser_topic_, marker_topic_, output_file_name_;

        //message to be published at every iteration
        visualization_msgs::Marker      marker_msg_;

        //Laserscan points
        std::vector<LaserScanPoint>*    laser_scan_;

        //Calibration variables
        int                             number_of_measurements_;
        double                          field_of_view_;
        LaserCalibrationScans*          calibration_scans_;
        std::string                     calibration_filename_;
        CalibrationState                calibration_state_;

        //Calibration correction 
        const double                    angle_correction_ = 0.0; //0.0132; // 0.0 no correction
        const double                    distance_linear_correction_ = 1.0; //0.9894; // 1.0 no correction
        const double                    distance_offset_correction_ = 0.0; //0.0047; // 0.0 no correction

    //functions
    private:
        //callback for publishing timer
        void marker_pub_callback(const ros::TimerEvent &te);

        //callback for laser message
        void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        
        // Service callback functions
        bool save_data_callback(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &res );

        bool save_calibration_data_callback(msgs::LrfCalibration::Request  &req,
                msgs::LrfCalibration::Response &res );

        // Calibration functions
        void runCalibration();

        void saveLaserScan();

        void saveCalibration();

        visualization_msgs::Marker createMarker();

    public:
        //default constructor
        LaserSubscriberNode();

        //subscribes to topics and advertises publishers
        void subscribeAndAdvertise();

        ~LaserSubscriberNode();

};

#endif

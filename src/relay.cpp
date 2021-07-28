/*
 * yolo_depth_fusion/src/relay.cpp
 *
 * Ros node to fuse and republish data from darknet_ros and realsense_ros.
 * Developed during the course CDT406 at MDH.
 *
 * Authors:
 * Magnus Östgren    <ninjalostinsnow@gmail.com>
 *
 */



/***************************************************************
 * Includes
 ***************************************************************/
#include <vector>
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <yolo_depth_fusion/yoloObject.h>
#include <yolo_depth_fusion/yoloObjects.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseStamped.h"



/***************************************************************
 * Global variables
 ***************************************************************/
ros::Publisher* pub;
ros::Publisher* pub_marker;
ros::Publisher* pub_all;

cv_bridge::CvImagePtr depthMap;
std::mutex depthMapAccess;
cv::Mat mDepthImage;

float global_x = 0;
float global_y = 0;
float global_z = 0;

double filterConst;
double maxThreshold;

/***************************************************************
 * Functions
 ***************************************************************/

void republishYolo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void poseRecord(const geometry_msgs::PoseStamped::ConstPtr& msg);
void depthMapCallback(const sensor_msgs::ImageConstPtr& msg);


/***************************************************************
 * main
 * * Inits ros and sets up subscribers and publishers.
 ***************************************************************/

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "relay");
    ros::NodeHandle com;
    
    //Constants for depth filtering, currently not in use
    //com.param("settings/depth_filter", filterConst, 2.0);
    //com.param("settings/threshold", maxThreshold, 0.8);


    //Reading parameters and setting up publisher
    std::string publisherTopicName;
    int publisherQueueSize;
    bool publisherLatch;
    com.param("publishers/object_data/topic", publisherTopicName, std::string("/yolo_depth_fusion/objects"));
    com.param("publishers/object_data/queue_size", publisherQueueSize, 1);
    com.param("publishers/object_data/latch", publisherLatch, false);
    ros::Publisher publisher = com.advertise<yolo_depth_fusion::yoloObjects>(publisherTopicName, publisherQueueSize, publisherLatch);
    pub = &publisher;

    ros::Publisher marker_pub = com.advertise<visualization_msgs::MarkerArray>("/fire", 10, publisherLatch);
    pub_marker = &marker_pub;
    ros::Publisher all_pub = com.advertise<visualization_msgs::MarkerArray>("/all", 100, publisherLatch);
    pub_all = &all_pub;
    //Reading parameters and setting up subscriber for bounding boxes from darknet_ros
    int yoloQueueSize;
    std::string yoloTopicName;
    com.param("subscribers/bounding_boxes/topic", yoloTopicName, std::string("/darknet_ros/bounding_boxes"));
    com.param("subscribers/bounding_boxes/queue_size", yoloQueueSize, 1);
    ros::Subscriber yoloSub = com.subscribe(yoloTopicName, yoloQueueSize, republishYolo);
    ros::Subscriber poseSub = com.subscribe("/mavros/local_position/pose", 10, poseRecord);
    //Reading parameters and setting up subsctriber for depth map
    int depthQueueSize;
    std::string depthTopicName;
    com.param("subscribers/depth_map/topic", depthTopicName, std::string("/camera/depth/image_rect_raw"));
    com.param("subscribers/depth_map/queue_size", depthQueueSize, 1);
    image_transport::ImageTransport imageLayer(com);
    image_transport::Subscriber depthSub = imageLayer.subscribe(depthTopicName, depthQueueSize, depthMapCallback);
    

    ros::spin();
}



/***************************************************************
 * republishYolo
 * * Subscribes to bounding box data from darknet_ros,
 * * reformats that and combibines it with distance data
 * * from a depth map. This fusion of data is then published
 * * in a new topic.
 ***************************************************************/
void poseRecord(const geometry_msgs::PoseStamped::ConstPtr& msg){
    global_x = msg->pose.position.x;
    global_y = msg->pose.position.y;
    global_z = msg->pose.position.z;
}

void republishYolo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    yolo_depth_fusion::yoloObjects objects;
    objects.header = msg->header;
    objects.image_header = msg->image_header;
    
    for (int i = 0; i < (msg->bounding_boxes).size(); i++) {
        yolo_depth_fusion::yoloObject current;
        long xmin = (msg->bounding_boxes)[i].xmin;
        long ymin = (msg->bounding_boxes)[i].ymin;
        long xmax = (msg->bounding_boxes)[i].xmax;
        long ymax = (msg->bounding_boxes)[i].ymax;
        current.width = xmax - xmin;
        current.height = ymax - ymin;
        current.x = xmin + current.width / 2;
        current.y = ymin + current.height / 2;

        depthMapAccess.lock();

        auto intensity = mDepthImage.at<float>(static_cast<int>(current.y), static_cast<int>(current.x));
        depthMapAccess.unlock();

        float dist = intensity;
        double prob = (msg->bounding_boxes)[i].probability;
        //if (prob > maxThreshold || dist - filterConst / prob > 0.0) {
        {
            current.classification = (msg->bounding_boxes)[i].Class;
            current.probability = prob;
            current.distance = dist;
            current.px = dist * (current.x - 320.70361328125) / 617.6864013671875;
            current.py = dist * (current.y - 244.4276123046875) / 618.0162353515625;
            current.pz = dist;
            objects.list.push_back(current);
            //-------------------------------------
            visualization_msgs::Marker marker;
            visualization_msgs::MarkerArray mark;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "my_namespace";
            //marker.type = marker.MESH_RESOURCE
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.id = 0;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = current.px + global_x;
            marker.pose.position.y = current.py + global_y;
            marker.pose.position.z = current.pz + global_z;
            marker.pose.orientation.x = 1.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.text = "fire";
            marker.color.a = 0.9;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            mark.markers.push_back(marker);
            pub_marker->publish(mark);
            pub_all->publish(mark);
            //-------------------------------------
        }
    }
    //if (objects.list.size() > 0)
        pub->publish(objects);
}



/***************************************************************
 * depthMapCallback
 * * Subscribes to an image topic of depth data. Converts the
 * * depth map to opencv mat of encoding 32FC1 and saves it
 * * for use in republishYolo.
 ***************************************************************/

void depthMapCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        auto depthMap = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depthMapAccess.lock();
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            depthMap->image.convertTo(mDepthImage, -1, 0.001f);
        else
            mDepthImage = depthMap->image;
        depthMapAccess.unlock();
    }catch(cv_bridge::Exception& err){
        ROS_ERROR("cv_bridge exception: %s", err.what());
    }
}











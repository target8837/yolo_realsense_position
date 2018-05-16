/***************************************************************
 * Includes
 ***************************************************************/
#include <vector>
#include <string>


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


/***************************************************************
 * Global variables
 ***************************************************************/
ros::Publisher* pub;

cv_bridge::CvImagePtr depthMap;
std::mutex depthMapAccess;

double filterConst;
double maxThreshold;

/***************************************************************
 * Functions
 *
 * main
 * * Inits ros and sets up subscribers and publishers.
 *
 * republishYolo
 * * Subscribes to bounding box data from darknet_ros,
 * * reformats that and combibines it with distance data
 * * from a depth map. This fusion of data is then published
 * * in a new topic.
 *
 * depthMapCallback
 * * Subscribes to an image topic of depth data. Converts the
 * * depth map to opencv mat of encoding 32FC1 and saves it
 * * for use in republishYolo.
 ***************************************************************/

void republishYolo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void depthMapCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "relay");
    ros::NodeHandle com;
    
    com.param("settings/depth_filter", filterConst, 2.0);
    com.param("settings/threshold", maxThreshold, 0.8);

    std::string publisherTopicName;
    int publisherQueueSize;
    bool publisherLatch;
    com.param("publishers/object_data/topic", publisherTopicName, std::string("/yolo_depth_fusion/objects"));
    com.param("publishers/object_data/queue_size", publisherQueueSize, 1);
    com.param("publishers/object_data/latch", publisherLatch, false);
    ros::Publisher publisher = com.advertise<yolo_depth_fusion::yoloObjects>(publisherTopicName, publisherQueueSize, publisherLatch);
    pub = &publisher;
    
    int yoloQueueSize;
    std::string yoloTopicName;
    com.param("subscribers/bounding_boxes/topic", yoloTopicName, std::string("/darknet_ros/bounding_boxes"));
    com.param("subscribers/bounding_boxes/queue_size", yoloQueueSize, 1);
    ros::Subscriber yoloSub = com.subscribe(yoloTopicName, yoloQueueSize, republishYolo);


    int depthQueueSize;
    std::string depthTopicName;
    com.param("subscribers/depth_map/topic", depthTopicName, std::string("/zed/depth/depth_registered"));
    com.param("subscribers/depth_map/queue_size", depthQueueSize, 1);
    image_transport::ImageTransport imageLayer(com);
    image_transport::Subscriber depthSub = imageLayer.subscribe(depthTopicName, depthQueueSize, depthMapCallback);
    

    ros::spin();
}


void republishYolo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    yolo_depth_fusion::yoloObjects objects;
    objects.header = msg->header;
    objects.image_header = msg->image_header;
    
    depthMapAccess.lock();
    cv_bridge::CvImagePtr localMap = depthMap;
    depthMapAccess.unlock();
    
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

        cv::Scalar intensity = localMap->image.at<float>(current.y, current.x);
        float dist = intensity.val[0];
        double prob = (msg->bounding_boxes)[i].probability;
        //if (prob > maxThreshold || dist - filterConst / prob > 0.0) {
        {
            current.classification = (msg->bounding_boxes)[i].Class;
            current.probability = prob;
            current.distance = dist;
            objects.list.push_back(current);
        }
    }
    //if (objects.list.size() > 0)
        pub->publish(objects);
}

void depthMapCallback(const sensor_msgs::ImageConstPtr& msg){
    depthMapAccess.lock();
    try {
        depthMap = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }catch(cv_bridge::Exception& err){
        ROS_ERROR("cv_bridge exception: %s", err.what());
    }
    depthMapAccess.unlock();
}











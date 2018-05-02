/***************************************************************
 * Includes
 ***************************************************************/
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <limits>

#include "ros/ros.h"
#include <std_msgs/Header.h>

#include <yolo_depth_fusion/yoloObject.h>
#include <yolo_depth_fusion/yoloObjects.h>

#include "ms_coco.hpp"

/***************************************************************
 * Defines
 ***************************************************************/
#define HEIGHT 480
#define WIDTH  640
#define INITIAL_MAX 20
#define CHANGE 4





void republishYolo(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void depthMapCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "relay");
    ros::NodeHandle com;
    

    std::string publisherTopicName;
    int publisherQueueSize;
    bool publisherLatch;
    com.param("publishers/object_data/topic", publisherTopicName, std::string("/yolo_depth_fusion/objects"));
    com.param("publishers/object_data/queue_size", publisherQueueSize, 1);
    com.param("publishers/object_data/latch", publisherLatch, false);
    ros::Publisher pub = com.advertise<yolo_depth_fusion::yoloObjects>(publisherTopicName, publisherQueueSize, publisherLatch);
    std::srand (time(NULL));
    ros::Rate loop_rate(5);
    
    int newNumber = std::rand() % INITIAL_MAX;
    std::vector<yolo_depth_fusion::yoloObject> last;
    while (ros::ok()) {
        yolo_depth_fusion::yoloObjects objects;
        objects.header.stamp = ros::Time::now();
        objects.header.frame_id = "detection";
        
        int lastNumber = last.size();
        newNumber += std::rand() % (2*CHANGE) - CHANGE;
        std::random_shuffle(last.begin(), last.end());
        
        for (int i = 0; i < newNumber; i++) {
            
            if ( i < lastNumber ) {
                objects.list.push_back(last[i]);
            }
            else {
                yolo_depth_fusion::yoloObject current;
                current.classification = classes[rand() % classes.size()];
                
                current.probability = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
                current.x = rand() % WIDTH + 1;
                current.y = rand() % HEIGHT + 1;
                current.width = rand() % (WIDTH / 2) + 5;
                current.height = rand() % (HEIGHT / 2) + 5;
                current.distance = 0.7 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(20.0 - 0.7)));

                objects.list.push_back(current);               
            }

            objects.list[i].probability += static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(0.5))) - 0.25;
            
            objects.list[i].x += (rand() % (2*CHANGE)) - CHANGE;
            if (objects.list[i].x > WIDTH){ objects.list[i].x = WIDTH; }
            else if (objects.list[i].x < 1) objects.list[i].x = 1;

            objects.list[i].y += (rand() % (2*CHANGE)) - CHANGE;
            if (objects.list[i].y > HEIGHT){ objects.list[i].y = HEIGHT; }
            else if (objects.list[i].y < 1) objects.list[i].y = 1;

            objects.list[i].width += (rand() % (2*CHANGE)) - CHANGE;
            objects.list[i].height += (rand() % (2*CHANGE)) - CHANGE;

            objects.list[i].distance += static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.0))) - 0.5;
            if (objects.list[i].distance < 0.0){ objects.list[i].distance = 0.0; }
            else if (objects.list[i].distance > 25.0) objects.list[i].distance = 25.0;
        }

        last = objects.list;
        
        for (int i = 0; i < objects.list.size(); i++) {
            if (objects.list[i].distance < 0.7) { objects.list[i].distance = -std::numeric_limits<float>::infinity(); }
            else if (objects.list[i].distance > 20.0) objects.list[i].distance = std::numeric_limits<float>::infinity();
        }

        if (objects.list.size() > 0)
            pub.publish(objects);
    }
}






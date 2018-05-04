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



/***************************************************************
 * Defines
 ***************************************************************/
#define HEIGHT 376
#define WIDTH  672
#define INITIAL_MAX 20
#define CHANGE 4








std::vector<std::string> classes = {std::string("person"), std::string("bicycle"), std::string("car"), std::string("motorbike"), std::string("aeroplane"), std::string("bus"), std::string("train"), std::string("truck"), std::string("boat"), std::string("traffic light"), std::string("fire hydrant"), std::string("stop sign"), std::string("parking meter"), std::string("bench"), std::string("bird"), std::string("cat"), std::string("dog"), std::string("horse"), std::string("sheep"), std::string("cow"), std::string("elephant"), std::string("bear"), std::string("zebra"), std::string("giraffe"), std::string("backpack"), std::string("umbrella"), std::string("handbag"), std::string("tie"), std::string("suitcase"), std::string("frisbee"), std::string("skis"), std::string("snowboard"), std::string("sports ball"), std::string("kite"), std::string("baseball bat"), std::string("baseball glove"), std::string("skateboard"), std::string("surfboard"), std::string("tennis racket"), std::string("bottle"), std::string("wine glass"), std::string("cup"), std::string("fork"), std::string("knife"), std::string("spoon"), std::string("bowl"), std::string("banana"), std::string("apple"), std::string("sandwich"), std::string("orange"), std::string("broccoli"), std::string("carrot"), std::string("hot dog"), std::string("pizza"), std::string("donut"), std::string("cake"), std::string("chair"), std::string("sofa"), std::string("pottedplant"), std::string("bed"), std::string("diningtable"), std::string("toilet"), std::string("tvmonitor"), std::string("laptop"), std::string("mouse"), std::string("remote"), std::string("keyboard"), std::string("cell phone"), std::string("microwave"), std::string("oven"), std::string("toaster"), std::string("sink"), std::string("refrigerator"), std::string("book"), std::string("clock"), std::string("vase"), std::string("scissors"), std::string("teddy bear"), std::string("hair drier"), std::string("toothbrush")};










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






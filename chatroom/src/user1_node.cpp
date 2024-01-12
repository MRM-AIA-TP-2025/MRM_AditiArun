// chatroom/src/user1_node.cpp
#include "ros/ros.h"
#include "chatroom/ChatMessage.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user1_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<chatroom::ChatMessage>("chat", 10);

    while (ros::ok())
    {
        chatroom::ChatMessage msg;
        msg.username = "User1";
        std::cout << "User1, enter your message: ";
        std::getline(std::cin, msg.message);

        pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}


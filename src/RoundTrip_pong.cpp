// Copyright 2017 ADLINK Technology, Inc.
// Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" 
#include "adlink_msgs/msg/ping_pong.hpp"

using std::placeholders::_1;

class RoundTripPONG : public rclcpp::Node
{
    public:
    RoundTripPONG(int argc, char * argv[]) : Node("roundtrip_pong"), count_(0)
    {
        // Verify command arguments            
        self_id_ = 0;
        if (rcutils_cli_option_exist(argv, argv + argc, "-i"))
        {
            self_id_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-i"));
            if (self_id_ <= 0) 
                self_id_ = 0;
        }
        
        publisher_ = this->create_publisher<adlink_msgs::msg::PingPong>("roundtrip_pong", rmw_qos_profile_parameters); //topic, QoS
        subscription_ = this->create_subscription<adlink_msgs::msg::PingPong>("roundtrip_ping", 
                        std::bind(&RoundTripPONG::topic_callback, this, _1), rmw_qos_profile_parameters);
    }

private:
    void topic_callback(const adlink_msgs::msg::PingPong::SharedPtr msg)
    {
        msg_back_ = *msg;
        msg_back_.sender_id = self_id_;
        msg_back_.recver_id = msg->sender_id;         
        publisher_->publish(msg_back_);
        //std::cout << "I heard: " << msg->data.c_str() << ", Id: " << count_ << std::endl; //debug
        count_ ++;
    }

    rclcpp::Publisher<adlink_msgs::msg::PingPong>::SharedPtr publisher_;
    rclcpp::Subscription<adlink_msgs::msg::PingPong>::SharedPtr subscription_;
    adlink_msgs::msg::PingPong msg_back_;
    size_t count_, self_id_;
};


void print_usage()
{
    printf("ROS2 RoundTrip testing tool:\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-i self_id: Specify the No. of self node. [default=0].\n");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Print the help function
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) 
    {
        print_usage();
        return 0;
    }

    std::cout << "RoundTrip Testing: pong side !" << std::endl;
    auto pong = std::make_shared<RoundTripPONG>(argc, argv);
    rclcpp::spin(pong);
    rclcpp::shutdown();
    return 0;
}

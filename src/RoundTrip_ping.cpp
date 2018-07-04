// Copyright 2018 ADLINK Technology, Inc.
// Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
// Developer: Alan, CHEN (alan.chen@adlinktech.com)
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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip> // std::setw & setprecision
#include <memory>
#include <string>
#include <sys/stat.h>
#include <ctime>
#include <time.h>
#include <sys/types.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "adlink_msgs/msg/ping_pong.hpp"

#define MSG_HEADER_SIZE (4)

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

typedef struct _PacketRecord
{
    bool isReceived;
    time_point<high_resolution_clock> preWriteTime_;
    time_point<high_resolution_clock> postWriteTime_;
    time_point<high_resolution_clock> postReadTime_;
    double WriteAccessDuration;
    double RoundTripDuration;
    double OverallRoundTripDuration;
} PacketRecord, *pPacketRecord;

static int clamp(int v, int v_min, int v_max)
{
    return std::min(std::max(v_min, v), v_max);
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
static std::string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);
    std::string ret(buf);
    return ret;
}


class RoundTripPING : public rclcpp::Node
{
public:
    RoundTripPING(int argc, char* argv[])
        : Node("roundtrip_ping")
        , id_(0)
        , pub_count_(0)
        , isAlive_(false)
    {
        // verify command arguments

        // ID
        id_ = 0;
        if (rcutils_cli_option_exist(argv, argv + argc, "-i"))
        {
            id_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-i"));
        }

        // Frequency
        uint32_t hz = 10;
        if (rcutils_cli_option_exist(argv, argv + argc, "-f"))
        {
            // Min = 1 hz, Max = 1000 hz, default = 10 hz
            hz = atoi(rcutils_cli_get_option(argv, argv + argc, "-f"));
        }
        pub_timer_ = clamp(1000 / hz, 0, 1000);

        // Message Size(msg_size_)
        msg_size_ = MSG_HEADER_SIZE;
        if (rcutils_cli_option_exist(argv, argv + argc, "-p"))
        {
            // Min = 1 Byte, Max = 16 MB, default = 4Byte
            msg_size_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-p"));
            msg_size_ = clamp(msg_size_, 1, 16 * 1024 * 1024);
        }

        message_.sender_id = 0;
        message_.recver_id = 0xFF; // broadcast
        message_.packet_no = 0;
        message_.payload.resize(msg_size_ - MSG_HEADER_SIZE);

        // Pong Numbers
        pong_number_ = 1;
        if (rcutils_cli_option_exist(argv, argv + argc, "-s"))
        {
            // Min = 0 times
            pong_number_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-s"));
            pong_number_ = clamp(pong_number_, 0, pong_number_);
        }

        // Test Times
        max_loop_ = 100;
        if (rcutils_cli_option_exist(argv, argv + argc, "-t"))
        {
            // Min = 0 times
            max_loop_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-t"));
            max_loop_ = clamp(max_loop_, 0, max_loop_);
        }
        
        PacketRecord pr;
        pr.isReceived = false;
        pr.WriteAccessDuration = 0.0;
        pr.RoundTripDuration = 0.0;
        pr.OverallRoundTripDuration = 0.0;
        std::vector<PacketRecord> vpr(max_loop_, pr);
        pongPacketRecords_.resize(pong_number_, vpr);

        // Log File
        dir_name_ = "RoundTrip_" + currentDateTime() + '_';
        dir_name_ += std::to_string(hz) + "Hz" + '_';
        dir_name_ += std::to_string(msg_size_) + "Byte" + '_';
        dir_name_ += std::to_string(max_loop_) + "Times" + '_';
        dir_name_ += std::to_string(pong_number_) + "Pongs";
        char* cli_option = rcutils_cli_get_option(argv, argv + argc, "-l");
        if (nullptr != cli_option)
        {
            dir_name_ = std::string(cli_option);
        }

        // Debug Info Message
        debug_info_ = false;
        if (rcutils_cli_option_exist(argv, argv + argc, "-d"))
        {
            debug_info_ = true;
            std::cout << std::fixed << std::setprecision(3) << std::setw(20) 
                        << "Pong Number #" << std::setw(20)
                        << "WriteAccess duration" << std::setw(20)
                        << "RoundTrip duration" << std::setw(20) 
                        << "Overall RoundTrip duration" << std::endl;
        }

        // ROS2 Pub, Sub, Timer
        publisher_ = this->create_publisher<adlink_msgs::msg::PingPong>(
            "roundtrip_ping", rmw_qos_profile_parameters); // topic, QoS
            
        subscription_ = this->create_subscription<adlink_msgs::msg::PingPong>("roundtrip_pong",
            std::bind(&RoundTripPING::topic_callback, this, _1), rmw_qos_profile_parameters);
            
        timer_ = this->create_wall_timer(
            milliseconds(pub_timer_), std::bind(&RoundTripPING::timer_callback, this));
    }

    // Function for saving the log file
    void SaveLogfile()
    {
        const int dir_err = mkdir(dir_name_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == dir_err)
        {
            printf("Error creating directory!n");
            return;
        }

        std::cout << "========== Writing logfile ==========" << std::endl;

        for (int i = 0; i < pong_number_; ++i)
        {
            std::vector<PacketRecord>* vpr = &pongPacketRecords_[i];
            std::ofstream logfile;
            std::string filename = std::to_string(i + 1) + ".log";
            logfile.open(dir_name_ + '/' + filename);
            logfile << "RoundTripDuration,OverallRoundTripDuration\n";
            int loss_cnt = max_loop_;
            for (int j = 0; j < (int)vpr->size(); ++j)
            {
                PacketRecord* pr = &(*vpr)[j];
                
                // if debug_info_ is true, the delay will be calculate in topic_callback()
                if (debug_info_ == false) 
                {            
                    pr->WriteAccessDuration
                        = duration<double, std::milli>(pr->postWriteTime_ - pr->preWriteTime_).count();
                    pr->RoundTripDuration
                        = duration<double, std::milli>(pr->postReadTime_ - pr->postWriteTime_).count() / 2.0;
                    pr->OverallRoundTripDuration
                        = duration<double, std::milli>(pr->postReadTime_ - pr->preWriteTime_).count() / 2.0;
                }
                
                if (pr->isReceived)
                {
                    loss_cnt--;
                    logfile << pr->RoundTripDuration << "," << pr->OverallRoundTripDuration << "\n";
                }
            }
            std::cout << "Packet Lost of No." << i + 1 << "= " << loss_cnt
                      << "packet @rate = " << 100 * loss_cnt / (double)max_loop_ << "%"
                      << std::endl;
            logfile.close();
            std::cout << "File saved! Name: " << filename << std::endl;
        }
    } // end of func

    bool IsAlive()
    {
        /* debug message */
        return isAlive_;
    }

    void ResetAlive()
    {
        /* debug message */
        isAlive_ = false;
    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<adlink_msgs::msg::PingPong>::SharedPtr publisher_;
    rclcpp::Subscription<adlink_msgs::msg::PingPong>::SharedPtr subscription_;
    adlink_msgs::msg::PingPong message_ = adlink_msgs::msg::PingPong();

    int id_, pub_count_, pub_timer_, msg_size_, max_loop_, pong_number_;
    bool debug_info_, isAlive_;

    std::string dir_name_;
    std::vector<std::vector<PacketRecord> > pongPacketRecords_;

    // Callback function for timer
    void timer_callback()
    {
        if (pub_count_ >= max_loop_)
            return;

        message_.packet_no = pub_count_;

        time_point<high_resolution_clock> pre = high_resolution_clock::now();
        publisher_->publish(message_);
        time_point<high_resolution_clock> post = high_resolution_clock::now();

        for (int i = 0; i < pong_number_; ++i)
        {
            PacketRecord* pr = &pongPacketRecords_[i][pub_count_];
            pr->preWriteTime_ = pre;
            pr->postWriteTime_ = post;
        }

        pub_count_++;
    } // end of func


    // Callback function for subscription
    void topic_callback(const adlink_msgs::msg::PingPong::SharedPtr msg)
    {
        time_point<high_resolution_clock> tmp = high_resolution_clock::now();

        if (msg->sender_id - 1 >= pong_number_)
            return;

        PacketRecord* pr = &pongPacketRecords_[msg->sender_id - 1][msg->packet_no];
        pr->isReceived = true;
        pr->postReadTime_ = tmp;

        if (debug_info_)
        {            
            pr->WriteAccessDuration
                = duration<double, std::milli>(pr->postWriteTime_ - pr->preWriteTime_).count();
            pr->RoundTripDuration
                = duration<double, std::milli>(pr->postReadTime_ - pr->postWriteTime_).count() / 2.0;
            pr->OverallRoundTripDuration
                = duration<double, std::milli>(pr->postReadTime_ - pr->preWriteTime_).count() / 2.0;
        
            std::cout << std::fixed << std::setprecision(3) << std::setw(20)
                        << msg->sender_id << std::setw(20) 
                        << pr->WriteAccessDuration << std::setw(20)
                        << pr->RoundTripDuration << std::setw(20) 
                        << pr->OverallRoundTripDuration << std::endl;
        }
        isAlive_ = true;
    } // end of func
};


void print_usage()
{
    printf("ROS2 RoundTrip testing tool:\n");
    printf("options:\n");
    printf("-d : debug_info. Showing the debug info.\n");
    printf("-f : frequency. Specify the publishing frequency(hz). [default=10]\n");
    printf("-h : Print this help function.\n");
    printf("-i : identification. Specify the id of this ROS node. [default=0]\n");
    printf("-l : logfile_name. Specify the logfile name. [default=time_stamp].\n");
    printf("-p : packet_size. Specify the size of payload for publishing (Byte) [default=4].\n");
    printf("-s : pong_number_. Specify the number of pongs for single ping and multiple pong test [default=1].\n");
    printf("-t : test_times. Specify the number of loops for testing [default=100].\n");
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // countdown timer for testing timeout
    std::clock_t start;
    double duration;

    // Print the help function
    if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
    {
        print_usage();
        return 0;
    }

    auto ping = std::make_shared<RoundTripPING>(argc, argv);
    start = std::clock();

    while (1)
    {
        rclcpp::spin_some(ping);

        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        if (duration > 10.0)
        {
            printf("No packet arrived in 10 sec ... Timeout!\n");
            break;
        }

        if (ping->IsAlive())
        {
            start = std::clock();
            ping->ResetAlive();
        }
    }

    rclcpp::shutdown();
    ping->SaveLogfile();
    // ping->ShowResult();
    return 0;
}

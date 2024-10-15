/**
 * @file bag_recorder_node.cpp
 * @author Casey Nichols (casey.nichols@nrel.gov)
 * @brief This package enables more control over the ros bag record process such as the ability to start and stop recording
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include "modaq_messages/msg/bagcontrol.hpp"
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <ctime>
using namespace std::chrono_literals;
using std::placeholders::_1;
/**
 * @brief main class which handles the control of the bag recorder
 * 
 */
class BagRecorder : public rclcpp::Node
{
public:
/**
 * @brief Construct a new Bag Recorder object
 * 
 */
  BagRecorder()
      : Node("BagRecorder")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("dataFolder", "/home/m2/Data");
    this->declare_parameter<int>("fileDuration", 60);
    this->declare_parameter<std::vector<std::string>>("loggedTopics", {"/rosout", "/system_messenger", "/labjack_ain"});
    bagctrl_sub = create_subscription<modaq_messages::msg::Bagcontrol>("/bag_control", 10, std::bind(&BagRecorder::control_callback, this, _1));

    // Get parameter values from config file
    data_folder_ = this->get_parameter("dataFolder").as_string();
    file_duration_ = this->get_parameter("fileDuration").as_int();
    logged_topics_ = this->get_parameter("loggedTopics").as_string_array();

  }
/**
 * @brief Get the recorder object
 * 
 * @return std::shared_ptr<rosbag2_transport::Recorder> 
 */
  std::shared_ptr<rosbag2_transport::Recorder> get_recorder() const
  {
    return recorder_;
  }
/**
 * @brief Set the executor object
 * 
 * @param executorSet 
 */
  void set_executor(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executorSet)
  {
    executor = executorSet;
  }
/**
 * @brief Destroy the Bag Recorder object
 * 
 */
  ~BagRecorder()
  {
    stop_recording();
  }
/**
 * @brief spins up a new node for the bag recorder
 * 
 */
  void start_recording()
  {

    // Log parameter values for verification
    std::cout << "Data Folder: " << data_folder_ << std::endl;
    std::cout << "File Duration: " << file_duration_ << " seconds" << std::endl;
    std::cout << "Logged Topics: ";
    for (const auto &topic : logged_topics_)
    {
      std::cout << topic << " ";
    }
    std::cout << std::endl;

    // Set storage options
    storage_options_.uri = getDataPath(data_folder_);
    storage_options_.storage_id = "mcap";
    storage_options_.max_bagfile_size = 0;
    storage_options_.max_bagfile_duration = std::chrono::seconds(file_duration_).count();
    storage_options_.max_cache_size = 10485760;
    storage_options_.storage_preset_profile = "";
    storage_options_.snapshot_mode = false;

    std::cout << "Storage Path: " << storage_options_.uri << std::endl;
  
    writer_ = std::make_shared<rosbag2_cpp::Writer>();
    rosbag2_transport::RecordOptions record_options;

    // Set record options
    if (logged_topics_[0] == "*") // handle record all topics
    {
      std::vector<std::string> emptyTopics;
      record_options = {
          true, false, emptyTopics, "cdr", std::chrono::milliseconds(1000)};
    }
    else
    {
      record_options = {
          false, false, logged_topics_, "cdr", std::chrono::milliseconds(1000)};
    }
    // Initialize recorder with unique node name
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        writer_, storage_options_, record_options);

    // Start recording
    executor->add_node(recorder_);
    recorder_->record();
    recording = true;
  }

private:
/**
 * @brief callback function to control the start and stop of bag recording
 * 
 * @param msg the ros message which has the control data
 */
  void control_callback(modaq_messages::msg::Bagcontrol msg)
  {
    RCLCPP_INFO(this->get_logger(), "Message Received");
    if (msg.enable_recording && !recording)
    {
      start_recording();
      RCLCPP_INFO(this->get_logger(), "Message Received: starting new recording");
    }
    if (!msg.enable_recording && recording)
    {
      stop_recording();
      RCLCPP_INFO(this->get_logger(), "Message Received: stopping existing recording");
    }
  }
/**
 * @brief stops the bag recorder node
 * 
 */
  void stop_recording()
  {
    if (recording)
    {
      recorder_->stop();
      executor->remove_node(recorder_);
      recording = false;
    }
  }
/**
 * @brief stops and restarts the bag recorder node
 * 
 */
  void reset_bag_recording()
  {
    stop_recording();

    start_recording();
  }


  rclcpp::TimerBase::SharedPtr reset_bag_timer; /// TODO: timer used for automatically resetting the bag recorder
  std::string data_folder_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  int file_duration_;
  std::vector<std::string> logged_topics_;
  rosbag2_storage::StorageOptions storage_options_;
  rclcpp::NodeOptions node_options_;
  rclcpp::Subscription<modaq_messages::msg::Bagcontrol>::SharedPtr bagctrl_sub;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  bool recording = false;

  std::string getCurrentUtcTime()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm = *std::gmtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y_%m_%d_%H_%M_%S");
    return oss.str();
  }

  std::string getDataPath(const std::string &baseFolder)
  {
    std::string systemStartTime = getCurrentUtcTime();
    return baseFolder + "/Bag_" + systemStartTime;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto bag_recorder_node = std::make_shared<BagRecorder>();

  // Use a MultiThreadedExecutor to spin the BagRecorder and Recorder nodes
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(bag_recorder_node);
  bag_recorder_node->set_executor(executor);
  bag_recorder_node->start_recording();

  executor->spin();

  rclcpp::shutdown();
  return 0;
}

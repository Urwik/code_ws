// cpp
#include <iostream>
#include <boost/filesystem.hpp>
#include <experimental/filesystem>
#include <typeinfo>
#include <string>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <map>
#include <chrono>
#include <ctime>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <set>
#include <vector>

// Function to list all topics in the bag file with their message types
std::map<std::string, std::string> list_topics_with_types(rosbag::Bag& bag) {
  rosbag::View view(bag);
  std::map<std::string, std::string> topics_types;
  for (const auto& conn_info : view.getConnections()) {
    topics_types[conn_info->topic] = conn_info->datatype;
  }
  return topics_types;
}

// Function to prompt user to select topics
std::vector<std::string> select_topics(const std::map<std::string, std::string>& topics_types) {
  std::vector<std::string> selected;
  std::vector<std::pair<std::string, std::string>> topic_list(topics_types.begin(), topics_types.end());
  
  std::cout << "Available topics:\n";
  for (size_t i = 0; i < topic_list.size(); ++i) {
    std::cout << i + 1 << ": " << topic_list[i].first << " (" << topic_list[i].second << ")\n";
  }
  
  std::cout << "Enter topic numbers to extract (comma separated): ";
  std::string input;
  std::getline(std::cin, input);
  std::stringstream ss(input);
  std::string token;
  
  while (std::getline(ss, token, ',')) {
    // Remove whitespace
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
    if (!token.empty()) {
      size_t idx = std::stoi(token) - 1;
      if (idx < topic_list.size()) {
        selected.push_back(topic_list[idx].first);
      }
    }
  }
  return selected;
}

// Function to convert timestamp to string
std::string timestamp_to_string(const ros::Time& timestamp) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(9) << timestamp.toSec();
  return ss.str();
}

// Function to get current date and time as string
std::string get_current_datetime() {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
  ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

// Function to create output directory if it doesn't exist
void create_output_directory(const std::string& dir_name) {
  boost::filesystem::path dir(dir_name);
  if (!boost::filesystem::exists(dir)) {
    boost::filesystem::create_directories(dir);
  }
}

// Function to save image message
void save_image(const sensor_msgs::Image::ConstPtr& img_msg, const std::string& output_dir, const ros::Time& timestamp) {
  try {
    cv_bridge::CvImagePtr cv_ptr;
    
    // Handle different image encodings
    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8 || 
        img_msg->encoding == sensor_msgs::image_encodings::RGB8 ||
        img_msg->encoding == sensor_msgs::image_encodings::BGRA8 ||
        img_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    } else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    } else {
      // Try to convert to BGR8, fallback to original encoding if it fails
      try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
        cv_ptr = cv_bridge::toCvCopy(img_msg);
      }
    }
    
    std::string filename = output_dir + "/" + timestamp_to_string(timestamp) + ".png";
    cv::imwrite(filename, cv_ptr->image);
    std::cout << "Saved image: " << filename << " (encoding: " << img_msg->encoding << ")" << std::endl;
  } catch (cv_bridge::Exception& e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }
}

// Function to save point cloud message
void save_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const std::string& output_dir, const ros::Time& timestamp) {
  try {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    std::string filename = output_dir + "/" + timestamp_to_string(timestamp) + ".pcd";
    pcl::io::savePCDFileASCII(filename, cloud);
    std::cout << "Saved point cloud: " << filename << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error saving point cloud: " << e.what() << std::endl;
  }
}

// Function to extract selected topics and save them as files
void extract_and_save_topics(rosbag::Bag& bag, const std::vector<std::string>& topics, const std::map<std::string, std::string>& topics_types) {
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Create root extraction directory with current datetime
  std::string root_extraction_dir = "extraction_" + get_current_datetime();
  create_output_directory(root_extraction_dir);
  std::cout << "Created extraction directory: " << root_extraction_dir << std::endl;
  
  // Create output directories for each topic inside the root extraction directory
  for (const auto& topic : topics) {
    std::string clean_topic = topic;
    std::replace(clean_topic.begin(), clean_topic.end(), '/', '_');
    if (clean_topic[0] == '_') clean_topic = clean_topic.substr(1);
    std::string topic_dir = root_extraction_dir + "/" + clean_topic;
    create_output_directory(topic_dir);
  }
  
  int message_count = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
    std::string clean_topic = topic;
    std::replace(clean_topic.begin(), clean_topic.end(), '/', '_');
    if (clean_topic[0] == '_') clean_topic = clean_topic.substr(1);
    
    // Create full path with root extraction directory
    std::string full_topic_path = root_extraction_dir + "/" + clean_topic;
    
    std::string message_type = topics_types.at(topic);
    
    if (message_type == "sensor_msgs/Image") {
      sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
      if (img_msg != nullptr) {
        save_image(img_msg, full_topic_path, m.getTime());
      }
    } 
    else if (message_type == "sensor_msgs/PointCloud2") {
      sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (cloud_msg != nullptr) {
        save_pointcloud(cloud_msg, full_topic_path, m.getTime());
      }
    }
    else {
      std::cout << "Unsupported message type for topic " << topic << ": " << message_type << std::endl;
    }
    
    message_count++;
    if (message_count % 10 == 0) {
      std::cout << "Processed " << message_count << " messages..." << std::endl;
    }
  }
  
  std::cout << "Extraction completed. Total messages processed: " << message_count << std::endl;
}



int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_file>" << std::endl;
    return 1;
  }

  rosbag::Bag bag;

  try {
    bag.open(argv[1]);
    std::cout << "Successfully opened bag file: " << argv[1] << std::endl;
    
    // List all topics with their types
    auto topics_types = list_topics_with_types(bag);
    
    if (topics_types.empty()) {
      std::cout << "No topics found in the bag file." << std::endl;
      return 1;
    }
    
    // Let user select topics
    auto selected_topics = select_topics(topics_types);
    
    if (selected_topics.empty()) {
      std::cout << "No topics selected. Exiting." << std::endl;
      return 1;
    }
    
    std::cout << "\nSelected topics:" << std::endl;
    for (const auto& topic : selected_topics) {
      std::cout << "- " << topic << " (" << topics_types[topic] << ")" << std::endl;
    }
    
    std::cout << "\nStarting extraction..." << std::endl;
    extract_and_save_topics(bag, selected_topics, topics_types);
    
    bag.close();
  }
  catch(const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unordered_set>
#include <vector>

using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObject;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using std::placeholders::_1;

class ObjectsFilterNode : public rclcpp::Node {
public:
  ObjectsFilterNode() : Node("objects_filter_node") {
    // パラメータ宣言
    this->declare_parameter<std::string>("input_topic", "/objects_on_route_preprocess");
    this->declare_parameter<std::string>("output_topic", "/objects_on_route");
    this->declare_parameter<std::vector<int>>("excluded_labels", std::vector<int>{});
    this->declare_parameter<std::string>("output_marker_topic", "/objects_marker_filtered");

    this->declare_parameter<std::string>("marker_ex_class_ns", "objects_ex_class");
    this->declare_parameter<std::string>("marker_ex_pose_ns", "objects_ex_pose");
    this->declare_parameter<std::string>("marker_filtered_ns", "objects_marker_filtered");

    // パラメータ取得
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    auto label_list = this->get_parameter("excluded_labels").as_integer_array();
    // int64 → int に変換してセットに格納
    for (const auto &label : label_list) {
      excluded_labels_.insert(static_cast<int>(label));
    }
    std::string output_marker_topic = this->get_parameter("output_marker_topic").as_string();

    marker_ex_class_ns_ = this->get_parameter("marker_ex_class_ns").as_string();
    marker_ex_pose_ns_ = this->get_parameter("marker_ex_pose_ns").as_string();
    marker_filtered_ns_ = this->get_parameter("marker_filtered_ns").as_string();

    // パブリッシャー・サブスクライバー設定
    pub_ = this->create_publisher<DetectedObjects>(output_topic, 10);
    sub_ = this->create_subscription<DetectedObjects>(input_topic, 10, std::bind(&ObjectsFilterNode::callback, this, _1));

    // デバッグマーカー用パブリッシャーとタイマー
    marker_pub_ = this->create_publisher<MarkerArray>(output_marker_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
  }

private:
  std::unordered_set<int> excluded_labels_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  std::string marker_ex_class_ns_;
  std::string marker_ex_pose_ns_;
  std::string marker_filtered_ns_;

  void callback(const DetectedObjects::SharedPtr msg) {
    DetectedObjects filtered;
    filtered.header = msg->header;

    MarkerArray marker_array;
    int id = 0;
    for (const auto &obj : msg->objects) {
      if (obj.classification.empty()) continue;

      uint8_t label = obj.classification.front().label;

      Marker marker;
      marker.header.frame_id=msg->header.frame_id;
      marker.header.stamp=msg->header.stamp;
      marker.type = Marker::CUBE;
      marker.action = Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.a = 0.5;
      marker.pose.position = obj.kinematics.pose_with_covariance.pose.position;
      marker.pose.orientation.w = 1.0;

      // ラベルが除外対象か判定
      if (excluded_labels_.count(label) > 0) {
        marker.id = id++;
        marker.ns=marker_ex_class_ns_;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
        continue;
      }

      // カメラ座標系：右側（x > 0）を除外
      const auto &pos = obj.kinematics.pose_with_covariance.pose.position;
      if (pos.x > 0.0) {
        marker.id = id++;
        marker.ns=marker_ex_pose_ns_;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker_array.markers.push_back(marker);
        continue;
      }
      filtered.objects.push_back(obj);
      marker.id = id++;
      marker.ns=marker_filtered_ns_;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);

    }
    pub_->publish(filtered);
    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectsFilterNode>());
  rclcpp::shutdown();
  return 0;
}

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

class ObjectFilterNode : public rclcpp::Node {
public:
  ObjectFilterNode() : Node("object_filter_node") {
    // パラメータ宣言
    this->declare_parameter<std::string>("input_topic", "/objects_on_route");
    this->declare_parameter<std::string>("output_topic", "/objects_filtered");
    this->declare_parameter<std::vector<int>>("excluded_labels", std::vector<int>{});

    // パラメータ取得
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    auto label_list = this->get_parameter("excluded_labels").as_integer_array();

    for (const auto &label : label_list) {
      excluded_labels_.insert(static_cast<int>(label));
    }

    // 通常のパブリッシャー・サブスクライバー設定
    pub_ = this->create_publisher<DetectedObjects>(output_topic, 10);
    sub_ = this->create_subscription<DetectedObjects>(input_topic, 10, std::bind(&ObjectFilterNode::callback, this, _1));

    // デバッグマーカー用パブリッシャーとタイマー
    marker_pub_ = this->create_publisher<MarkerArray>("/debug/filtered_objects/invalid_direction", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ObjectFilterNode::publishDebugMarkers, this));

    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
  }

private:
  std::unordered_set<int> excluded_labels_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::Point> invalid_points_;
  std_msgs::msg::Header last_header_;

  void callback(const DetectedObjects::SharedPtr msg) {
    DetectedObjects filtered;
    filtered.header = msg->header;
    last_header_ = msg->header;
    invalid_points_.clear();

    for (const auto &obj : msg->objects) {
      if (obj.classification.empty()) continue;

      uint8_t label = obj.classification.front().label;

      // ラベルが除外対象か判定
      if (excluded_labels_.count(label) > 0) continue;

      const auto &pos = obj.kinematics.pose_with_covariance.pose.position;
      if (pos.y < 0.0) {
        // デバッグ用に記録
        geometry_msgs::msg::Point pt;
        pt.x = pos.x;
        pt.y = pos.y;
        pt.z = pos.z;
        invalid_points_.push_back(pt);
        continue;
      }

      filtered.objects.push_back(obj);
    }

    pub_->publish(filtered);
  }

  void publishDebugMarkers() {
    MarkerArray marker_array;
    Marker marker;
    marker.header.frame_id = "front_wide_camera_optical_link";
    marker.ns = "invalid_direction";
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    int id = 0;

    // 座標範囲の定義（y < 0 の領域）
    const double min_x = 0.0, max_x = 10.0;
    const double min_y = -10.0, max_y = -0.1;
    const double min_z = -2.0, max_z = 2.0;
    const double step = 1.0;  // 間隔（密度）

    for (double x = min_x; x <= max_x; x += step) {
      for (double y = min_y; y <= max_y; y += step) {
        for (double z = min_z; z <= max_z; z += step) {
          marker.id = id++;
          marker.pose.position.x = x;
          marker.pose.position.y = y;
          marker.pose.position.z = z;
          marker.pose.orientation.w = 1.0;
          marker_array.markers.push_back(marker);
        }
      }
    }

    marker_pub_->publish(marker_array);
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFilterNode>());
  rclcpp::shutdown();
  return 0;
}

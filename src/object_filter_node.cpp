#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <unordered_set>
#include <vector>

using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::DetectedObject;
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
    // int64 → int に変換してセットに格納
    for (const auto &label : label_list) {
      excluded_labels_.insert(static_cast<int>(label));
    }
    // パブリッシャー・サブスクライバー設定
    pub_ = this->create_publisher<DetectedObjects>(output_topic, 10);
    sub_ = this->create_subscription<DetectedObjects>(input_topic, 10, std::bind(&ObjectFilterNode::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
  }

private:
  std::unordered_set<int> excluded_labels_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_;

  void callback(const DetectedObjects::SharedPtr msg) {
    DetectedObjects filtered;
    filtered.header = msg->header;

    for (const auto &obj : msg->objects) {
      if (obj.classification.empty()) continue;

      uint8_t label = obj.classification.front().label;

      // ラベルが除外対象か判定
      if (excluded_labels_.count(label) > 0) {
        continue;
      }

      // カメラ座標系：右側（y < 0）を除外
      const auto &pos = obj.kinematics.pose_with_covariance.pose.position;
      if (pos.y < 0.0) continue;
      filtered.objects.push_back(obj);
    }

    pub_->publish(filtered);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFilterNode>());
  rclcpp::shutdown();
  return 0;
}
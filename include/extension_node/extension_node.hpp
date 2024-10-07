#pragma once
#include <iostream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <data_logger_utils/log_publisher.hpp>

namespace ext_rclcpp {
  class ExtensionNode : public rclcpp::Node {
  public:
    data_logger::LogPublisher log_;
    explicit ExtensionNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, options) {
      file_name_ = node_name;
      time_sync_ = false;
      log_.set_node(this);
    }

    explicit ExtensionNode(const std::string& node_name, const std::string& namespace_, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, namespace_, options) {
      file_name_ = node_name;
      time_sync_ = false;
      log_.set_node(this);
    }

    /**
     * @brief clock topicでの時刻同期を有効にする
     *
     * @param clock_topic_name
     * @param qos
     */
    void set_time_sync(const std::string& clock_topic_name, const rclcpp::QoS& qos) {
      time_sync_ = true;
      clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(clock_topic_name, qos,
                                                                        [&](const rosgraph_msgs::msg::Clock::SharedPtr msg) { clock_ = msg->clock; });
    }

    /**
     * @brief 現在時刻取得(時刻同期有効時はclock topicの時刻)
     *
     * @return rclcpp::Time
     */
    rclcpp::Time get_now() {
      if (time_sync_) {
        if (clock_)
          return clock_.value();
        else
          RCLCPP_WARN(this->get_logger(), "Unable to obtain time");
      }
      return now();
    }

    /**
     * @brief パラメータ取得関数
     *
     * @tparam T
     * @param name
     * @param def
     * @return T
     */
    template <class T>
    T param(const std::string& name, const T& def) {
      T value;
      declare_parameter(name, def);
      get_parameter(name, value);
      return value;
    }

    /**
     * @brief rosのparam関数の互換
     *
     * @tparam T
     * @param name
     * @param value
     * @param def
     */
    template <class T>
    void param(const std::string& name, T& value, const T& def) {
      declare_parameter(name, def);
      get_parameter(name, value);
    }

    /**
     * @brief data_loggerの初期化(ファイル名指定)
     *
     * @param name : file name
     * @param column_names : colmnのname list
     */
    void init_data_logger(const std::string& name, const std::vector<std::string>& column_names) {
      log_.init_data_logger(name, column_names);
    }
    /**
     * @brief data_loggerの初期化
     *
     * @param column_names : colmnのname list
     */
    void init_data_logger(const std::vector<std::string>& column_names) { init_data_logger(file_name_, column_names); }

    /**
     * @brief data_loggerへlogをpublish
     *
     * @param args : 記録する値
     */
    template <class... Args>
    void log(Args... args) {
      log_.publish(args...);
    }

    /**
     * @brief 変化が合った時に出力するINFO関数
     *
     * @param num (0~)
     * @param logger
     * @param fmt
     * @param args
     */
    void RCLCPP_INFO_CHANGE(int num, rclcpp::Logger logger, const char* fmt, auto... args) {
      std::string data = format(fmt, args...);
      if ((num + 1) > mem_strs_.size()) {
        mem_strs_.push_back(data);
        RCLCPP_INFO(logger, "%s", data.c_str());
      } else {
        if (mem_strs_[num] != data) {
          mem_strs_[num] = data;
          RCLCPP_INFO(logger, "%s", data.c_str());
        }
      }
    }

  private:
    bool time_sync_ = false;
    std::string file_name_;
    std::vector<std::string> mem_strs_;
    std::optional<rclcpp::Time> clock_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  };
} // namespace ext_rclcpp
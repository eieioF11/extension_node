#pragma once
#include <iostream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace common_lib {
  class ExtensionNode : public rclcpp::Node {
  public:
    explicit ExtensionNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, options) {
      file_name_ = node_name;
    }

    explicit ExtensionNode(const std::string& node_name, const std::string& namespace_, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, namespace_, options) {
      file_name_ = node_name;
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
      init_log_pub_ = create_publisher<std_msgs::msg::String>("/data_logger/init", rclcpp::QoS(10).reliable());
      log_pub_      = create_publisher<std_msgs::msg::String>("/data_logger/log", rclcpp::QoS(10).reliable());
      std_msgs::msg::String msg;
      msg.data = name + ",";
      for (const auto& column_name : column_names)
        msg.data += column_name + ",";
      init_log_pub_->publish(msg);
    }
    /**
     * @brief data_loggerの初期化
     *
     * @param column_names : colmnのname list
     */
    void init_data_logger(const std::vector<std::string>& column_names) { init_data_logger(file_name_, column_names); }

    /**
     * @brief data_loggerへlogをpublish(ファイル名指定)
     *
     * @param name : file name
     * @param args : 記録する値
     */
    template <class... Args>
    void log(const std::string& name, Args... args) {
      std_msgs::msg::String msg;
      msg.data = name + ",";
      for (auto&& x : {args...})
        msg.data += std::to_string(x) + ",";
      log_pub_->publish(msg);
    }

    /**
     * @brief data_loggerへlogをpublish
     *
     * @param args : 記録する値
     */
    template <class... Args>
    void log(Args... args) {
      log(file_name_, args...);
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
    std::string file_name_;
    std::vector<std::string> mem_strs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_log_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  };
} // namespace common_lib
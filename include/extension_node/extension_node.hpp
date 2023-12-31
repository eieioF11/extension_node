#pragma once
#include <string>
#include <iostream>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace common_lib
{
    class ExtensionNode : public rclcpp::Node
    {
        public:
            explicit ExtensionNode(
                const std::string &node_name,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : rclcpp::Node(node_name, options) {}

            explicit ExtensionNode(
                const std::string &node_name, const std::string &namespace_,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : rclcpp::Node(node_name, namespace_, options) {}

            /**
             * @brief パラメータ取得関数
             *
             * @tparam T
             * @param name
             * @param def
             * @return T
             */
            template <class T>
            T param(const std::string &name, const T &def)
            {
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
            void param(const std::string &name,T &value, const T &def)
            {
                declare_parameter(name, def);
                get_parameter(name, value);
            }

            /**
             * @brief 変化が合った時に出力するINFO関数
             *
             * @param logger
             * @param fmt
             * @param args
             */
            void RCLCPP_INFO_CHANGE(rclcpp::Logger logger, const char *fmt, auto... args)
            {
                std::string data = format(fmt, args...);
                if (mem_str_ != data) {
                    mem_str_ = data;
                    RCLCPP_INFO(logger, "%s", data.c_str());
                }
            }

        private:
            std::string mem_str_;
    };
} // namespace common_lib
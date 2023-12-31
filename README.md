# extension_node
拡張機能付きノード
## example
### パラメータ取得

```c++
#pragma once
#include <iostream>
#include "extension_node/extension_node.hpp"

using namespace std::chrono_literals;
class ExampleNode : public ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ExtensionNode("icp_scan_matcher_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start node");
		// get param
		std::string PARAM1 = param<std::string>("param1", "str");
		double PARAM2 = param<double>("param2", 0.001);
		int PARAM3 = param<int>("param2", 0.001);
	}
};
```

```c++
#pragma once
#include <iostream>
#include "extension_node/extension_node.hpp"

using namespace std::chrono_literals;
class ExampleNode : public ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ExtensionNode("icp_scan_matcher_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start node");
		// get param
		param<std::string>("param1",PARAM1, "str");
		param<double>("param2",PARAM2, 0.001);
		param<int>("param2",PARAM3, 0.001);
	}
private:
	std::string PARAM1;
	double PARAM2;
	int PARAM3;
};
```

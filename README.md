# extension_node

rclcpp::Node extension library

## include

```c++
#include "extension_node/extension_node.hpp"
```

## package.xml

```xml
		︙
  <depend>extension_node</depend> <!-- ← add -->
		︙
```

## example

### Get Parameters

```c++
#pragma once
#include <iostream>
#include "extension_node/extension_node.hpp"

using namespace std::chrono_literals;
class ExampleNode : public ext_rclcpp::ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ext_rclcpp::ExtensionNode("example_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
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
class ExampleNode : public ext_rclcpp::ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ext_rclcpp::ExtensionNode("example_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
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

### Time Synchronization

#### Time publish side

```C++
#pragma once
#include <iostream>
#include "extension_node/extension_node.hpp"

using namespace std::chrono_literals;
class ExampleNode : public ext_rclcpp::ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ext_rclcpp::ExtensionNode("example_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start node");
		//  Time Publish Settings
		publish_clock("/example_clock", 1ms,rclcpp::QoS(10).reliable()); //(Topic name, transmission cycle, QoS)
	}
};
```

#### Time acquisition side

```C++
#pragma once
#include <iostream>
#include "extension_node/extension_node.hpp"

using namespace std::chrono_literals;
class ExampleNode : public ext_rclcpp::ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ext_rclcpp::ExtensionNode("example_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start node");
		// Time Synchronization Settings
		set_time_sync("/example_clock", rclcpp::QoS(10).reliable()); //(Topic name, QoS)
		︙
		// Synchronized time acquisition
		auto t = this->get_now();
	}
};
```

### Using data_logger

```C++
#pragma once
#include <iostream>
#include "extension_node/extension_node.hpp"

using namespace std::chrono_literals;
class ExampleNode : public ext_rclcpp::ExtensionNode
{
public:
	ExampleNode(const rclcpp::NodeOptions &options) : ExampleNode("", options) {}
	ExampleNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ext_rclcpp::ExtensionNode("example_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start node");
		//Initialization of data_logger
		log_.init_data_logger({"column1", "column2", "column3"});
		︙
		//data set
    log_.set("column1", value1);
    log_.set("column2", value2);
    log_.set("column3", value3);
		//log publish
		log_.publish();
	}
};
```

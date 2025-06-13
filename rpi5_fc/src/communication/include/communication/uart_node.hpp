#ifndef UART_NODE_HPP
#define UART_NODE_HPP

#include <rclcpp/rclcpp.hpp>

class UART_NODE : public rclcpp::Node
{
    private:
    UART_NODE(std::string& node_name);

};


#endif
#include <memory>
#include <string>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h" 

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;

// --- 1. ConditionNode: is GPS Online? ---
class CheckGPS : public BT::ConditionNode {
public:
    CheckGPS(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { 
        return { BT::InputPort<bool>("gps_online") }; 
    }

    BT::NodeStatus tick() override {
        bool online;
        if (!getInput("gps_online", online) || !online) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

// --- 2. ActionNode: Set Fusion Weights ---
class SetFusionWeights : public BT::SyncActionNode {
public:
    SetFusionWeights(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
        // Client per il servizio set_parameters del nodo Complementary Filter
        client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/complementary_filter_node/set_parameters");
    }

    static BT::PortsList providedPorts() { 
        return { 
            BT::InputPort<double>("a1"), 
            BT::InputPort<double>("a2"), 
            BT::InputPort<double>("a3") 
        }; 
    }

    BT::NodeStatus tick() override {
        double a1, a2, a3;
        if (!getInput("a1", a1) || !getInput("a2", a2) || !getInput("a3", a3)) {
            RCLCPP_ERROR(node_->get_logger(), "SetFusionWeights: Missing ports!");
            return BT::NodeStatus::FAILURE;
        }

        if (!client_->wait_for_service(500ms)) {
            RCLCPP_WARN(node_->get_logger(), "Servizio parametri CF non disponibile.");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        
        // Creation of parameters alpha1, alpha2, alpha3
        auto make_param = [](std::string name, double val) {
            rcl_interfaces::msg::Parameter p;
            p.name = name;
            p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            p.value.double_value = val;
            return p;
        };

        request->parameters.push_back(make_param("alpha1", a1));
        request->parameters.push_back(make_param("alpha2", a2));
        request->parameters.push_back(make_param("alpha3", a3));

        auto result = client_->async_send_request(request);
        // Being a SyncActionNode, we return SUCCESS immediately or handle the callback if necessary
        RCLCPP_INFO(node_->get_logger(), "BT: Parametri inviati [a1:%.2f, a2:%.2f, a3:%.2f]", a1, a2, a3);
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
};

// --- 3. Manager class of the tree ---
class BTManager : public rclcpp::Node {
public:
    BTManager() : Node("bt_manager_node") {
        blackboard_ = BT::Blackboard::create();
        
        // Subscriber GPS: update blackboard in run time
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                // Consider the signal valid if the status is not NO_FIX (-1)
                blackboard_->set<bool>("gps_online", msg->status.status >= 0);
            });
    }

    bool init_tree(const std::string& xml_path) {
        // Registration of custom nodes
        factory_.registerNodeType<CheckGPS>("CheckGPS");
        
        auto node_ptr = shared_from_this();
        factory_.registerBuilder<SetFusionWeights>("SetFusionWeights", 
            [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<SetFusionWeights>(name, config, node_ptr);
            });

        try {
            tree_ = factory_.createTreeFromFile(xml_path, blackboard_);
            // Publisher for Groot2 (live visualization)
            zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_);
            
            // execution Timer for tree execution at 10Hz
            timer_ = this->create_wall_timer(100ms, [this]() { 
                if(tree_.rootNode()) {
                    tree_.tickRoot(); 
                }
            });
            
            RCLCPP_INFO(this->get_logger(), "BT Manager: Caricato con successo. ZMQ attivo.");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "BT Manager Errore XML: %s", e.what());
            return false;
        }
    }

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// --- 4. Main ---
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BTManager>();
    
    // XML path of the tree file (you can pass it as a parameter or leave it static)
    // If you use Groot2, make sure the names of the registered nodes match in the XML file.
    std::string xml_file = "src/control_logic_bt/config/my_tree.xml"; 

    if (node->init_tree(xml_file)) {
        RCLCPP_INFO(node->get_logger(), "BT Manager: Tree initialized successfully. Spinning...");
        rclcpp::spin(node);
    } else {
        RCLCPP_ERROR(node->get_logger(), "BT Manager: Failed to initialize tree. Exiting...");
    }
    
    rclcpp::shutdown();
    return 0;
}
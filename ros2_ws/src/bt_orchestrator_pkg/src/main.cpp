#include <iostream>
#include <chrono>
#include <thread>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h" // The Groot2 Magic Link

using namespace BT;

// ==========================================================
// 1. MOCK CONDITION NODE (The GPS Watchdog)
// ==========================================================
class CheckGpsSignal : public ConditionNode {
public:
    CheckGpsSignal(const std::string& name) : ConditionNode(name, {}) {}

    NodeStatus tick() override {
        // A simple counter to simulate driving in and out of a tunnel
        static int counter = 0;
        counter++;
        
        if (counter % 6 < 3) {
            std::cout << "[GPS] Signal GOOD. Environment: OUTDOOR." << std::endl;
            return NodeStatus::SUCCESS;
        } else {
            std::cout << "[GPS] Signal LOST! Environment: INDOOR." << std::endl;
            return NodeStatus::FAILURE; // Triggers the Fallback to Branch 2
        }
    }
};

// ==========================================================
// 2. MOCK ACTION NODES (Using a C++ Macro for speed)
// ==========================================================
#define CREATE_MOCK_ACTION(ClassName, Message) \
class ClassName : public SyncActionNode { \
public: \
    ClassName(const std::string& name) : SyncActionNode(name, {}) {} \
    NodeStatus tick() override { \
        std::cout << "  -> " << Message << std::endl; \
        return NodeStatus::SUCCESS; \
    } \
};

// Define the exact names we used in the XML file
CREATE_MOCK_ACTION(RunDualEKF, "Running Dual EKF (IMU + GPS + Odom)")
CREATE_MOCK_ACTION(TrainNeuralNetwork, "Gathering data. Training NN async...")
CREATE_MOCK_ACTION(HaltEKF1_UpdateEKF2, "Halting EKF1. Updating EKF2 with last known GPS.")
CREATE_MOCK_ACTION(RunNeuralNetworkInference, "Running NN Inference for estimated Position.")
CREATE_MOCK_ACTION(CalculateSlipError, "Calculating Slip Error (IMU vs Odom).")
CREATE_MOCK_ACTION(RunFuzzyLogicController, "Running Fuzzy Logic to generate Alpha Weights.")
CREATE_MOCK_ACTION(FuseNN_and_EKF2, "Fusing NN and EKF2 Output for final position.")

// ==========================================================
// 3. THE MAIN ORCHESTRATOR LOOP
// ==========================================================
int main() {
    BehaviorTreeFactory factory;

    // A. Register all our custom nodes with the factory
    factory.registerNodeType<CheckGpsSignal>("CheckGpsSignal");
    factory.registerNodeType<RunDualEKF>("RunDualEKF");
    factory.registerNodeType<TrainNeuralNetwork>("TrainNeuralNetwork");
    factory.registerNodeType<HaltEKF1_UpdateEKF2>("HaltEKF1_UpdateEKF2");
    factory.registerNodeType<RunNeuralNetworkInference>("RunNeuralNetworkInference");
    factory.registerNodeType<CalculateSlipError>("CalculateSlipError");
    factory.registerNodeType<RunFuzzyLogicController>("RunFuzzyLogicController");
    factory.registerNodeType<FuseNN_and_EKF2>("FuseNN_and_EKF2");

    // B. Load the XML file we created earlier
    // (Make sure to run the executable from the root of your workspace!)
    auto tree = factory.createTreeFromFile("src/bt_orchestrator_pkg/bt_xml/localization_tree.xml");

    // C. Attach the Groot2 Publisher (Uses ZMQ port 1666/1667 by default)
    PublisherZMQ publisher_zmq(tree, 25, 1666, 1667);
    std::cout << "--- Dual-State Localization Brain Started ---" << std::endl;
    std::cout << "Open Groot2 and click 'Connect' to visualize live." << std::endl;
    std::cout << "Press Ctrl+C to stop.\n" << std::endl;

    // D. Run the tree at 1 Hz
    while (true) {
        std::cout << "\n--- BT Tick ---" << std::endl;
        tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
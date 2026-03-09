// #include "rclcpp/rclcpp.hpp"
// #include "oakd_roi_detector_interfaces/srv/get_detections.hpp"
// #include "std_msgs/msg/u_int8_multi_array.hpp"


// class TestingClient : public rclcpp::Node
// {
// public:

//     std_msgs::msg::UInt8MultiArray arr;

//     TestingClient() : Node("client_node")
//     {
//         client_ = this->create_client<oakd_roi_detector_interfaces::srv::GetDetections>("/get_detections");
//         this -> publisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>("ros_data", 20);

//         //Wait for service
//         while (!client_->wait_for_service(std::chrono::milliseconds(100))) {
//             RCLCPP_WARN(this->get_logger(), "Waiting for service...");
//         }


//         /*Sends "true" to start the camera, doesnt take the data here*/
//         auto request = std::make_shared<oakd_roi_detector_interfaces::srv::GetDetections::Request>();
//         request->reset = true;       
//         request->trigger_roi = "";  
//         client_->async_send_request(request);

//         /*Sends "false" to recieve the data*/
//         auto request_data = std::make_shared<oakd_roi_detector_interfaces::srv::GetDetections::Request>();
//         request_data -> reset = false;
//         request_data -> trigger_roi = "";
//         auto future = client_->async_send_request(request_data); // send reset request 
        

//         try {
//             response_ = future.get();

//             timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),
//             [this]() {
//                 handle_roi_state(response_->roi1_class, response_->roi2_class); // now it will publish
//             });

//             // Logging
//             RCLCPP_INFO(this->get_logger(),
//                         "ROI1: %s, ROI2: %s, Success: %s, Status: %s",
//                         response_->roi1_class.c_str(),
//                         response_->roi2_class.c_str(),
//                         response_->success ? "true" : "false",
//                         response_->status.c_str());
//         }
//         catch (const std::exception &e) {
//             RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
//         }

        
//     }

//     void handle_roi_state(const std::string &roi1_class,const std::string &roi2_class);
//     void turn_clock();
//     void turn_counter();
//     rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher;
//     rclcpp::TimerBase::SharedPtr timer_;
//     oakd_roi_detector_interfaces::srv::GetDetections::Response::SharedPtr response_;


// private:
//     rclcpp::Client<oakd_roi_detector_interfaces::srv::GetDetections>::SharedPtr client_;
// };

// void TestingClient::handle_roi_state(const std::string &roi1_class,const std::string &roi2_class)
// {
//     if (roi1_class == "real" && roi2_class == "fake") {
//         turn_counter();

//     }

//     else if (roi1_class == "fake" && roi2_class == "real") {
//         turn_clock();
//     }
// }

// void TestingClient::turn_clock()
// {
//     RCLCPP_INFO(this->get_logger(), "Turning Clockwise");
//     arr.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//     publisher -> publish(arr);
//     // Implement the logic to turn clockwise
// }

// void TestingClient::turn_counter()
// {
//     RCLCPP_INFO(this->get_logger(), "Turning Counter-Clockwise");
//     arr.data = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//     publisher -> publish(arr);
//     // Implement the logic to turn counter-clockwise
// }

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TestingClient>();
//     rclcpp::spin(node);  // spin_once is enough for one request
//     rclcpp::shutdown();
//     return 0;
// }


#include "rclcpp/rclcpp.hpp"
#include "oakd_roi_detector_interfaces/srv/get_detections.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class TestingClient : public rclcpp::Node
{
public:
    TestingClient() : Node("client_node")
    {
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("ros_data", 20);

        // Start the service logic in a separate thread to avoid deadlocking the executor
        threads_.push_back(std::thread(std::bind(&TestingClient::call_roi_service, this)));
    }

    // Function running in a separate thread
    void call_roi_service()
    {
        auto client = this->create_client<oakd_roi_detector_interfaces::srv::GetDetections>("/get_detections");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service...");
        }

        // 1. Send Reset Request
        auto reset_req = std::make_shared<oakd_roi_detector_interfaces::srv::GetDetections::Request>();
        reset_req->reset = true;
        reset_req->trigger_roi = "";
        client->async_send_request(reset_req); 
        
        RCLCPP_INFO(this->get_logger(), "Reset sent. Waiting for camera to process...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 2. Send Data Request
        auto request = std::make_shared<oakd_roi_detector_interfaces::srv::GetDetections::Request>();
        request->reset = false;
        request->trigger_roi = "";

        auto future = client->async_send_request(request);

        try {
            // This blocking call is now safe because it's in a separate thread
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "ROI1: %s, ROI2: %s, Status: %s",
                        response->roi1_class.c_str(),
                        response->roi2_class.c_str(),
                        response->status.c_str());

            handle_roi_state(response->roi1_class, response->roi2_class);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

private:
    void handle_roi_state(const std::string &roi1, const std::string &roi2) {
        std_msgs::msg::UInt8MultiArray arr;
        if (roi1 == "real" && roi2 == "fake") {
            RCLCPP_INFO(this->get_logger(), "Turning Counter-Clockwise");
            arr.data = {0x00, 0x00, 0x00, 0x00, 0xb2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            publisher_->publish(arr);
        } 
        else if (roi1 == "fake" && roi2 == "real") {
            RCLCPP_INFO(this->get_logger(), "Turning Clockwise");
            arr.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            publisher_->publish(arr);
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestingClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
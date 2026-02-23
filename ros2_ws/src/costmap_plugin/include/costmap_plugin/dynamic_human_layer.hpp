#pragma once
#include <vector>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <unitycustommsg/msg/human_array.hpp>
#include <unitycustommsg/msg/human_group_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
namespace unitycustom_layers
{

    class DynamicHumanLayer : public nav2_costmap_2d::CostmapLayer
    {
    public:
        DynamicHumanLayer();
        void onInitialize() override;
        void updateBounds(double, double, double, double *min_x, double *min_y, double *max_x, double *max_y) override;
        void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
        void publishDebugMarker(const std::string &frame,
                                double x, double y,
                                double yaw, double v,
                                int id,
                                visualization_msgs::msg::MarkerArray &arr);

        void reset() override;
        bool isClearable() override;

        void declareParameters();
        rcl_interfaces::msg::SetParametersResult
        onParameterChange(const std::vector<rclcpp::Parameter> &params);
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    protected:
        // Params

        // Data
        rclcpp::Subscription<unitycustommsg::msg::HumanArray>::SharedPtr human_sub_;
        // rclcpp::Subscription<unitycustommsg::msg::HumanGroupArray>::SharedPtr groups_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;

        // std::vector<geometry_msgs::msg::TransformStamped> humans_;

        struct HumanData
        {
            geometry_msgs::msg::TransformStamped transform;
            geometry_msgs::msg::Twist twist;
        };
        std::vector<HumanData> humans_;

        // struct GroupDisc
        //{
        //     std::string frame_id;
        //     geometry_msgs::msg::Pose center;
        //     double radius{0.0}; // meters
        //     rclcpp::Time stamp;
        // };

        // bool enable_groups_{true};
        // double group_radius_scale_{1.0};
        // bool group_lethal_{true};
        //
        // std::vector<GroupDisc> groups_;

        // === PARAMETERS ===
        bool enabled_{true};
        double human_radius_{0.65};
        double human_core_radius_{0.5};

        // Cost function params (same as your current constants)
        double A_base_{60.0};
        double sigma_base_{1.0};

        double A_forward_{100.0};
        double sigma_long_base_{3.0};
        double sigma_lat_base_{0.5};
        double velocity_scale_long_{2.0};
        double spreading_rate_{0.8};

        double A_back_{70.0};
        double sigma_back_long_base_{1.0};
        double sigma_back_lat_base_{0.5};
        double velocity_reduction_back_{1.4};
        double spreading_rate_back_{0.45};

        double k_angular_{2.0};
        double sigma_angle_deg_{60.0};

        double transition_width_{0.4};
        double C_max_{100.0};
        double gamma_{2.0};
        double max_cost_value_{254.0};
        double cutoff_radius_{0.65};

        // TF
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::mutex data_mutex_;

        void humanCallback(const unitycustommsg::msg::HumanArray::SharedPtr msg);
        void groupsCallback(const unitycustommsg::msg::HumanGroupArray::SharedPtr msg);
        double computeHumanCost(double x_eval, double y_eval,
                                double theta, double v) const;
    };

} // namespace unitycustom_layers

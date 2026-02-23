#include "costmap_plugin/dynamic_human_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <algorithm>
#include <cstdint>
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace unitycustom_layers
{

    DynamicHumanLayer::DynamicHumanLayer() {}

    void DynamicHumanLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            RCLCPP_ERROR(rclcpp::get_logger("DynamicHumanLayer"), "Failed to lock node pointer");
            return;
        }

        // 1) Declare all parameters and load initial values
        declareParameters();

        // 2) Register dynamic parameter callback
        param_cb_handle_ = node->add_on_set_parameters_callback(
            std::bind(&DynamicHumanLayer::onParameterChange, this, std::placeholders::_1));

        // declareParameter("enabled", rclcpp::ParameterValue(true));
        // declareParameter("human_radius", rclcpp::ParameterValue(human_radius_));
        // declareParameter("human_core_radius", rclcpp::ParameterValue(human_core_radius_));

        // node->get_parameter(name_ + ".enabled", enabled_);
        // node->get_parameter(name_ + ".human_radius", human_radius_);
        // node->get_parameter(name_ + ".human_core_radius", human_core_radius_);

        // declareParameter("enable_groups", rclcpp::ParameterValue(true));
        // declareParameter("group_radius_scale", rclcpp::ParameterValue(1.0));
        // declareParameter("group_lethal", rclcpp::ParameterValue(true));

        // node->get_parameter(name_ + ".enable_groups", enable_groups_);
        // node->get_parameter(name_ + ".group_radius_scale", group_radius_scale_);
        // node->get_parameter(name_ + ".group_lethal", group_lethal_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // RViz debug marker publisher
        debug_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/dynamic_human_layer/debug_markers", 10);

        // Subscribe to Unity humans
        human_sub_ = node->create_subscription<unitycustommsg::msg::HumanArray>(
            "/all_humans", rclcpp::SensorDataQoS(),
            std::bind(&DynamicHumanLayer::humanCallback, this, std::placeholders::_1));

        matchSize();
        current_ = true;
    }

    void DynamicHumanLayer::humanCallback(const unitycustommsg::msg::HumanArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        humans_.clear();
        humans_.reserve(msg->humans.size());

        for (const auto &h : msg->humans)
        {
            HumanData hd;
            hd.transform = h.transform;
            hd.twist = h.twist;
            humans_.push_back(hd);
        }

        current_ = true;
    }

    void DynamicHumanLayer::updateBounds(
        double, double, double, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        const std::string global_frame = layered_costmap_->getGlobalFrameID();

        for (const auto &h : humans_)
        {
            geometry_msgs::msg::TransformStamped tf_in = h.transform;

            if (!tf_in.header.frame_id.empty() && tf_in.header.frame_id != global_frame)
            {
                try
                {
                    tf_in = tf_buffer_->transform(
                        h.transform, global_frame, tf2::durationFromSec(0.05));
                }
                catch (...)
                {
                    continue;
                }
            }

            const auto &tf = tf_in.transform;
            double x = tf.translation.x;
            double y = tf.translation.z; // your x–z plane convention

            *min_x = std::min(*min_x, x - human_radius_);
            *min_y = std::min(*min_y, y - human_radius_);
            *max_x = std::max(*max_x, x + human_radius_);
            *max_y = std::max(*max_y, y + human_radius_);
        }
    }

    void DynamicHumanLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;

        std::vector<HumanData> humans_copy;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (humans_.empty())
                return;
            humans_copy = humans_;
        }

        const double res = master_grid.getResolution();
        const int size_x = master_grid.getSizeInCellsX();
        const int size_y = master_grid.getSizeInCellsY();
        const std::string global_frame = layered_costmap_->getGlobalFrameID();

        visualization_msgs::msg::MarkerArray debug_arr;
        int id = 0;

        for (const auto &h : humans_copy)
        {
            geometry_msgs::msg::TransformStamped tf_in = h.transform;

            if (!tf_in.header.frame_id.empty() && tf_in.header.frame_id != global_frame)
            {
                try
                {
                    tf_in = tf_buffer_->transform(
                        h.transform, global_frame, tf2::durationFromSec(0.05));
                }
                catch (...)
                {
                    continue;
                }
            }

            double wx = tf_in.transform.translation.x;
            double wy = tf_in.transform.translation.z;

            geometry_msgs::msg::Quaternion qU = tf_in.transform.rotation;

            double roll, pitch, yaw;
            tf2::Quaternion q(qU.x, qU.y, qU.z, qU.w);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Adjust Unity human yaw
            yaw = yaw + 1.57080;

            // Velocity
            double vx = h.twist.linear.x;
            double vz = h.twist.linear.z;
            double v = std::sqrt(vx * vx + vz * vz);
            // double v = 0.5;
            RCLCPP_INFO(rclcpp::get_logger("DynamicHumanLayer"), "velocity = %.6f", v);

            unsigned int mx, my;
            if (!master_grid.worldToMap(wx, wy, mx, my))
                continue;

            publishDebugMarker(global_frame, wx, wy, yaw, v, id++, debug_arr);

            int rad_cells = std::max(1, (int)std::round(human_radius_ / res));

            for (int dx = -rad_cells; dx <= rad_cells; dx++)
            {
                for (int dy = -rad_cells; dy <= rad_cells; dy++)
                {

                    int nx = mx + dx;
                    int ny = my + dy;

                    if (nx < min_i || ny < min_j || nx >= max_i || ny >= max_j)
                        continue;
                    if (nx < 0 || ny < 0 || nx >= size_x || ny >= size_y)
                        continue;

                    double x_eval = dx * res;
                    double y_eval = dy * res;

                    // double dist = std::sqrt(x_eval * x_eval + y_eval * y_eval);
                    //  if (dist > human_radius_)
                    //      continue;

                    double val = computeHumanCost(x_eval, y_eval, yaw, v);

                    unsigned char old_cost = master_grid.getCost(nx, ny);
                    if (old_cost == LETHAL_OBSTACLE)
                        continue;

                    // unsigned char human_cost = static_cast<unsigned char>(val);
                    // unsigned char final_cost = std::max(old_cost, human_cost);
                    // master_grid.setCost(nx, ny, final_cost);
                    // unsigned char human_cost = static_cast<unsigned char>(
                    //    std::clamp(val, 0.0, double(252)));

                    unsigned char human_cost = static_cast<unsigned char>(val);

                    unsigned char final_cost = std::max(old_cost, human_cost);

                    master_grid.setCost(nx, ny, final_cost);
                }
            }
        }

        debug_pub_->publish(debug_arr);
    }

    double DynamicHumanLayer::computeHumanCost(
        double x_eval, double y_eval,
        double theta, double v) const
    {
        // === PARAMETERS (SAME AS NEW PYTHON VERSION) ===
        const double A_base = A_base_;
        const double sigma_base = sigma_base_;

        const double A_forward = A_forward_;
        const double sigma_long_base = sigma_long_base_;
        const double sigma_lat_base = sigma_lat_base_;
        const double velocity_scale_long = velocity_scale_long_;
        const double spreading_rate = spreading_rate_;

        const double A_back = A_back_;
        const double sigma_back_long_base = sigma_back_long_base_;
        const double sigma_back_lat_base = sigma_back_lat_base_;
        const double velocity_reduction_back = velocity_reduction_back_;
        const double spreading_rate_back = spreading_rate_back_;

        const double k_angular = k_angular_;
        const double sigma_angle =
            M_PI * (sigma_angle_deg_ / 180.0); // deg → rad

        const double transition_width = transition_width_;
        const double C_max = C_max_;
        const double gamma = gamma_;
        const double max_cost_value = max_cost_value_;
        static constexpr unsigned char MAX_HUMAN_COST = 254;
        double cutoff_radius = cutoff_radius_;

        // === 1) Transform into human coordinate frame ===
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        double d_long = x_eval * cos_t + y_eval * sin_t; // + front
        double d_lat = -x_eval * sin_t + y_eval * cos_t; // + left

        // === 2) Base Gaussian ===
        double r2 = x_eval * x_eval + y_eval * y_eval;
        double C_base = A_base * std::exp(-r2 / (2.0 * sigma_base * sigma_base));

        // === 3) Forward Gaussian (defined everywhere; uses max(d_long, 0)) ===
        double d_long_forward = std::max(d_long, 0.0);
        double sigma_lat_forward =
            sigma_lat_base + spreading_rate * d_long_forward * (1.0 + v);
        double sigma_long =
            sigma_long_base * (1.0 + velocity_scale_long * v);

        double forward_gauss =
            A_forward * std::exp(-(d_long_forward * d_long_forward / (2 * sigma_long * sigma_long) + d_lat * d_lat / (2 * sigma_lat_forward * sigma_lat_forward)));

        // === 4) Backward Gaussian (defined everywhere; uses max(-d_long, 0)) ===
        double d_long_back = std::max(-d_long, 0.0);
        double velocity_factor = 1.0 / (1.0 + velocity_reduction_back * v);

        double sigma_back_long = sigma_back_long_base * velocity_factor;
        double sigma_back_lat =
            (sigma_back_lat_base + spreading_rate_back * d_long_back) * velocity_factor;

        double back_gauss =
            A_back * std::exp(-(d_long_back * d_long_back / (2 * sigma_back_long * sigma_back_long) + d_lat * d_lat / (2 * sigma_back_lat * sigma_back_lat)));

        // === 5) Smooth blending using tanh ===
        double w_forward = 0.5 * (1.0 + std::tanh(d_long / transition_width));
        double w_back = 1.0 - w_forward;

        double C_forward = forward_gauss * w_forward;
        double C_back = back_gauss * w_back;

        // === 6) Angular modulation ===
        double ang = std::atan2(y_eval, x_eval);

        // wrap angle diff to [-π, π]
        double diff = std::atan2(std::sin(ang - theta), std::cos(ang - theta));

        // double ang_local = std::atan2(d_lat, d_long);
        // double diff = std::atan2(std::sin(ang_local), std::cos(ang_local)); // simpler

        double angular_boost =
            1.0 + k_angular * v * std::exp(-(diff * diff) / (2 * sigma_angle * sigma_angle));

        // === 7) Final continuous cost ===
        double total_cost = C_base + C_forward * angular_boost + C_back;

        // === HARD CUTOFF RADIUS ===
        // static constexpr double cutoff_radius = 2.5; // meters (adjust!)
        double distance = std::sqrt(x_eval * x_eval + y_eval * y_eval);
        if (distance > cutoff_radius)
        {
            return 0.0;
        }

        // === 8) Normalize to [0, 1] using C_max ===
        double C_norm = total_cost / C_max;
        C_norm = std::clamp(C_norm, 0.0, 1.0);

        // === 9) Apply nonlinear gamma scaling ===
        C_norm = std::pow(C_norm, gamma);

        // === 10) Convert to discrete Nav2 cost (uint8) ===
        // double nav_cost = C_norm * max_cost_value;

        // Safety clamp
        // nav_cost = std::clamp(nav_cost, 0.0, max_cost_value);

        double nav_cost = C_norm * MAX_HUMAN_COST;
        nav_cost = std::clamp(nav_cost, 0.0, double(max_cost_value));

        return nav_cost;
    }
    void DynamicHumanLayer::publishDebugMarker(
        const std::string &frame, double x, double y,
        double yaw, double v, int id,
        visualization_msgs::msg::MarkerArray &arr)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = rclcpp::Clock().now();
        m.ns = "human_debug";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 1.0;

        m.scale.x = 0.4;
        m.scale.y = 0.4;
        m.scale.z = 0.4;

        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 0.9;

        arr.markers.push_back(m);

        visualization_msgs::msg::Marker arrow;
        arrow.header = m.header;
        arrow.ns = "human_debug_arrow";
        arrow.id = id + 1000;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.pose.position.x = x;
        arrow.pose.position.y = y;
        arrow.pose.position.z = 1.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        arrow.pose.orientation = tf2::toMsg(q);

        arrow.scale.x = 1.0 + v;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;

        arrow.color.g = 1.0;
        arrow.color.a = 1.0;

        arr.markers.push_back(arrow);
    }
    void DynamicHumanLayer::declareParameters()
    {
        auto node = node_.lock();
        if (!node)
        {
            return;
        }

        // use CostmapLayer::declareParameter so name_ prefix is applied
        declareParameter("enabled", rclcpp::ParameterValue(enabled_));
        declareParameter("human_radius", rclcpp::ParameterValue(human_radius_));
        declareParameter("human_core_radius", rclcpp::ParameterValue(human_core_radius_));

        declareParameter("A_base", rclcpp::ParameterValue(A_base_));
        declareParameter("sigma_base", rclcpp::ParameterValue(sigma_base_));

        declareParameter("A_forward", rclcpp::ParameterValue(A_forward_));
        declareParameter("sigma_long_base", rclcpp::ParameterValue(sigma_long_base_));
        declareParameter("sigma_lat_base", rclcpp::ParameterValue(sigma_lat_base_));
        declareParameter("velocity_scale_long", rclcpp::ParameterValue(velocity_scale_long_));
        declareParameter("spreading_rate", rclcpp::ParameterValue(spreading_rate_));

        declareParameter("A_back", rclcpp::ParameterValue(A_back_));
        declareParameter("sigma_back_long_base", rclcpp::ParameterValue(sigma_back_long_base_));
        declareParameter("sigma_back_lat_base", rclcpp::ParameterValue(sigma_back_lat_base_));
        declareParameter("velocity_reduction_back", rclcpp::ParameterValue(velocity_reduction_back_));
        declareParameter("spreading_rate_back", rclcpp::ParameterValue(spreading_rate_back_));

        declareParameter("k_angular", rclcpp::ParameterValue(k_angular_));
        declareParameter("sigma_angle_deg", rclcpp::ParameterValue(sigma_angle_deg_));

        declareParameter("transition_width", rclcpp::ParameterValue(transition_width_));
        declareParameter("C_max", rclcpp::ParameterValue(C_max_));
        declareParameter("gamma", rclcpp::ParameterValue(gamma_));
        declareParameter("max_cost_value", rclcpp::ParameterValue(max_cost_value_));

        declareParameter("cutoff_radius", rclcpp::ParameterValue(cutoff_radius_));

        // Load actual values (possibly from YAML)
        node->get_parameter(name_ + ".enabled", enabled_);
        node->get_parameter(name_ + ".human_radius", human_radius_);
        node->get_parameter(name_ + ".human_core_radius", human_core_radius_);

        node->get_parameter(name_ + ".A_base", A_base_);
        node->get_parameter(name_ + ".sigma_base", sigma_base_);

        node->get_parameter(name_ + ".A_forward", A_forward_);
        node->get_parameter(name_ + ".sigma_long_base", sigma_long_base_);
        node->get_parameter(name_ + ".sigma_lat_base", sigma_lat_base_);
        node->get_parameter(name_ + ".velocity_scale_long", velocity_scale_long_);
        node->get_parameter(name_ + ".spreading_rate", spreading_rate_);

        node->get_parameter(name_ + ".A_back", A_back_);
        node->get_parameter(name_ + ".sigma_back_long_base", sigma_back_long_base_);
        node->get_parameter(name_ + ".sigma_back_lat_base", sigma_back_lat_base_);
        node->get_parameter(name_ + ".velocity_reduction_back", velocity_reduction_back_);
        node->get_parameter(name_ + ".spreading_rate_back", spreading_rate_back_);

        node->get_parameter(name_ + ".k_angular", k_angular_);
        node->get_parameter(name_ + ".sigma_angle_deg", sigma_angle_deg_);

        node->get_parameter(name_ + ".transition_width", transition_width_);
        node->get_parameter(name_ + ".C_max", C_max_);
        node->get_parameter(name_ + ".gamma", gamma_);
        node->get_parameter(name_ + ".max_cost_value", max_cost_value_);

        node->get_parameter(name_ + ".cutoff_radius", cutoff_radius_);
    }

    rcl_interfaces::msg::SetParametersResult
    DynamicHumanLayer::onParameterChange(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &p : params)
        {
            const std::string &name = p.get_name();

            // All plugin params are prefixed with "<layer_name>."
            if (name == name_ + ".enabled")
            {
                enabled_ = p.as_bool();
            }
            else if (name == name_ + ".human_radius")
            {
                human_radius_ = p.as_double();
            }
            else if (name == name_ + ".human_core_radius")
            {
                human_core_radius_ = p.as_double();
            }
            else if (name == name_ + ".A_base")
            {
                A_base_ = p.as_double();
            }
            else if (name == name_ + ".sigma_base")
            {
                sigma_base_ = p.as_double();
            }
            else if (name == name_ + ".A_forward")
            {
                A_forward_ = p.as_double();
            }
            else if (name == name_ + ".sigma_long_base")
            {
                sigma_long_base_ = p.as_double();
            }
            else if (name == name_ + ".sigma_lat_base")
            {
                sigma_lat_base_ = p.as_double();
            }
            else if (name == name_ + ".velocity_scale_long")
            {
                velocity_scale_long_ = p.as_double();
            }
            else if (name == name_ + ".spreading_rate")
            {
                spreading_rate_ = p.as_double();
            }
            else if (name == name_ + ".A_back")
            {
                A_back_ = p.as_double();
            }
            else if (name == name_ + ".sigma_back_long_base")
            {
                sigma_back_long_base_ = p.as_double();
            }
            else if (name == name_ + ".sigma_back_lat_base")
            {
                sigma_back_lat_base_ = p.as_double();
            }
            else if (name == name_ + ".velocity_reduction_back")
            {
                velocity_reduction_back_ = p.as_double();
            }
            else if (name == name_ + ".spreading_rate_back")
            {
                spreading_rate_back_ = p.as_double();
            }
            else if (name == name_ + ".k_angular")
            {
                k_angular_ = p.as_double();
            }
            else if (name == name_ + ".sigma_angle_deg")
            {
                sigma_angle_deg_ = p.as_double();
            }
            else if (name == name_ + ".transition_width")
            {
                transition_width_ = p.as_double();
            }
            else if (name == name_ + ".C_max")
            {
                C_max_ = p.as_double();
            }
            else if (name == name_ + ".gamma")
            {
                gamma_ = p.as_double();
            }
            else if (name == name_ + ".max_cost_value")
            {
                max_cost_value_ = p.as_double();
            }
            else if (name == name_ + ".cutoff_radius")
            {
                cutoff_radius_ = p.as_double();
            }
        }

        // after parameter update we can mark the layer current so Nav2 recomputes
        current_ = true;
        return result;
    }

    void DynamicHumanLayer::reset()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        humans_.clear();
        current_ = true;
    }

    bool DynamicHumanLayer::isClearable()
    {
        return true;
    }

} // namespace unitycustom_layers

PLUGINLIB_EXPORT_CLASS(unitycustom_layers::DynamicHumanLayer, nav2_costmap_2d::Layer)

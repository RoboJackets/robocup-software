#pragma once

#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/kicker_picker.hpp>
#include <rj_msgs/srv/kicker_picker.hpp>

namespace strategy {

/**
 * @brief Client for interacting with the KickerPicker coordinator.
 *
 * Manages membership in the kicker group and tracks the currently selected kicker.
 */
class KickerPickerClient {
public:
    struct Result {
        bool am_i_member{false};  // Whether this robot is currently a member of the kicker group.
        std::optional<int> kicker_id{0};  // ID of Kicker id
    };

    using StatusCallback = std::function<void(Result)>;

    explicit KickerPickerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id);
    ~KickerPickerClient() = default;
    KickerPickerClient(const KickerPickerClient&) = delete;
    KickerPickerClient& operator=(const KickerPickerClient&) = delete;
    KickerPickerClient(KickerPickerClient&&) = delete;
    KickerPickerClient& operator=(KickerPickerClient&&) = delete;

    /**
     * @brief Join the kicker group.
     * @param callback Called with current membership status after attempt to join.
     */
    void join_group(StatusCallback callback = nullptr);

    /**
     * @brief Leave the kicker group.
     * @param callback Called with current membership status after attempt to leave.
     */
    void leave_group(StatusCallback callback = nullptr);

    /**
     * @brief Check if this robot is a member of the kicker group.
     */
    [[nodiscard]] bool am_i_member() const;

    /**
     * @brief Get the currently selected kicker.
     * @return robot ID of selected kicker, or kInvalidRobotId if none selected.
     */
    [[nodiscard]] uint8_t selected_kicker() const;

    /**
     * @brief Check if this robot is currently selected as the kicker.
     */
    [[nodiscard]] bool is_selected() const;

private:
    rclcpp::Node::SharedPtr node_;
    const uint8_t robot_id_;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) -- class
                              // isn't move/copy-able anyway
    rclcpp::Client<rj_msgs::srv::KickerPicker>::SharedPtr client_;
    rclcpp::Subscription<rj_msgs::msg::KickerPicker>::SharedPtr subscription_;

    bool am_i_member_{false};
    uint8_t selected_kicker_;
};

}  // namespace strategy

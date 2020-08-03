#include <Geometry2d/GeometryConversions.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

static rj_geometry_msgs::msg::Point make_ros_point() {
    return rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
}

static Geometry2d::Point make_rj_point() { return Geometry2d::Point{1, 2}; }

TEST(ROSConvertGeometry, point_convert) {
    test_lossless_convert_ros_value<Geometry2d::Point,
                                    rj_geometry_msgs::msg::Point>(
        make_ros_point());
    test_lossless_convert_cpp_value<Geometry2d::Point,
                                    rj_geometry_msgs::msg::Point>(
        make_rj_point());
}

TEST(ROSConvertGeometry, pose_convert) {
    test_lossless_convert_ros_value<Geometry2d::Pose,
                                    rj_geometry_msgs::msg::Pose>(
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Pose>()
            .position(make_ros_point())
            .heading(3.0));
    test_lossless_convert_cpp_value<Geometry2d::Pose,
                                    rj_geometry_msgs::msg::Pose>(
        (Geometry2d::Pose{1, 2, 3}));
}

TEST(ROSConvertGeometry, twist_convert) {
    test_lossless_convert_ros_value<Geometry2d::Twist,
                                    rj_geometry_msgs::msg::Twist>(
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Twist>()
            .linear(make_ros_point())
            .angular(3.0));
    test_lossless_convert_cpp_value<Geometry2d::Twist,
                                    rj_geometry_msgs::msg::Twist>(
        (Geometry2d::Twist{1, 2, 3}));
}

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
        Geometry2d::Pose{1, 2, 3});
}

TEST(ROSConvertGeometry, twist_convert) {
    test_lossless_convert_ros_value<Geometry2d::Twist,
                                    rj_geometry_msgs::msg::Twist>(
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Twist>()
            .linear(make_ros_point())
            .angular(3.0));
    test_lossless_convert_cpp_value<Geometry2d::Twist,
                                    rj_geometry_msgs::msg::Twist>(
        Geometry2d::Twist{1, 2, 3});
}

TEST(ROSConvertGeometry, line_convert) {
    rj_geometry_msgs::msg::Line line;
    line.pt[0] =
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    line.pt[1] =
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    test_lossless_convert_ros_value<Geometry2d::Line,
                                    rj_geometry_msgs::msg::Line>(line);
    test_lossless_convert_cpp_value<Geometry2d::Line,
                                    rj_geometry_msgs::msg::Line>(
        Geometry2d::Line(make_rj_point(), make_rj_point() * 2));
}

TEST(ROSConvertGeometry, segment_convert) {
    rj_geometry_msgs::msg::Segment segment;
    segment.pt[0] =
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    segment.pt[1] =
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    test_lossless_convert_ros_value<Geometry2d::Segment,
                                    rj_geometry_msgs::msg::Segment>(segment);
    test_lossless_convert_cpp_value<Geometry2d::Segment,
                                    rj_geometry_msgs::msg::Segment>(
        Geometry2d::Segment(make_rj_point(), make_rj_point() * 2));
}

TEST(ROSConvertGeometry, rect_convert) {
    rj_geometry_msgs::msg::Rect rect;
    rect.pt[0] =
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    rect.pt[1] =
        rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    test_lossless_convert_ros_value<Geometry2d::Rect,
                                    rj_geometry_msgs::msg::Rect>(rect);
    test_lossless_convert_cpp_value<Geometry2d::Rect,
                                    rj_geometry_msgs::msg::Rect>(
        Geometry2d::Rect(make_rj_point(), make_rj_point() * 2));
}

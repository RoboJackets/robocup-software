#include <rj_geometry/geometry_conversions.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

static rj_geometry_msgs::msg::Point make_ros_point() {
    return rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
}

static rj_geometry::Point make_rj_point() { return rj_geometry::Point{1, 2}; }

TEST(ROSConvertGeometry, point_convert) {
    test_lossless_convert_ros_value(make_ros_point());
    test_lossless_convert_cpp_value(make_rj_point());
}

TEST(ROSConvertGeometry, pose_convert) {
    test_lossless_convert_ros_value(rj_geometry_msgs::build<rj_geometry_msgs::msg::Pose>()
                                        .position(make_ros_point())
                                        .heading(3.0));
    test_lossless_convert_cpp_value(rj_geometry::Pose{1, 2, 3});
}

TEST(ROSConvertGeometry, twist_convert) {
    test_lossless_convert_ros_value(rj_geometry_msgs::build<rj_geometry_msgs::msg::Twist>()
                                        .linear(make_ros_point())
                                        .angular(3.0));
    test_lossless_convert_cpp_value(rj_geometry::Twist{1, 2, 3});
}

TEST(ROSConvertGeometry, line_convert) {
    rj_geometry_msgs::msg::Line line;
    line.pt[0] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    line.pt[1] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    test_lossless_convert_ros_value(line);
    test_lossless_convert_cpp_value(rj_geometry::Line(make_rj_point(), make_rj_point() * 2));
}

TEST(ROSConvertGeometry, segment_convert) {
    rj_geometry_msgs::msg::Segment segment;
    segment.pt[0] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    segment.pt[1] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    test_lossless_convert_ros_value(segment);
    test_lossless_convert_cpp_value(rj_geometry::Segment(make_rj_point(), make_rj_point() * 2));
}

TEST(ROSConvertGeometry, rect_convert) {
    rj_geometry_msgs::msg::Rect rect;
    rect.pt[0] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    rect.pt[1] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    test_lossless_convert_ros_value(rect);
    test_lossless_convert_cpp_value(rj_geometry::Rect(make_rj_point(), make_rj_point() * 2));
}

TEST(ROSConvertGeometry, shape_set_convert) {
    rj_geometry_msgs::msg::Rect rect;
    rect.pt[0] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(0).y(1);
    rect.pt[1] = rj_geometry_msgs::build<rj_geometry_msgs::msg::Point>().x(1).y(2);
    rj_geometry_msgs::msg::Circle circle;
    circle.center = rj_convert::convert_to_ros(rj_geometry::Point(1.0, 1.0));
    circle.radius = 0.5;

    rj_geometry_msgs::msg::ShapeSet shapes;
    shapes.circles.emplace_back(circle);
    test_lossless_convert_ros_value(shapes);
}

#include <gtest/gtest.h>
#include <cmath>
#include "vision/robot/KalmanRobot.hpp"
#include "vision/robot/WorldRobot.hpp"

TEST(KalmanRobot, invalid_world_robot) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b = CameraRobot(t, p, theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    Geometry2d::Point rp = kb.getPos();
    Geometry2d::Point rv = kb.getVel();
    double th = kb.getTheta();
    double om = kb.getOmega();

    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(th, theta);
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_EQ(om, 0);
    EXPECT_FALSE(kb.isUnhealthy());
    EXPECT_EQ(kb.getCameraID(), cID);
    EXPECT_GT(kb.getHealth(), 0);
}

TEST(KalmanRobot, valid_world_robot) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b1 = CameraRobot(t, p, theta, robotID);
    CameraRobot b2 = CameraRobot(t, p+p, theta+theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b1, w);
    kb.predictAndUpdate(t, b2);

    std::list<KalmanRobot> kbl;
    kbl.push_back(kb);

    WorldRobot wb = WorldRobot(t, WorldRobot::Team::BLUE, robotID, kbl);

    KalmanRobot kb2 = KalmanRobot(cID, t, b1, wb);

    Geometry2d::Point rp = kb2.getPos();
    Geometry2d::Point rv = kb2.getVel();
    double th = kb2.getTheta();
    double om = kb2.getOmega();

    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(th, theta);
    EXPECT_GT(rv.x(), 0);
    EXPECT_GT(rv.y(), 0);
    EXPECT_GT(om, 0);
    EXPECT_LT(rv.x(), p.x());
    EXPECT_LT(rv.y(), p.y());
    EXPECT_LT(om, theta);
}

TEST(KalmanRobot, predict) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b1 = CameraRobot(t, p, theta, robotID);
    CameraRobot b2 = CameraRobot(t, p+p, theta+theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b1, w);
    kb.predictAndUpdate(t, b2);

    Geometry2d::Point rp = kb.getPos();
    Geometry2d::Point rv = kb.getVel();
    double th = kb.getTheta();
    double om = kb.getOmega();
    
    kb.predict(t);

    Geometry2d::Point rp2 = kb.getPos();
    Geometry2d::Point rv2 = kb.getVel();
    double th2 = kb.getTheta();
    double om2 = kb.getOmega();
    
    EXPECT_NEAR(rp2.x(), rp.x() + rv.y()*0.01, 0.01);
    EXPECT_NEAR(rp2.y(), rp.y() + rv.y()*0.01, 0.01);
    EXPECT_NEAR(th2, th + om*0.01, 0.01);
    EXPECT_FALSE(kb.isUnhealthy());
    EXPECT_EQ(kb.getCameraID(), cID);
    EXPECT_GT(kb.getHealth(), 0);
}

TEST(KalmanRobot, predict_and_update) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b1 = CameraRobot(t, p, theta, robotID);
    CameraRobot b2 = CameraRobot(t, p+p, theta+theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b1, w);
    kb.predictAndUpdate(t, b2);

    Geometry2d::Point rp = kb.getPos();
    Geometry2d::Point rv = kb.getVel();
    double th = kb.getTheta();
    double om = kb.getOmega();
    
    EXPECT_NEAR(rp.x(), p.x()*2, 0.1);
    EXPECT_NEAR(rp.y(), p.y()*2, 0.1);
    EXPECT_NEAR(th, theta*2, 0.1);
    EXPECT_GT(rv.x(), 0);
    EXPECT_GT(rv.y(), 0);
    EXPECT_GT(om, 0);
    EXPECT_LT(rv.x(), p.x() / .01);
    EXPECT_LT(rv.y(), p.y() / .01);
    EXPECT_LT(om, th / 0.01);
    EXPECT_FALSE(kb.isUnhealthy());
    EXPECT_EQ(kb.getCameraID(), cID);
    EXPECT_GT(kb.getHealth(), 0);
}

TEST(KalmanRobot, is_unhealthy) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b = CameraRobot(t, p, theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    kb.predict(RJ::now() + RJ::Seconds(10));

    EXPECT_TRUE(kb.isUnhealthy());
}

TEST(KalmanRobot, max_measurement_size) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b = CameraRobot(t, p, theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    for (int i = 0; i < 100; i++) {
        kb.predictAndUpdate(RJ::now() + RJ::Seconds(10), b);
    }

    boost::circular_buffer<CameraRobot> list = kb.getPrevMeasurements();

    EXPECT_LT(list.size(), 10);
}

TEST(KalmanRobot, getters) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 1;
    int robotID = 1;
    
    CameraRobot b = CameraRobot(t, p, theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    Geometry2d::Point rpc = kb.getPosCov();
    double rtc = kb.getThetaCov();
    Geometry2d::Point rvc = kb.getVelCov();
    double roc = kb.getOmegaCov();
    Geometry2d::Point rp = kb.getPos();
    double rt = kb.getTheta();
    Geometry2d::Point rv = kb.getVel();
    double ro = kb.getOmega();


    boost::circular_buffer<CameraRobot> list = kb.getPrevMeasurements();

    EXPECT_EQ(kb.getCameraID(), cID);
    EXPECT_GT(kb.getHealth(), 0);
    EXPECT_EQ(rp.x(), p.x());
    EXPECT_EQ(rp.y(), p.y());
    EXPECT_EQ(rt, theta);
    EXPECT_EQ(rv.x(), 0);
    EXPECT_EQ(rv.y(), 0);
    EXPECT_EQ(ro, 0);
    EXPECT_GT(rpc.x(), 0);
    EXPECT_GT(rpc.y(), 0);
    EXPECT_GT(rtc, 0);
    EXPECT_GT(rvc.x(), 0);
    EXPECT_GT(rvc.y(), 0);
    EXPECT_GT(roc, 0);
    EXPECT_LT(rpc.x(), 10000);
    EXPECT_LT(rpc.y(), 10000);
    EXPECT_LT(rtc, 10000);
    EXPECT_LT(rvc.x(), 10000);
    EXPECT_LT(rvc.y(), 10000);
    EXPECT_LT(roc, 10000);
    EXPECT_EQ(list.size(), 1);
}

TEST(KalmanRobot, wrap_theta_up) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 0;
    int robotID = 1;
    
    CameraRobot b = CameraRobot(t, p, theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    double ut = 0;
    for (int i = 0; i < 800; i++) {
        theta += 1.0/100.0;
        ut += 1.0/100.0;
    
        if (theta > M_PI) {
            theta -= 2*M_PI;
        }
 
        p += Geometry2d::Point(1,1)*1.0/100.0;

        b = CameraRobot(t, p, theta, robotID);
        kb.predictAndUpdate(RJ::now() + RJ::Seconds(10), b);

    }

    double rt = kb.getTheta();
    double ro = kb.getOmega();
    EXPECT_NEAR(rt, ut, 0.01);
    EXPECT_NEAR(ro, 1, 0.01);

    Geometry2d::Point rp = kb.getPos();
    Geometry2d::Point rv = kb.getVel();
    EXPECT_NEAR(rp.x(), p.x(), 0.01);
    EXPECT_NEAR(rp.y(), p.y(), 0.01);
    EXPECT_NEAR(rv.x(), 1, 0.01);
    EXPECT_NEAR(rv.y(), 1, 0.01);
}

TEST(KalmanRobot, wrap_theta_down) {
    RJ::Time t = RJ::now();
    Geometry2d::Point p = Geometry2d::Point(1,1);
    double theta = 0;
    int robotID = 1;
    
    CameraRobot b = CameraRobot(t, p, theta, robotID);
    int cID = 1;
    WorldRobot w;

    KalmanRobot kb = KalmanRobot(cID, t, b, w);

    double ut = 0;
    for (int i = 0; i < 800; i++) {
        theta -= 1.0/100.0;
        ut -= 1.0/100.0;
    
        if (theta < -M_PI) {
            theta += 2*M_PI;
        }
 
        p -= Geometry2d::Point(1,1)*1.0/100.0;

        b = CameraRobot(t, p, theta, robotID);
        kb.predictAndUpdate(RJ::now() + RJ::Seconds(10), b);

    }

    double rt = kb.getTheta();
    double ro = kb.getOmega();
    EXPECT_NEAR(rt, ut, 0.01);
    EXPECT_NEAR(ro, -1, 0.01);

    Geometry2d::Point rp = kb.getPos();
    Geometry2d::Point rv = kb.getVel();
    EXPECT_NEAR(rp.x(), p.x(), 0.01);
    EXPECT_NEAR(rp.y(), p.y(), 0.01);
    EXPECT_NEAR(rv.x(), -1, 0.01);
    EXPECT_NEAR(rv.y(), -1, 0.01);
}
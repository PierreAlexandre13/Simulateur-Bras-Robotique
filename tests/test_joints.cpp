#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

#include "../include/Cjoint.hpp"


//g++ test_joints.cpp -std=c++17 -I./include $(pkg-config --cflags --libs pinocchio) -lgtest -lgtest_main -pthread -o test_gtest


// Tolérance pour comparaisons flottantes
constexpr double tol = 1e-10;


// Test setQ() hors bornes
TEST(CJointTest, SetQOutOfRange)
{
    CJointRevolute j(-1.0,1.0,0.0, 0.0);
    EXPECT_THROW(j.setQ(2.0), std::out_of_range);
}


// Test CJointRevolute : getTransform() = I4 pour θ=0, dx=0
TEST(CJointTest, RevoluteIdentity)
{
    CJointRevolute j(-M_PI, M_PI, 0.0, 0.0);
    j.setQ(0.0);

    Eigen::Matrix4d M = j.getTransform();
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    EXPECT_TRUE(M.isApprox(I, tol));
}


// Test CJointPrismatic : translation d = 1
TEST(CJointTest, PrismaticTranslation)
{
    CJointPrismatic j(0,2, 0.0);
    j.setQ(1.0);

    Eigen::Matrix4d M = j.getTransform();

    EXPECT_NEAR(M(2,3), 1.0, tol);
}


// Test ligature dynamique / clone
TEST(CJointTest, DynamicPolymorphism)
{
    CJoint* ptr = new CJointRevolute(-M_PI,M_PI,0.5, 0.0);
    ptr->setQ(M_PI/4);

    Eigen::Matrix4d M1 = ptr->getTransform();
    auto clone = ptr->clone();
    Eigen::Matrix4d M2 = clone->getTransform();

    EXPECT_TRUE(M1.isApprox(M2, tol));

    delete ptr;
}

/*
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}*/



#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <memory>
#include <cmath>
#include <stdexcept>

#include "../include/Cjoint.hpp"
#include "../include/Cbras.hpp"


// g++ test_bras.cpp -std=c++17  -I./include  $(pkg-config --cflags --libs pinocchio) -lgtest -lgtest_main -pthread  -o test_bras

// Tolérance pour comparaisons flottantes
constexpr double tol = 1e-10;


// ─────────────────────────────────────────────────────────────────────────────
// a. computeFK() sur un bras vide retourne I4
// ─────────────────────────────────────────────────────────────────────────────
TEST(CBrasTest, ComputeFKEmptyArm)
{
    CBras bras;
    Eigen::Matrix4d T = bras.computeFK();
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    EXPECT_TRUE(T.isApprox(I, tol));
}


// ─────────────────────────────────────────────────────────────────────────────
// b. 1 joint rotatif (theta=0, dx=0.5) : effecteur en (0.5 ; 0 ; 0)
// ─────────────────────────────────────────────────────────────────────────────
TEST(CBrasTest, ComputeFKSingleRevolute)
{
    CBras bras;
    // Signature : (min, max, q_init, dx)
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));

    Eigen::Matrix4d T = bras.computeFK();

    // Extraction de la translation avec block<3,1>(0,3)
    Eigen::Vector3d t = T.block<3,1>(0,3);

    EXPECT_NEAR(t(0), 0.5, tol);   // x = 0.5
    EXPECT_NEAR(t(1), 0.0, tol);   // y = 0
    EXPECT_NEAR(t(2), 0.0, tol);   // z = 0

    // Partie rotation = I3 pour theta = 0
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            EXPECT_NEAR(T(i,j), (i == j ? 1.0 : 0.0), tol);

}


// ─────────────────────────────────────────────────────────────────────────────
// c. getJoint(i) lève std::out_of_range pour i >= N
// ─────────────────────────────────────────────────────────────────────────────
TEST(CBrasTest, GetJointOutOfRange)
{
    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute>(-M_PI, M_PI, 0.0, 0.5));

    // N = 1 → i = 1 doit lever
    EXPECT_THROW(bras.getJoint(1), std::out_of_range);
    // Grand indice
    EXPECT_THROW(bras.getJoint(42), std::out_of_range);
}

TEST(CBrasTest, GetJointOutOfRangeEmptyArm)
{
    CBras empty;
    EXPECT_THROW(empty.getJoint(0), std::out_of_range);
}


// ── Idempotence : setQ(getQ()) ne modifie pas l'état ───────────────────
TEST(CBrasTest, SetQGetQIdempotence) {
    CBras bras;
    bras.addJoint(std::make_unique<CJointRevolute> (-M_PI, M_PI, 0.0, 0.5));
    bras.addJoint(std::make_unique<CJointRevolute> (-M_PI, M_PI, 1.0, 0.5));
    bras.addJoint(std::make_unique<CJointPrismatic>(0.0,  1.0, 0.5));

    // Configuration initiale
    const Eigen::VectorXd q_before = bras.getQ();

    // Application : setQ avec la valeur courante
    bras.setQ(bras.getQ());

    // Configuration après
    const Eigen::VectorXd q_after = bras.getQ();

    // Vérification composante par composante
    ASSERT_EQ(q_before.size(), q_after.size());
    for (Eigen::Index i = 0; i < q_before.size(); ++i)
        EXPECT_DOUBLE_EQ(q_before[i], q_after[i])
            << "Joint " << i << " modifié par setQ(getQ())";
}


// ─────────────────────────────────────────────────────────────────────────────
/*
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}*/
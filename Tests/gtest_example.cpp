#include "gtest/gtest.h"
#include "my_math.h"
#include "meshcollider.h"

#if WITH_GTESTS
using namespace RoboMath;

TEST(TestVector, MultipleTranspose)
{
    EXPECT_EQ(Vector({1,2,3,4}).MultiplyTranspose(Vector{3,4,5,6}), 50);
    EXPECT_EQ(Vector({1,2,3,4}).MultiplyTranspose(Vector{0,0,0,0}), 0);
//    EXPECT_EQ(Vector({1,2,3,4}).MultiplyTranspose(Vector{1,1,1,1}), 1);
}

TEST(TestVector, Comparison)
{
    EXPECT_EQ(Vector({1,2,3,4}) == Vector({1,2,3,4}), true);
    EXPECT_EQ(Vector({1,2,3,4}) == Vector({-1,2,3,4}), false);
    EXPECT_EQ(Vector({1,2,3,4}) == Vector({1,2.001,3.001,4}), false);
    EXPECT_EQ(Vector({-1.22, -5.009, 123, 54.0009}) == Vector({-1.22, -5.009, 123, 54.0009}), true);
}
TEST(Matrix, MultipleTranspose)
{
    Matrix m {{1,2,3},{5,6,7},{7,8,9},{4,3,2},{7,6,5}};
    Matrix n {{3,4,5,5,4},{7,6,5,4,1},{9,3,2,6,3}};
    Vector v{0.1,0.5,0.9};
    EXPECT_EQ(m.MultiplyTranspose(m), Matrix({{14,38,50,16,34},{38, 110, 146,52,106},{50,146,194,70,142},{16,52,70,29,56},{34,106,142,56,110}}));
    EXPECT_EQ(m.MultiplyTranspose(v), Vector({3.8, 9.8, 12.8, 3.7, 8.2}));
}

TEST(Matrix, All)
{
    Matrix4x4 m({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16});
    Matrix4x4 n({6,5,4,3,1,2,3,4,9,8,7,6,5,8,2,6});
    Matrix4x4 m_t({});
    EXPECT_EQ(m*n, Matrix4x4({55,65,39,53,139,157,103,129,223,249,167,205,307,341,231,281}));
    EXPECT_EQ(n*m, Matrix4x4({106,124,142,160,90,100,110,120,190,220,250,280,141,162,183,204}));
    EXPECT_EQ(m(0,0), 1);
    EXPECT_EQ(m(3,3), 16);
    EXPECT_EQ(m.MultiplyTranspose(n), Matrix4x4({40, 30, 70, 51, 112, 70, 190, 135, 184, 110, 310, 219, 256, 150, 430, 303}));
    EXPECT_EQ(m.MultiplyTranspose(m), Matrix4x4({30, 70, 110, 150, 70, 174, 278, 382, 110, 278, 446, 614, 150, 382, 614, 846}));
    EXPECT_EQ(m.MaxElem(), 2);
    EXPECT_EQ(n.MaxElem(), 0);
    m.MakeIdentity();
    EXPECT_EQ(m.MaxElem(), 0);
}
TEST(Collision, All)
{
    MeshCollider mc1;
    MeshCollider mc2;

    EXPECT_EQ(mc1.ReadStl("Tests/models/cube2m.stl"), true);
    EXPECT_EQ(mc2.ReadStl("Tests/models/cube2m.stl"), true);
    EXPECT_EQ(mc1.CheckCollision(mc2), true);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(1,1,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), true);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(1,1,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), true);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(0.1,0.1,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), false);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-2.1,-2.1,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), true);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-2.0,-2.0,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), true);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-0.1,-0.1,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), false);
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-11.1,-11.1,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), false);
    //move to zero position
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(11.2,11.2,0));
    EXPECT_EQ(mc1.CheckCollision(mc2), true);
    //rotate
    mc2.MoveMesh(Quaternion(0.0, 0, 0.707, 0.707), Vector3(2, 2, 0));
    EXPECT_EQ(mc1.CheckCollision(mc2), false);
}



int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif

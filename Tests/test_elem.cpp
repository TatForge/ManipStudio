#include "gtest/gtest.h"
#include "robo.h"

using namespace RoboMath;

TEST(Elem, DefaultConstructorIsZero)
{
    Elem e;
    EXPECT_DOUBLE_EQ(e.GetPosition().x, 0);
    EXPECT_DOUBLE_EQ(e.GetPosition().y, 0);
    EXPECT_DOUBLE_EQ(e.GetPosition().z, 0);
    EXPECT_DOUBLE_EQ(e.GetTheta(), 0);
}

TEST(Elem, ComponentConstructor)
{
    Vector3 pos(1, 2, 3);
    Vector3 rot(0, 0, 1);
    Elem e(pos, rot, 0.5);
    EXPECT_DOUBLE_EQ(e.GetPosition().x, 1);
    EXPECT_DOUBLE_EQ(e.GetPosition().y, 2);
    EXPECT_DOUBLE_EQ(e.GetPosition().z, 3);
    EXPECT_DOUBLE_EQ(e.GetRotationAxis().x, 0);
    EXPECT_DOUBLE_EQ(e.GetRotationAxis().z, 1);
    EXPECT_DOUBLE_EQ(e.GetTheta(), 0.5);
}

TEST(Elem, SetPosition)
{
    Elem e;
    e.SetPosition(Vector3(4, 5, 6));
    EXPECT_DOUBLE_EQ(e.GetPosition().x, 4);
    EXPECT_DOUBLE_EQ(e.GetPosition().y, 5);
    EXPECT_DOUBLE_EQ(e.GetPosition().z, 6);
}

// NOTE: Elem::SetPositionLocal(const scalar&, const scalar&, const scalar&)
// is declared `inline` in robo.h but its only definition (in robo.cpp) is
// never called from within robo.cpp itself. Because it's inline and unused
// in its own translation unit, the compiler elides it entirely, so any
// other translation unit that calls it fails to link. Not tested here; see
// the audit notes for this newly-found issue.

TEST(Elem, SetPositionLocalFromVector3)
{
    Elem e;
    e.SetPositionLocal(Vector3(7, 8, 9));
    EXPECT_DOUBLE_EQ(e.GetPositionLocal().x, 7);
    EXPECT_DOUBLE_EQ(e.GetPositionLocal().y, 8);
    EXPECT_DOUBLE_EQ(e.GetPositionLocal().z, 9);
}

TEST(Elem, SetRotationAxis)
{
    Elem e;
    e.SetRotationAxis(Vector3(0, 1, 0));
    EXPECT_DOUBLE_EQ(e.GetRotationAxis().y, 1);
}

TEST(Elem, SetRotationAxisLocal)
{
    Elem e;
    e.SetRotationAxisLocal(Vector3(1, 0, 0));
    EXPECT_DOUBLE_EQ(e.GetRotationAxisLocal().x, 1);
}

TEST(Elem, GetThetaIncludesBaseAndAdded)
{
    Elem e(Vector3(), Vector3(), 0.3);
    EXPECT_DOUBLE_EQ(e.GetTheta(), 0.3);
    e.AddDeltaTheta(0.2);
    EXPECT_NEAR(e.GetTheta(), 0.5, 1e-9);
}

TEST(Elem, AddDeltaThetaAccumulates)
{
    Elem e;
    e.AddDeltaTheta(0.1);
    e.AddDeltaTheta(0.1);
    e.AddDeltaTheta(0.1);
    EXPECT_NEAR(e.GetTheta(), 0.3, 1e-9);
}

TEST(Effector, SetAndGetRotation)
{
    Effector eff;
    Quaternion q(unit_z, M_PI / 2);
    eff.SetRotation(q);
    EXPECT_NEAR(static_cast<double>(eff.GetRotation().GetAngle()), M_PI / 2, 1e-9);
}

TEST(Effector, InheritsElemPositionBehaviour)
{
    Effector eff;
    eff.SetPosition(Vector3(1, 2, 3));
    EXPECT_DOUBLE_EQ(eff.GetPosition().x, 1);
}

#include "gtest/gtest.h"
#include "my_math.h"

using namespace RoboMath;

TEST(Tensor, InitializerListConstructorAndAccessors)
{
    Tensor t{1, 2, 3, 4, 5, 6};
    EXPECT_DOUBLE_EQ(t.xx(), 1);
    EXPECT_DOUBLE_EQ(t.yy(), 2);
    EXPECT_DOUBLE_EQ(t.zz(), 3);
    EXPECT_DOUBLE_EQ(t.xy(), 4);
    EXPECT_DOUBLE_EQ(t.xz(), 5);
    EXPECT_DOUBLE_EQ(t.yz(), 6);
}

TEST(Tensor, AccessorsAreWritable)
{
    Tensor t{0, 0, 0, 0, 0, 0};
    t.xx() = 42;
    EXPECT_DOUBLE_EQ(t.xx(), 42);
}

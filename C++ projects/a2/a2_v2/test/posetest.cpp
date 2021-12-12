#include "gtest/gtest.h"
#include <iostream>

#include <vector>

// Student defined libraries, for instance
//#include "flightplanner.h"


#include "types.h"
#include "tf.h"
#include "tf2.h"


//==================== UNIT TEST START ====================//

TEST(PoseTest1, LocalToGlobal)
{
    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {3171.34,-4574.67,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }

    {
        Point bogie = {-3288.52,-2516.65,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }
}

TEST(PoseTest2, LocalToGlobal)
{
    Pose aircraft;
    aircraft.position = {2000,1500,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {5171.3,-3074.7,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }

    {
        Point bogie = {-1288.5,-1016.6,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }
}

TEST(PoseTest3, LocalToGlobal)
{
    Pose aircraft;
    aircraft.position = {-400,-600,0};
    aircraft.orientation = tf::yawToQuaternion(2.7676);

    {
        Point bogie = {-5896.0,-1482.3,0};
        RangeBearingStamped rb = {5566.41,0.53316,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }

    {
        Point bogie = {3221.7,-2607.7,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }
}

TEST(PoseTest, GlobalToLocal)
{
    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {3171.34,-4574.67,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);
    }

    {
        Point bogie = {-3288.52,-2516.65,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);

    }
}

TEST(PoseTest1, GlobalToLocal)
{
    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {3171.34,-4574.67,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);
    }

    {
        Point bogie = {-3288.52,-2516.65,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);

    }
}

TEST(PoseTest2, GlobalToLocal)
{
    Pose aircraft;
    aircraft.position = {2000,1500,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {5171.3,-3074.7,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);
    }

    {
        Point bogie = {-1288.5,-1016.6,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);

    }
}
TEST(PoseTest3, GlobalToLocal)
{
    Pose aircraft;
    aircraft.position = {-400,-600,0};
    aircraft.orientation = tf::yawToQuaternion(2.7676);

    {
        Point bogie = {-5896.0,-1482.3,0};
        RangeBearingStamped rb = {5566.41,0.53316,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);
    }

    {
        Point bogie = {3221.7,-2607.7,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        RangeBearingStamped bogieComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rb.range,bogieComputed.range,0.5);
        EXPECT_NEAR(rb.bearing,bogieComputed.bearing,0.5);

    }
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

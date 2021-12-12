#include "gtest/gtest.h"
#include <iostream>

#include <vector>
#include <cmath>
// Student defined libraries, for instance
//#include "flightplanner.h"


#include "types.h"
#include "tf2.h"


//==================== UNIT TEST START ====================//


TEST(BogieTest, DetermineVelocityCorrectedPos)
{
    double bv = 1;
    Pose loc;
    loc.position = {0,0,0};
    loc.orientation = tf::yawToQuaternion(0);
    Point gp {3,4,0};
    Point og {2,5,0};
    double time = sqrt(2);

    Point np = tf2::addVector(gp,og,bv,time);

    EXPECT_NEAR(np.x,4,0.01);
    EXPECT_NEAR(np.y,3,0.01);

}

TEST(BogieTest2, DetermineVelocityCorrectedPos)
{
    double bv = 1;
    Pose loc;
    loc.position = {0,0,0};
    loc.orientation = tf::yawToQuaternion(M_PI/2);
    Point gp {-3,4,0};
    Point og {-2,5,0};
    double time = sqrt(2);

    Point np = tf2::addVector(gp,og,bv,time);

    EXPECT_NEAR(np.x,-4,0.01);
    EXPECT_NEAR(np.y,3,0.01);

}

TEST(BogieTest3, DetermineVelocityCorrectedPos)
{
    double bv = 1;
    Pose loc;
    loc.position = {0,0,0};
    loc.orientation = tf::yawToQuaternion(M_PI/2);
    Point gp {-3,-4,0};
    Point og {-2,-5,0};
    double time = sqrt(2);

    Point np = tf2::addVector(gp,og,bv,time);

    EXPECT_NEAR(np.x,-4,0.01);
    EXPECT_NEAR(np.y,-3,0.01);

}

TEST(BogieTest, ExtrapolationInTime)
{



}



int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

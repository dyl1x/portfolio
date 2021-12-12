#include "gtest/gtest.h"
#include <iostream>
#include <vector>

// Student defined libraries, for instance
//#include "flightplanner.h"

#include "types.h"
#include "tf2.h"
using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;


//==================== UNIT TEST START ====================//


//loc -270.431 -270.431
//rvs
//range 3585.06 velocity 454.32
//range 3693.49 velocity 420.529
//range 3870.42 velocity 351.958
//range 4327.24 velocity 176.531

//rbs
//range 2441.05 bearing 6.21235
//range 4304.59 bearing 4.17406
//range 4972.9 bearing 3.50308
//range 5427.4 bearing 3.53322

//loc -986.535 -986.535
//rvs
//range 2985.74 velocity 420.529
//range 3273.13 velocity 351.958
//range 4021.27 velocity 176.531

//rbs
//range 201.477 bearing 5.39367
//range 4620.59 bearing 3.94539
//range 5912.4 bearing 3.49326
//range 6634.94 bearing 3.51203

TEST(AssociationTest1, arrangeVelocity)
{
    //data
    //loc 119.887 119.887
    //rvs
    //range 4645.34 velocity 454.32
    //range 4658.66 velocity 420.529
    //range 4681.42 velocity 351.958
    //range 4739.82 velocity 176.531

    //rbs
    //range 4673.96 bearing 4.79933
    //range 4677.65 bearing 2.0771
    //range 4740.09 bearing 2.9073
    //range 4748.72 bearing 2.14408

    std::vector<RangeBearingStamped> rbs;
    std::vector<RangeVelocityStamped> rvs;
    Pose loc;
    loc.position.x = 119.887;
    loc.position.y = 119.887;

    rbs.push_back(RangeBearingStamped {4673.96,4.79933,0});
    rbs.push_back(RangeBearingStamped {4677.65,2.0771,0});
    rbs.push_back(RangeBearingStamped {4740.09,2.9073,0});
    rbs.push_back(RangeBearingStamped {4748.72,2.14408,0});

    rvs.push_back(RangeVelocityStamped {4645.34, 454.32, 0});
    rvs.push_back(RangeVelocityStamped {4658.66 , 420.529, 0});
    rvs.push_back(RangeVelocityStamped {4681.42 , 351.958, 0});
    rvs.push_back(RangeVelocityStamped {4739.82 , 176.531, 0});

    std::vector<RangeVelocityStamped> rvsNew = tf2::arrangeVelocity(rbs,rvs,loc);

    for (unsigned int a=0; a<rbs.size(); a++) {
        EXPECT_NEAR(rvsNew.at(a).range,rbs.at(a).range, 10);
    }

}

TEST(AssociationTest2, arrangeVelocity)
{

    std::vector<RangeBearingStamped> rbs;
    std::vector<RangeVelocityStamped> rvs;
    Pose loc;
    loc.position.x = 119.887;
    loc.position.y = 119.887;

    rbs.push_back(RangeBearingStamped {4000,4.79933,0});
    rbs.push_back(RangeBearingStamped {3000,2.0771,0});
    rbs.push_back(RangeBearingStamped {2000,2.9073,0});
    rbs.push_back(RangeBearingStamped {1000,2.14408,0});

    rvs.push_back(RangeVelocityStamped {1000, 454.32, 0});
    rvs.push_back(RangeVelocityStamped {4000 , 420.529, 0});
    rvs.push_back(RangeVelocityStamped {3000 , 351.958, 0});
    rvs.push_back(RangeVelocityStamped {2000 , 176.531, 0});

    std::vector<RangeVelocityStamped> rvsNew = tf2::arrangeVelocity(rbs,rvs,loc);

    for (unsigned int a=0; a<rbs.size(); a++) {
        EXPECT_NEAR(rvsNew.at(a).range,rbs.at(a).range, 1);
    }

}

TEST(AssociationTest3, arrangeVelocity)
{
    std::vector<RangeBearingStamped> rbs;
    std::vector<RangeVelocityStamped> rvs;
    std::vector<RangeVelocityStamped> rvs1;
    Pose loc;
    loc.position.x = 119.887;
    loc.position.y = 119.887;

    rbs.push_back(RangeBearingStamped {500,4.79933,0});
    rbs.push_back(RangeBearingStamped {600,2.0771,0});
    rbs.push_back(RangeBearingStamped {700,2.9073,0});
    rbs.push_back(RangeBearingStamped {800,2.14408,0});

    rvs.push_back(RangeVelocityStamped {570, 454.32, 0});
    rvs.push_back(RangeVelocityStamped {540 , 420.529, 0});
    rvs.push_back(RangeVelocityStamped {840 , 351.958, 0});
    rvs.push_back(RangeVelocityStamped {710 , 176.531, 0});

    rvs1.push_back(RangeVelocityStamped {540, 454.32, 0});
    rvs1.push_back(RangeVelocityStamped {570 , 420.529, 0});
    rvs1.push_back(RangeVelocityStamped {710 , 351.958, 0});
    rvs1.push_back(RangeVelocityStamped {840 , 176.531, 0});

    std::vector<RangeVelocityStamped> rvsNew = tf2::arrangeVelocity(rbs,rvs,loc);

    for (unsigned int a=0; a<rbs.size(); a++) {
        EXPECT_NEAR(rvsNew.at(a).range,rvs1.at(a).range, 1);
    }

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

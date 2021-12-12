#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"

//refer laser_1.png for graph
//use sonar_fusion for fusion with both sensors

TEST (LaserFusionTest1, LaserPassesThrough) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 0
    //  ranges: 3.28502,3.06601,7.90626,4.49436,7.10431,7.32364,6.41023,
    //  Cell parameters
    //  centre [x,y]=[5.7,0]
    //
    // Fusion FREE (LASER occupied);


    RangerMockLaser r1(180, 30, {0.0,0.0,0}, {5.68,3.06601,7.90626,4.49436,7.10431,7.32364,6.41023});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(5.7,0); //just passes into the cell.
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (LaserFusionTest2, LaserCantSee) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 120deg
    //  ranges: 8,8,8,8,8,8,8
    //  Cell parameters
    //  centre [x,y]=[-4.6,3.3]
    //
    // Fusion FREE (LASER unknown);

    double theta = 120*(M_PI/180);
    RangerMockLaser r1(180, 30, {0.0,0.0,theta}, {8,8,8,8,8,8,8});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(-4.6,3.3); //within range but no lines pass through or intersect
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::UNKNOWN,cells.at(0)->getState());
}

TEST (LaserFusionTest3, LaserCanSee) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, -20deg
    //  ranges: 5.68,5.68,5.68,5.68,5.68,5.68,5.68
    //  Cell parameters
    //  centre [x,y]=[5.33,-1.94]
    //
    // Fusion FREE (LASER occupied);

    double theta = -20*(M_PI/180);
    RangerMockLaser r1(180, 30, {0.0,0.0,theta}, {5.68,5.68,5.68,5.68,5.68,5.68,5.68});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(5.33,-1.94); //-20 pose, and 1 line ends inside
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (LaserFusionTest4, LaserCanSee) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 130deg
    //  ranges: 5.68,5.68,5.68,5.68,5.68,5.68,5.68
    //  Cell parameters
    //  centre [x,y]=[-3.65,4.35]
    //
    // Fusion FREE (LASER occupied);

    double theta = 130*(M_PI/180);
    RangerMockLaser r1(180, 30, {0.0,0.0,theta}, {5.68,5.68,5.68,5.68,5.68,5.68,5.68});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(-3.65,4.35); //130 pose, and 1 line ends inside
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (LaserFusionTest5, LaserInTheCell) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 130deg
    //  ranges: 5.68,5.68,5.68,5.68,5.68,5.68,5.68
    //  Cell parameters
    //  centre [x,y]=[0.0000,0.0]
    //
    // Fusion FREE (LASER unknown);

    double theta = 130*(M_PI/180);
    RangerMockLaser r1(180, 30, {0.0,0.0,theta}, {5.68,5.68,5.68,5.68,5.68,5.68,5.68});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0,0); //same position as sensor, shouldnt be able to see it
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::UNKNOWN,cells.at(0)->getState());
}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

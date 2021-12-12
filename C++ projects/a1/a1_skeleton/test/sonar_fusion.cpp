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

//refer sonar_1.png for graph

TEST (SonarFusionTest1, SonarPassesThrough) {

    //  Sonar laser
    //  res: 20.0000
    //  pose: -2, 2, 30deg
    //  ranges: 6.88442
    //  Cell parameters
    //  centre [x,y]=[-3.286,3.5321]

    double theta = 30*(M_PI/180);
    RangerMockSonar r2(20, 20, {-2,2,theta}, {6.88442});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(-3.286,3.5321); //centre is inside sonar sector: sonar return free
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}

TEST (SonarFusionTest2, SonarIntersects) {

    //  Sonar laser
    //  res: 20.0000
    //  pose: -2, 2, 210deg
    //  ranges: 6.88442
    //  Cell parameters
    //  centre [x,y]=[-3.286,3.5321]

    double theta = 210*(M_PI/180);
    RangerMockSonar r2(20, 20, {-2,2,theta}, {6.88442});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(2.525,-3.37); //one conrner of the cell is on the edge of the scan
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState()); //should be occupied
}

TEST (SonarFusionTest3, SonarUnknown) {

    //  Sonar laser
    //  res: 20.0000
    //  pose: -2, 2, 210deg
    //  ranges: 6.88442
    //  Cell parameters
    //  centre [x,y]=[-2,2]

    double theta = 210*(M_PI/180);
    RangerMockSonar r2(20, 20, {-2,2,theta}, {6.88442});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(-2,2); //cell and sonar shares cell
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::UNKNOWN,cells.at(0)->getState()); //should be unknown, sonar cant see, inside min range
}

TEST (SonarFusionTest4, SonarPassesThrough) {

    //  Sonar laser
    //  res: 20.0000
    //  pose: -2, 2, 210deg
    //  ranges: 6.88442
    //  Cell parameters
    //  centre [x,y]=[2,3]
    // side = 6

    double theta = 210*(M_PI/180);
    RangerMockSonar r2(20, 20, {-2,2,theta}, {6.88442});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(2,3); //big cell, intersect circle but ouside the sensor sector
    cells.at(0)->setSide(6);             //but one corner is inside sensor range so should be free, not occupied

    std::vector<RangerInterface *> sensors = { &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState()); //should be occupied
}

TEST (SonarFusionTest5, SonarJustOutofRnage) {

    //  Sonar laser
    //  res: 20.0000
    //  pose: -2, 2, 210deg
    //  ranges: 6.88442
    //  Cell parameters
    //  centre [x,y]=[1.59,-4.04]

    double theta = 210*(M_PI/180);
    RangerMockSonar r2(20, 20, {-2,2,theta}, {6.88442});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(1.59,-4.04); //all corners outside but between the two lines, just out of range
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::UNKNOWN,cells.at(0)->getState()); //should be unknown, sensor cant see
}


TEST (DataFusionTest1, LaserInteractsSonarMisses) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 0.0
    //  ranges: 3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023,
    //  Sonar laser
    //  res: 20.0000
    //  pose: 0.0000, 0.0000, 0.0
    //  ranges: 1.75965
    //  Cell parameters
    //  centre [x,y]=[0.0000,-1.7597]
    //
    // Fusion OCCUPIED (LASER Misses, SONAR INTERSECTS);


    RangerMockLaser r1(180, 30, {0.0,0.0,0}, {3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023});
    RangerMockSonar r2(20, 20, {0.0,0.0,0}, {1.75965});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(3.2,0);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1, &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (DataFusionTest2, BothSeeThrough) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 0.0
    //  ranges: 3.28502,3.06601,7.90626,7.9436,7.10431,7.32364,6.41023,
    //  Sonar
    //  res: 20.0000
    //  pose: 0.0000, 0.0000, 0.0
    //  ranges: 6.5965
    //  Cell parameters
    //  centre [x,y]=[0,4]
    //
    // Fusion OCCUPIED (LASER Misses, SONAR INTERSECTS);


    RangerMockLaser r1(180, 30, {0.0,0.0,0}, {3.28502,3.06601,7.90626,7.9436,7.10431,7.32364,6.41023});
    RangerMockSonar r2(20, 20, {0.0,0.0,0}, {6.5965});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0,4);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1, &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}


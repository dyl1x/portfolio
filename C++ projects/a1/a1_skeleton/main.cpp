#include <iostream>
#include "ranger.h"
#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"
#include "rangerinterface.h"
#include "rangerfusioninterface.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <limits>
using  std::cout;
using  std::endl;


int main()
{

    std::vector<RangerInterface*> rangers;
    rangers.push_back(new Laser());
    rangers.push_back(new Sonar({5,0.0,0}));
    rangers.push_back(new Sonar({9,0.0,0}));
    rangers.push_back(new Sonar({-5,0.0,M_PI}));
    unsigned int count = 1;
    for (auto s:rangers) {
        cout <<"Ranger: "<<count << " - ";
        if (s->getSensingMethod() == ranger::POINT) cout << "Laser" <<endl;
        else {
            cout <<"Sonar"<<endl;
        }
        cout << "   Angular resolution = " << s->getAngularResolution() << endl;
        cout << "   FOV = " << s->getFieldOfView() << endl;
        cout << "   Max range = " << s->getMaxRange() << endl;
        cout << "   Min range = " << s->getMinRange() << endl;
        cout << "   Pose [x,y,theta]= " << s->getSensorPose().x <<","<< s->getSensorPose().y <<","<< s->getSensorPose().theta << endl;

        cout << "Setting ranger angular resolution to 30.  return:" << s->setAngularResolution(30) <<endl;
        cout << "   Angular resolution = " << s->getAngularResolution() << endl;

        count++;
    }

    cout << "Setting Laser angular resolution to 10.  return:" << rangers.at(0)->setAngularResolution(10) <<endl;
    cout << "   Angular resolution = " << rangers.at(0)->getAngularResolution() << endl;

    cout << "Adding position offset of ranger 1 [1,-2,30].  Return:" << rangers.at(0)->setSensorPose({1,-2,30})<< endl;
    cout << "   New Pose [x,y,theta]= " << rangers.at(0)->getSensorPose().x <<","<< rangers.at(0)->getSensorPose().y <<","<< rangers.at(0)->getSensorPose().theta << endl;

    cout << "Adding position offset of ranger 1 [0,0,0].  Return:" << rangers.at(1)->setSensorPose({0,0,0})<< endl;
    cout << "   New Pose [x,y,theta]= " << rangers.at(0)->getSensorPose().x <<","<< rangers.at(0)->getSensorPose().y <<","<< rangers.at(0)->getSensorPose().theta << endl;

    cout << "Adding position offset of ranger 1 [4,1, pi].  Return:" << rangers.at(2)->setSensorPose({4,1,M_PI})<< endl;
    cout << "   New Pose [x,y,theta]= " << rangers.at(0)->getSensorPose().x <<","<< rangers.at(0)->getSensorPose().y <<","<< rangers.at(0)->getSensorPose().theta << endl;
    double pi_2 = M_PI/2;
    cout << "Adding position offset of ranger 1 [-4,5, pi/2].  Return:" << rangers.at(3)->setSensorPose({-4,5,pi_2})<< endl;
    cout << "   New Pose [x,y,theta]= " << rangers.at(0)->getSensorPose().x <<","<< rangers.at(0)->getSensorPose().y <<","<< rangers.at(0)->getSensorPose().theta << endl;


    unsigned int num;
    do{
        std::cout << "How many random cells do you wish to generate (0 - 100): ";
        while(!(std::cin >> num)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }
    }
    while(num>100);


    std::vector<Cell*> cells;

     cout<< "area covered not implemented due to lack of time"<<endl;

    RangerFusion fusion(rangers);


    while (true) {

        for(unsigned int i =0; i < num ;i++) {
            cells.push_back(new Cell);
        }
        fusion.setCells(cells);
        fusion.grabAndFuseData();
        fusion.getCells(cells);

        unsigned int cellcount = 1;
        for (auto s:cells)
        {
            cout<< "Cell "<< cellcount << " state:"<<s->getState();
            switch (s->getState()) {
            case (cell::FREE):
                cout<<" = FREE";
                break;
            case (cell::UNKNOWN):
                cout<<" = UNKNOWN";
                break;
            case (cell::OCCUPIED):
                cout<<" = OCCUPIED";

            }
            cout << endl;
            cellcount++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));//sleep
    }



}

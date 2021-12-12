#include "rangerfusion.h"
#include<limits>

RangerFusion::RangerFusion(std::vector<RangerInterface*> rangers):
    rangers_(rangers)
{
}

RangerFusion::~RangerFusion()
{
}

void RangerFusion::getCells(std::vector<Cell *> &cells)
{
    cells = cells_;
}

void RangerFusion::setCells(std::vector<Cell*> cells)
{
    cells_ = cells;
}

void RangerFusion::grabAndFuseData()
{
    //grab data
    std::vector<std::vector<double>> data;
    for (auto s : rangers_) {
        data.push_back(s->generateData());
    }
    data_ = data;
    //reset all cells to unknown
    for (auto iter : cells_) {
        iter->setState(cell::UNKNOWN);
    }

    //big loop
    unsigned int count = 0;

    for (auto s : rangers_) {
        std::vector <double> dataPoints = data_.at(count);
        if (s->getSensingMethod() == ranger::POINT) //if its a laser
        {
            genPointGeometry(dataPoints,s->getSensorPose(),s->getFieldOfView(),s->getAngularResolution());
            //all the lines are in laserlines_ now
            for (auto j : laserLines_) { //iterate through all the laser lines
                geometry_msgs::Point p1;
                geometry_msgs::Point p2;
                j.getPoints(p1,p2);

                //check of the points are in any of the cells, yes=occupied
                for (auto cell : cells_) {
                    geometry_msgs::Point cellCent;
                    cell->getCentre(cellCent.x,cellCent.y);
                    if (isPointinCell(p1, cellCent,cell->getSide()))
                    {
                        cell->setState(cell::OCCUPIED);
                    }
                }
                //for any unkown type cell check if they intersect any of the lines, yes =free, no = unknown
                for (auto testcell : cells_) {
                    if (testcell->getState() == cell::UNKNOWN)
                    {
                        if (isLineinCell(j,testcell))
                        {
                            testcell->setState(cell::FREE);
                        }
                    }
                    if (testcell->getState() != cell::OCCUPIED)
                    {
                        double halfSide = testcell->getSide()/2;
                        geometry_msgs::Point cent;
                        testcell->getCentre(cent.x,cent.y);
                        geometry_msgs::Point a{cent.x-halfSide,cent.y+halfSide};
                        geometry_msgs::Point b{cent.x+halfSide,cent.y+halfSide};
                        geometry_msgs::Point c{cent.x+halfSide,cent.y-halfSide};
                        geometry_msgs::Point d{cent.x-halfSide,cent.y-halfSide};
                        if(((isInsideCircle(a,s->getSensorPose(),s->getMinRange())) &&
                            (isInsideCircle(b,s->getSensorPose(),s->getMinRange())) &&
                            (isInsideCircle(c,s->getSensorPose(),s->getMinRange())) &&
                            (isInsideCircle(d,s->getSensorPose(),s->getMinRange()))))//all corners of the square are inside min range
                        {
                            testcell->setState(cell::UNKNOWN);
                        }
                    }
                }
            }
        }
        else if (s->getSensingMethod() == ranger::CONE) { //if its a sonar
            genConeGeometry(dataPoints, s->getSensorPose(), s->getFieldOfView(), s->getMinRange());
            for (auto t : cells_) {
                //check of any cell has all the points inside the min range
                double halfSide = t->getSide()/2;
                geometry_msgs::Point cent;
                t->getCentre(cent.x,cent.y);
                geometry_msgs::Point a{cent.x-halfSide,cent.y+halfSide};
                geometry_msgs::Point b{cent.x+halfSide,cent.y+halfSide};
                geometry_msgs::Point c{cent.x+halfSide,cent.y-halfSide};
                geometry_msgs::Point d{cent.x-halfSide,cent.y-halfSide};


                if(!((isInsideCircle(a,s->getSensorPose(),s->getMinRange())) &&
                    (isInsideCircle(b,s->getSensorPose(),s->getMinRange())) &&
                    (isInsideCircle(c,s->getSensorPose(),s->getMinRange())) &&
                    (isInsideCircle(d,s->getSensorPose(),s->getMinRange())))) //if all points are not inside min range
                {
                    //any points between the two lines
                    geometry_msgs::Point right;
                    geometry_msgs::Point left;
                    geometry_msgs::Point dummy; // takes the centre point
                    scanGeometry_.at(0).line1.getPoints(dummy,left);
                    scanGeometry_.at(0).line2.getPoints(dummy,right);
                    bool betweenLines = betweenIntersectinLines(dummy,right,left,a,dataPoints.at(0)) ||
                                        betweenIntersectinLines(dummy,right,left,b,dataPoints.at(0)) ||
                                        betweenIntersectinLines(dummy,right,left,c,dataPoints.at(0)) ||
                                        betweenIntersectinLines(dummy,right,left,d,dataPoints.at(0));   //stores if one of the edges are inside the sector

                    bool inRange = isInsideCircle(a,s->getSensorPose(),s->getMaxRange()) ||
                                   isInsideCircle(b,s->getSensorPose(),s->getMaxRange()) ||
                                   isInsideCircle(c,s->getSensorPose(),s->getMaxRange()) ||
                                   isInsideCircle(d,s->getSensorPose(),s->getMaxRange());

                    if (inRange && betweenLines && (t->getState() == cell::UNKNOWN)) t->setState(cell::FREE);

                    std::vector<geometry_msgs::Point> abInts = isLineInCircle({a,b},s->getSensorPose(),dataPoints.at(0));
                    std::vector<geometry_msgs::Point> bcInts =isLineInCircle({b,c},s->getSensorPose(),dataPoints.at(0));
                    std::vector<geometry_msgs::Point> cdInts =isLineInCircle({c,d},s->getSensorPose(),dataPoints.at(0));
                    std::vector<geometry_msgs::Point> daInts =isLineInCircle({d,a},s->getSensorPose(),dataPoints.at(0));

                    bool ab = false;
                    bool bc = false;
                    bool cd = false;
                    bool da = false;
                    //line ab
                    if (abInts.size() == 1)
                    {
                        ab = betweenIntersectinLines(dummy,right,left,abInts.at(0),dataPoints.at(0));
                    }
                    else if (abInts.size() == 2) {
                        ab = betweenIntersectinLines(dummy,right,left,abInts.at(0),dataPoints.at(0)) ||
                             betweenIntersectinLines(dummy,right,left,abInts.at(0),dataPoints.at(1));
                    }
                    //line bc
                    if (bcInts.size() == 1)
                    {
                        bc = betweenIntersectinLines(dummy,right,left,bcInts.at(0),dataPoints.at(0));
                    }
                    else if (bcInts.size() == 2) {
                        bc = betweenIntersectinLines(dummy,right,left,bcInts.at(0),dataPoints.at(0)) ||
                             betweenIntersectinLines(dummy,right,left,bcInts.at(0),dataPoints.at(1));
                    }
                    //line cs
                    if (cdInts.size() == 1)
                    {
                        cd = betweenIntersectinLines(dummy,right,left,cdInts.at(0),dataPoints.at(0));
                    }
                    else if (cdInts.size() == 2) {
                        cd = betweenIntersectinLines(dummy,right,left,cdInts.at(0),dataPoints.at(0)) ||
                             betweenIntersectinLines(dummy,right,left,cdInts.at(0),dataPoints.at(1));
                    }
                    //line da
                    if (daInts.size() == 1)
                    {
                        da = betweenIntersectinLines(dummy,right,left,daInts.at(0),dataPoints.at(0));
                    }
                    else if (daInts.size() == 2) {
                        da = betweenIntersectinLines(dummy,right,left,daInts.at(0),dataPoints.at(0)) ||
                             betweenIntersectinLines(dummy,right,left,daInts.at(0),dataPoints.at(1));
                    }

                    if (ab || bc || cd || da) t->setState(cell::OCCUPIED); //if one of the line secments intersect the cell is occupied
                }
            }
        }
        count++;
    }

}


std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{   
    return data_;
}

double RangerFusion::getScanningArea()
{
    double area = 0;

    return area;
}

bool RangerFusion::isPointinCell(geometry_msgs::Point point, geometry_msgs::Point cent, double side)
{

    /* a----b
     * |    |
     * d----c
     * */
    double halfSide = side/2;

    return (point.x > (cent.x-halfSide - std::numeric_limits<double>::epsilon()) && //push the sides of cell out by epsilon.
            point.x < (cent.x+halfSide + std::numeric_limits<double>::epsilon()) && //where epsilon is a smallest double value possible.
            point.y > (cent.y-halfSide - std::numeric_limits<double>::epsilon()) && //doing this we assume the thickness of th wall of the cell is epsilon.
            point.y < (cent.y+halfSide + std::numeric_limits<double>::epsilon()) );

}


bool RangerFusion::isLineinCell(Line line, Cell *cell)
{
    geometry_msgs::Point cent;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    line.getPoints(p1,p2);
    cell->getCentre(cent.x,cent.y);
    /* a----b
     * |    |
     * d----c
     * */
    double halfSide = cell->getSide()/2;
    geometry_msgs::Point a{cent.x-halfSide,cent.y+halfSide};
    geometry_msgs::Point b{cent.x+halfSide,cent.y+halfSide};
    geometry_msgs::Point c{cent.x+halfSide,cent.y-halfSide};
    geometry_msgs::Point d{cent.x-halfSide,cent.y-halfSide};


    return doIntersect(p1,p2,a,b) ||
            doIntersect(p1,p2,b,c) ||
            doIntersect(p1,p2,c,d) ||
            doIntersect(p1,p2,d,a);
}

bool RangerFusion::betweenIntersectinLines(geometry_msgs::Point inters, geometry_msgs::Point l1,
                             geometry_msgs::Point l2, geometry_msgs::Point point, double radius)
{
    //undo pose offset
    l1 = {l1.x - inters.x, l1.y-inters.y}; //left
    l2 = {l2.x - inters.x, l2.y-inters.y};
    point = {point.x - inters.x, point.y-inters.y};
    std::vector<geometry_msgs::Point> threePoints {l1,l2,point};
    //convert to polar
    double pointRadius = sqrt(point.x*point.x + point.y*point.y); //radius of the test point

    //get cos^-1(x/radius)
    double start = acos(l1.x/radius);

    if (l1.y < 0) // the point is in 3rd or 4th quadrant
    {
        start = 2*M_PI - start;
    }

    double end = acos(l2.x/radius);
    if (l2.y < 0) // the point is in 3rd or 4th quadrant
    {
        end = 2*M_PI - end;
    }

    double testAngle = acos(point.x/pointRadius);
    if (point.y < 0) // the point is in 3rd or 4th quadrant
    {
        testAngle = 2*M_PI - testAngle;
    }

    return (start < (testAngle) && end > (testAngle));
}

bool RangerFusion::isInsideCircle(geometry_msgs::Point testPoint, ranger::SensorPose cent, double radius)
{
    double rad = ((testPoint.x - cent.x)*(testPoint.x - cent.x)) + ((testPoint.y-cent.y)*(testPoint.y-cent.y));
    return rad < (radius - std::numeric_limits<double>::epsilon());
}

std::vector<geometry_msgs::Point> RangerFusion::isLineInCircle(Line line, ranger::SensorPose cent, double radius)
{
    double a = line.getGradient();
    double b = -1;
    double c = line.getYIntercept();


    double dist = (a * cent.x + b * cent.y + c) / sqrt(a * a + b * b);
    if (dist < 0)
    {
        dist*=-1;
    }
    std::vector<geometry_msgs::Point> intersects;
    if (radius < dist)
    {

    }
    else
    {
        //return true;
        intersects = circleIntersects(cent,line,radius);
    }
    return intersects;
}



//setting up geometry for calculations
//lasers
void RangerFusion::genPointGeometry(std::vector<double> datapoints,ranger::SensorPose pose, double FOV, double angResolution)
{

    std::vector<Line> lines;
    //ranger::SensorPose pose = getSensorPose();
    geometry_msgs::Point cent {pose.x,pose.y};

    double theta = (90 - (FOV/2))*(M_PI/180);
    geometry_msgs::Point point;
    unsigned int count = 0;
    for (auto s : datapoints) {

        //frist we calculate the points of the line, assuming the sensor is at the origin
        //and we also assume that sensor is facign vertically upwards
        point.x = s*cos(theta);
        point.y = s*sin(theta);
        //lets rotate the points about the position of the sensor, to match the pose angle
        geometry_msgs::Point rotated = rotateAboutZ(point,pose.theta);
        //now lets move all the points by moving the origin to the actual sensor location
        point.x = rotated.x + pose.x;
        point.y = rotated.y + pose.y;
        count++;
        theta = (angResolution *count)* (M_PI/180);
        laserLines_.push_back(Line(point,cent));
    }

}
//sonars
void RangerFusion::genConeGeometry(std::vector<double> points, ranger::SensorPose pose, double FOV, double minRange)
{
    std::vector<fusionGeometry::sonarScan> scan;
    geometry_msgs::Point cent {pose.x,pose.y};
    double phi = (90-(FOV/2))*(M_PI/180); //in radians
    double phi2 = (90+(FOV/2))*(M_PI/180);
    geometry_msgs::Point edgeLeft {pose.x,pose.y};
    geometry_msgs::Point edgeRight {pose.x,pose.y};

    //for (unsigned int i =0; i<points.size(); i++) {
        //frist we calculate the 2 edge points of the lines, assuming the sensor is at the origin
        //and we also assume that sensor is facign vertically upwards
        edgeRight.y = points.at(0)*sin(phi);
        edgeLeft.y = points.at(0)*sin(phi2);
        edgeRight.x = points.at(0)*cos(phi);
        edgeLeft.x = points.at(0)*cos(phi2);

        //lets rotate the points about the position of the sensor, to match the pose angle
        edgeLeft = rotateAboutZ(edgeLeft, pose.theta);
        edgeRight = rotateAboutZ(edgeRight, pose.theta);

        //now lets move all the points by moving the origin to the actual sensor location
        edgeRight.y = edgeRight.y + pose.y;
        edgeLeft.y = edgeLeft.y + pose.y;
        edgeRight.x = edgeRight.x + pose.x;
        edgeLeft.x = edgeLeft.x + pose.x;

        scan.push_back(fusionGeometry::sonarScan {Line(cent,edgeLeft),Line(cent,edgeRight),
                                             Circle(cent,minRange),Circle(cent,points.at(0))});
        scanGeometry_ = scan;
    //}
}




std::vector<geometry_msgs::Point> RangerFusion::circleIntersects(ranger::SensorPose cent, Line line, double r) {
    //std::vector<Point> res;
    std::vector<geometry_msgs::Point> res;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    line.getPoints(p1,p2);

    double x0 = cent.x;
    double y0 = cent.y;
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;
    double A = y2 - y1;
    double B = x1 - x2;
    double C = x2 * y1 - x1 * y2;
    double a = sq(A) + sq(B);
    double b, c;
    bool bnz = true;

    if (fabs(B) >= 1e-14) { //if b is not zero or close
        b = 2 * (A * C + A * B * y0 - sq(B) * x0);
        c = sq(C) + 2 * B * C * y0 - sq(B) * (sq(r) - sq(x0) - sq(y0));
    } else {
        b = 2 * (B * C + A * B * x0 - sq(A) * y0);
        c = sq(C) + 2 * A * C * x0 - sq(A) * (sq(r) - sq(x0) - sq(y0));
        bnz = false;
    }
    auto d = sq(b) - 4 * a * c; // discriminant

    if (d < 0) {//if true circle doesnt intersect
        //return res;
    }

    // checks whether a point is within a segment
    auto within = [x1, y1, x2, y2](double x, double y) {
        auto d1 = sqrt(sq(x2 - x1) + sq(y2 - y1));  // distance between end-points
        auto d2 = sqrt(sq(x - x1) + sq(y - y1));    // distance from point to one end
        auto d3 = sqrt(sq(x2 - x) + sq(y2 - y));    // distance from point to other end
        auto delta = d1 - d2 - d3;
        return fabs(delta) < 1e-14;                    // true if delta is less than a small tolerance
    };

    auto fx = [A, B, C](double x) {
        return -(A * x + C) / B;
    };

    auto fy = [A, B, C](double y) {
        return -(B * y + C) / A;
    };

    auto rxy = [&res, within](double x, double y) {
        if (within(x, y)) {
            geometry_msgs::Point p {x,y};
            res.push_back(p);
        }
    };

    double x, y;
    if (d == 0.0) {
        // line is tangent to circle, so just one intersect at most
        if (bnz) {
            x = -b / (2 * a);
            y = fx(x);
            rxy(x, y);
        } else {
            y = -b / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    } else {
        // two intersects at most
        d = sqrt(d);
        if (bnz) {
            x = (-b + d) / (2 * a);
            y = fx(x);
            rxy(x, y);
            x = (-b - d) / (2 * a);
            y = fx(x);
            rxy(x, y);
        } else {
            y = (-b + d) / (2 * a);
            x = fy(y);
            rxy(x, y);
            y = (-b - d) / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    }

    return res;
}

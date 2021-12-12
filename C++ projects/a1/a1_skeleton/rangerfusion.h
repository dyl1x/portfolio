#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "line.h"
#include "circle.h"
#include "processing.h"

/*!
 *  \brief     Ranger Fusion base Class

 *  \author    Chamath Edirisinhege
 *  \brief     Ranger Fusion Class
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */
namespace fusionGeometry {


    struct sonarScan{
        Line line1; /*!< left line */
        Line line2; /*!< right line */
        Circle c1; /*!< Min range */
        Circle c2; /*!< data point distance */
    };/*!< Structure for a sonar scan*/
}


/**
 * @class Ranger Fusion
 * @brief The Ranger Fusion class. This class is derived from the interface class ranger fusion interface
 * @details Accept a container of Sensors and a container of Cells. Then determine the area convered by the sensors as a whole.
 * Produce sensor readings and then checks for interaction with cells and label the cells accordingly. This class
 * uses the geometry library and its classes for calculations.
 */



class RangerFusion: public RangerFusionInterface
{
public:
    /**
    The Default constructor sets the cell centre to values within the #MAP_SIZE\n
    @sa RangerFusionInterface and @sa RangerInterface for more information
    */
    RangerFusion(std::vector<RangerInterface*> rangers);


    virtual ~RangerFusion();


    /**
     * @brief Returns the container of cells.
     * @return cells
     */
    void getCells(std::vector<Cell*> &cells);

    /**
     * @brief Accepts the container of cells.
     * @param cells container
     */
    void setCells(std::vector<Cell*> cells);

    /**
     * @brief Does two operations (1) Calls each ranger to generate data and uses this data to determine colissions
     * with provided container of cells (2) Generates a 'fusion' of the data based on collision conditions as descibed
     * in Assignment 2 specification
     */
    void grabAndFuseData();

    /**
     * @brief Returns the raw data from all sensors in the ranger container within a vector of vectors
     * The raw data is updated every time a new fusion is requested (grabAndFuseDat). The raw data is the data used
     * for collision checking. If no fusion has occured the vector shall be empty.
     * @return std::vector<std::vector<double>>  the outer elements of the vector related to the rangers, the inner
     * elements of vector are the respective range readings
     * @sa grabAndFuseData
     */
    std::vector<std::vector<double>> getRawRangeData();

    /**
     * @brief Returns the total scanning area possible with CONE based scanners supplied
     * A union of all areas https://en.wikipedia.org/wiki/Union_(set_theory)
     * @return double Total area coverage
     * @sa grabAndFuseData
     */
    double getScanningArea();
private:
//Generating Sensor scan Geometry
    //laser
    /**
     * @brief Creats a geometrical representation of the laser scan.
     * @param vector of points which are the distance readings of the laser scan
     * sensor position, fov and angular resolution
     * @return vector of lines representing the laser scans. (ie: the laser beam lines at scan locations)
     * @details uses geometry library to generate line objects from the laser position to the data points.
     */
    void genPointGeometry(std::vector<double> datapoints,ranger::SensorPose pose, double FOV, double angResolution);
    //sonar
    /**
     * @brief Creats a geometrical representation of the Sonar scan.
     * @param vector of points which are the distance readings of the Sonar scan, sensor position,
     * fov and minimum range
     * @return a structure of 2 lines and 2 circle, representing circle sector.
     * @details uses geometry library to generate lines and circles using the geometry library.
     */
    void genConeGeometry(std::vector<double> points, ranger::SensorPose pose, double FOV, double minRange);

//checking intersections
    /**
     * @brief Returns true if the point is inside the cell
     * @param test point, centre of the cell, length of a side
     * @return Returns true if the point is inside the cell
     * @details Uses processing.cpp helper functions from geometry library
     */
    bool isPointinCell(geometry_msgs::Point point, geometry_msgs::Point cent, double side);

    /**
     * @brief Returns true if the line intersects the cell
     * @param the line, and the cell
     * @return Returns true if the point is inside the cell
     * @details Uses processing.cpp helper functions from geometry library
     */
    bool isLineinCell(Line line, Cell *cell);

    /**
     * @brief Finds if a point is inside a circle segment.
     * @param circle centre,end point of the first side of the sector, when rotating anticlockwise, end
     * point of the second side, the point that needs to be tested and the radius of the sector
     * @return Returns true if a point is between 2 intersecting lines
     * @details calculates the angle of each point, and compares them.
     */
    bool betweenIntersectinLines(geometry_msgs::Point inters, geometry_msgs::Point l1,
                                 geometry_msgs::Point l2, geometry_msgs::Point point, double radius);

    /**
     * @brief Finds if a point is inside a circle.
     * @param test point, circle centre, radius
     * @return Returns true if a point is inside the circle
     */
    bool isInsideCircle(geometry_msgs::Point testPoint, ranger::SensorPose cent, double radius);

    /**
     * @brief Finds if a line intersect or touch a circle.
     * @param test line, circle centre, radius
     * @return Returns the intersection points if line intersect or touch the circle
     * @details Taken and modified from //https://www.geeksforgeeks.org/check-line-touches-intersects-circle/
     * modified and added more functionality.
     * function now return the intersection points. the vector will be size 0 if there is no intersection.
     * used to detect a cell is occupied. if the the intesection points are inbetween the two lines of the sonar
     * scan geometry, it would be occupied.
     */
    std::vector<geometry_msgs::Point> isLineInCircle(Line line, ranger::SensorPose cent, double radius);

    /**
     * @brief Gives the intersecting points of a line and a circle of the intersect
     * @param test line, circle centre, radius
     * @return Returns a vector points of the intersects.
     * @details Taken from //https://rosettacode.org/wiki/Line_circle_intersection and modified
     *
     */

    std::vector<geometry_msgs::Point> circleIntersects(ranger::SensorPose cent, Line line, double r);

private:
    //
    std::vector<std::vector<double>> data_; //!< This is to cater for getRawRangeData (which returns the raw data that was used for fusion))
    std::vector<RangerInterface*> rangers_; //!< A private copy of rangers @sa RangerInterface
    std::vector<Cell*> cells_; //!< A private copy of cells @sa Cell

    std::vector<Line> laserLines_; //!< stores the laser scan geometry data
    std::vector<fusionGeometry::sonarScan> scanGeometry_; //!< strores the sonar scan geometry data
};

#endif // RANGERFUSION_H

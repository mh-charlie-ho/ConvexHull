#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

struct ConvexHull
{
    std::vector<Eigen::Vector2f> vertex;
    Eigen::Vector2f center;
};

/*
 * The input polygons must ensure that they do not overlap.
 */
class GJK
{
public:
    GJK(ConvexHull s1, ConvexHull s2);

    GJK(std::vector<cv::Point2f> s1, std::vector<cv::Point2f> s2);

    float Distance();

private:
    ConvexHull mS1;
    ConvexHull mS2;

    Eigen::Vector2f mOrigin;

    ConvexHull ConvertFormat(std::vector<cv::Point2f> points);

    Eigen::Vector2f SupportFunciton(ConvexHull s, Eigen::Vector2f d);

    Eigen::Vector2f Support(Eigen::Vector2f d);

    // calculate the shortest distance to the line. Not always the vertical distance.
    Eigen::Vector2f NearestVector(Eigen::Vector2f &sPt, Eigen::Vector2f &ePt);

    void HandleSimplex(ConvexHull &simplex, Eigen::Vector2f &d);

    void LineCase(ConvexHull &simplex, Eigen::Vector2f &d);

    void TriangleCase(ConvexHull &simplex, Eigen::Vector2f &d);
};
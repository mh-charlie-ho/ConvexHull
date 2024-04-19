#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

struct ConvexHull
{
    std::vector<Eigen::Vector2f> vertex;
    Eigen::Vector2f center;
};

class GJK
{
public:
    GJK(ConvexHull s1, ConvexHull s2);

    GJK(std::vector<cv::Point2f> s1, std::vector<cv::Point2f> s2);

    bool CollisionCheck();

private:
    ConvexHull mS1;
    ConvexHull mS2;

    Eigen::Vector2f mOrigin;

    ConvexHull ConvertFormat(std::vector<cv::Point2f> points);

    Eigen::Vector2f SupportFunciton(ConvexHull s, Eigen::Vector2f d);

    Eigen::Vector2f Support(Eigen::Vector2f d);

    bool HandleSimplex(ConvexHull &simplex, Eigen::Vector2f &d);

    bool LineCase(ConvexHull &simplex, Eigen::Vector2f &d);

    bool TriangleCase(ConvexHull &simplex, Eigen::Vector2f &d);
};
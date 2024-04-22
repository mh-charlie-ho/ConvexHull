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
    GJK(const ConvexHull &s1, const ConvexHull &s2);

    GJK(const std::vector<cv::Point2f> &s1, const std::vector<cv::Point2f> &s2);

    bool CollisionCheck();

private:
    ConvexHull mS1;
    ConvexHull mS2;

    Eigen::Vector2f mOrigin;

    ConvexHull ConvertFormat(
        const std::vector<cv::Point2f> &points);

    Eigen::Vector2f SupportFunciton(
        const ConvexHull &s, 
        const Eigen::Vector2f &d);

    Eigen::Vector2f Support(const Eigen::Vector2f &d);

    bool HandleSimplex(ConvexHull &simplex, Eigen::Vector2f &d);

    bool LineCase(ConvexHull &simplex, Eigen::Vector2f &d);

    bool TriangleCase(ConvexHull &simplex, Eigen::Vector2f &d);

    void PrintSimplex(const ConvexHull &simplex);
};
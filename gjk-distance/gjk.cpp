#include "gjk.hpp"
#include <cmath>

static Eigen::Vector2f prod(Eigen::Vector2f a, Eigen::Vector2f b)
{
    return (1 - (a.dot(b) / (a.norm() * b.norm()))) * b;
}

GJK::GJK(ConvexHull s1, ConvexHull s2)
    : mS1(s1), mS2(s2), mOrigin(0, 0)
{
}

GJK::GJK(std::vector<cv::Point2f> s1, std::vector<cv::Point2f> s2)
    : mOrigin(0, 0)
{
    mS1 = ConvertFormat(s1);
    mS2 = ConvertFormat(s2);
}

ConvexHull GJK::ConvertFormat(std::vector<cv::Point2f> points)
{
    ConvexHull outputConvexHull;

    float xSum = 0;
    float ySum = 0;
    for (int i = 0; i <= points.size(); i++)
    {
        xSum += points[i].x;
        ySum += points[i].y;
        Eigen::Vector2f convertPt(points[i].x, points[i].y);
        outputConvexHull.vertex.push_back(convertPt);
    }

    outputConvexHull.center << xSum / points.size(), ySum / points.size();
    return outputConvexHull;
}

Eigen::Vector2f GJK::SupportFunciton(ConvexHull s, Eigen::Vector2f d)
{
    float tempdot = -1; // 單位向量投影最小值為-1
    int supportPtId = 0;
    for (int i = 0; i < s.vertex.size(); i++)
    {
        float dot = d[0] * s.vertex[i][0] + d[1] * s.vertex[i][1];
        if (tempdot < dot)
        {
            tempdot = dot;
            supportPtId = i;
        }
    }
    return s.vertex[supportPtId];
}

Eigen::Vector2f GJK::Support(Eigen::Vector2f d)
{
    return (SupportFunciton(mS2, d) - SupportFunciton(mS1, -d));
}

Eigen::Vector2f GJK::NearestVector(Eigen::Vector2f &sPt, Eigen::Vector2f &ePt)
{
    // the vertical vector
    Eigen::Vector2f AO(mOrigin - sPt);
    Eigen::Vector2f AB(ePt - sPt);

    if (AO.dot(AB) < 0)
    {
        return AO;
    }
    else if ((AO.dot(AB)) / AB.norm() > AB.norm())
    {
        Eigen::Vector2f BO(mOrigin - ePt);
        return BO;
    }
    else
    {
        return prod(AB, AO);
    }
}

void GJK::Remover(ConvexHull &s)
{
    if (s.vertex[0].norm() > s.vertex[1].norm())
    {
        s.vertex[0] = s.vertex[1];
        s.vertex.pop_back();
    }
    else
    {
        s.vertex.pop_back();
    }
}

float GJK::Distance()
{
    ConvexHull simplex;

    // initialization of the search vector
    Eigen::Vector2f d(mS2.center - mS1.center);
    d.normalize();

    simplex.vertex.push_back(Support(d)); // first pt

    d = mOrigin - simplex.vertex[0];
    d.normalize();

    Eigen::Vector2f p(Support(d)); // second pt
    if (p == simplex.vertex[0])
    {
        return true;
    }
    else
    {
        simplex.vertex.push_back(p);
    }

    while (true)
    {
        d = NearestVector(simplex.vertex[0], simplex.vertex[1]);
        d.normalize();

        p = Support(d);

        
        if (p == simplex.vertex[simplex.vertex.size()-1])
        {
            return true;
        }
        else
        {
            Remover(simplex);
            simplex.vertex.push_back(p);
        }
    }
}

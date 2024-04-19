#include "gjk.hpp"
#include <cmath>

static Eigen::Vector2f prod(Eigen::Vector2f a, Eigen::Vector2f b)
{
    return (1 - (a.dot(b) / (a.norm() * b.norm()))) * b;
}

static float min(float a, float b)
{
    if (a >= b)
    {
        return a;
    }
    return b;
}

static float minLoop(std::vector<float> v)
{
    float ans = v[0];
    for (int i = 1; i < v.size(); i++)
    {
        ans = min(ans, v[i]);
    }
    return ans;
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

void GJK::HandleSimplex(ConvexHull &simplex, Eigen::Vector2f &d)
{
    if (simplex.vertex.size() == 2)
    {
        LineCase(simplex, d); // d 修改成朝向原點的向量
    }
    TriangleCase(simplex, d);
}

void GJK::LineCase(ConvexHull &simplex, Eigen::Vector2f &d)
{
    // 會進來這區代表 simplex.vertex.size() == 2
    d = NearestVector(simplex.vertex[0], simplex.vertex[1]);
}

void GJK::TriangleCase(ConvexHull &simplex, Eigen::Vector2f &d)
{
    // 找最近距離以及迭代方向
    // 目前有三個點
    Eigen::Vector2f A = NearestVector(simplex.vertex[0], simplex.vertex[1]);
    Eigen::Vector2f B = NearestVector(simplex.vertex[1], simplex.vertex[2]);
    Eigen::Vector2f C = NearestVector(simplex.vertex[0], simplex.vertex[2]);

    std::vector<float> minEdge{A.norm(), B.norm(), C.norm()};
    float edgeD = minLoop(minEdge);
    if (edgeD == A.norm())
    {
        simplex.vertex.pop_back();
        d = A;
    }
    else if (edgeD == B.norm())
    {
        simplex.vertex[0] = simplex.vertex[2];
        simplex.vertex.pop_back();
        d = B;
    }
    else
    {
        simplex.vertex[1] = simplex.vertex[2];
        simplex.vertex.pop_back();
        d = C;
    }
}

float GJK::Distance()
{
    Eigen::Vector2f d(mS2.center - mS1.center);
    d.normalize();
    // So the vector d is built.

    ConvexHull simplex;
    simplex.vertex.push_back(Support(d)); // 第一個點
    d << mOrigin - simplex.vertex[0];

    while (true)
    {
        d.normalize();
        Eigen::Vector2f p(Support(d)); // 一條線求第三個點/一個點求第二個點
        if (p == simplex.vertex[simplex.vertex.size() - 1])
        {
            return true;
        }

        simplex.vertex.push_back(p);
        HandleSimplex(simplex, d);
    }
}

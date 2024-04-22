#include "gjk.hpp"
#include <iostream>

using namespace std;

static Eigen::Vector2f prod(Eigen::Vector2f a, Eigen::Vector2f b)
{
    return (1 - (a.dot(b) / (a.norm() * b.norm()))) * b;
}

GJK::GJK(const ConvexHull &s1, const ConvexHull &s2)
    : mS1(s1), mS2(s2), mOrigin(0, 0)
{
}

GJK::GJK(const std::vector<cv::Point2f> &s1, const std::vector<cv::Point2f> &s2)
    : mOrigin(0, 0)
{
    mS1 = ConvertFormat(s1);
    mS2 = ConvertFormat(s2);
}

ConvexHull GJK::ConvertFormat(
    const std::vector<cv::Point2f> &points)
{
    ConvexHull outputConvexHull;

    float xSum = 0;
    float ySum = 0;
    for (int i = 0; i < points.size(); i++)
    {
        xSum += points[i].x;
        ySum += points[i].y;
        Eigen::Vector2f convertPt(points[i].x, points[i].y);
        outputConvexHull.vertex.push_back(convertPt);
    }

    outputConvexHull.center << xSum / points.size(), ySum / points.size();
    return outputConvexHull;
}

Eigen::Vector2f GJK::SupportFunciton(
    const ConvexHull &s,
    const Eigen::Vector2f &d)
{
    cout << "SupportFunciton" << endl;
    float tempdot = -1; // 單位向量投影最小值為-1
    int supportPtId = 0;
    cout << s.vertex.size() << endl;
    for (int i = 0; i < s.vertex.size(); i++)
    {
        Eigen::Vector2f v;
        cout << s.vertex[i][0] << s.vertex[i][1] << endl;
        v << s.vertex[i][0], s.vertex[i][1];
        // cout << v << "\n" << endl;
        v.normalize();
        // cout << v << "\n" << endl;



        float dot = d[0] * v[0] + d[1] * v[1];
        cout << v[0] << v[1] << endl;
        cout << dot << endl;
        if (tempdot < dot)
        {
            tempdot = dot;
            supportPtId = i;
        }

        cout << "supportPtId" << supportPtId << endl;
    }
    cout << "---------------------------" << endl;
    return s.vertex[supportPtId];
}

Eigen::Vector2f GJK::Support(const Eigen::Vector2f &d)
{
    cout << d << "\n" << endl;
    cout << mS2.vertex.size() << endl;
    return (SupportFunciton(mS2, d) - SupportFunciton(mS1, -d));
}

bool GJK::HandleSimplex(ConvexHull &simplex, Eigen::Vector2f &d)
{
    if (simplex.vertex.size() == 2)
    {
        return LineCase(simplex, d); // d 修改成朝向原點的向量
    }
    return TriangleCase(simplex, d);
}

bool GJK::LineCase(ConvexHull &simplex, Eigen::Vector2f &d)
{
    // 會進來這區代表 simplex.vertex.size() == 2
    Eigen::Vector2f AB(simplex.vertex[0] - simplex.vertex[1]);
    AB.normalize();

    Eigen::Vector2f AO(mOrigin - simplex.vertex[1]);
    AO.normalize();

    if (AB == AO || AB * (-1) == AO) // 原點在線上
    {
        return true;
    }

    d = prod(AB, AO);

    return false;
}

bool GJK::TriangleCase(ConvexHull &simplex, Eigen::Vector2f &d)
{
    // simplex 有三個點
    // 關鍵在於第三個點 這邊稱作A
    Eigen::Vector2f AB(simplex.vertex[1] - simplex.vertex[2]);
    Eigen::Vector2f AC(simplex.vertex[0] - simplex.vertex[2]);
    Eigen::Vector2f AO(mOrigin - simplex.vertex[2]);

    Eigen::Vector2f ABperp = -1 * prod(AB, AC);
    Eigen::Vector2f ACperp = -1 * prod(AC, AB);

    if (ABperp.dot(AO) > 0)
    {
        simplex.vertex[0] = simplex.vertex[2];
        simplex.vertex.pop_back();
        d = ABperp;
        return false;
    }
    else if (ACperp.dot(AO) > 0)
    {
        simplex.vertex[1] = simplex.vertex[2];
        simplex.vertex.pop_back();
        d = ACperp;
        return false;
    }
    return true; // 在線上或是中間區域
}

void GJK::PrintSimplex(const ConvexHull &simplex)
{
    for (int i = 0; i < simplex.vertex.size(); i++)
    {
        printf(
            "simplex vertex (x, y): %f, %f\n",
            simplex.vertex[i][0], simplex.vertex[i][1]);
    }
}

bool GJK::CollisionCheck()
{
    Eigen::Vector2f d(mS2.center - mS1.center);
    d.normalize();

    ConvexHull simplex;
    simplex.vertex.push_back(Support(d));

    // 一開始就選到原點直接return
    if (simplex.vertex[0] == mOrigin)
    {
        return true;
    }
    d << mOrigin - simplex.vertex[0];

    while (true)
    {
        d.normalize();
        cout << d << "\n"
             << endl;
        Eigen::Vector2f p(Support(d)); // 一條線求第三個點/一個點求第二個點
        
        cout << "come here" << endl;
        cout << Support(d) << "\n"
             << endl;
        if (p == mOrigin)
        {
            return true; // 如果是原點就代表有重疊
        }

        if (p.dot(d) < 0) // 第二個點貨第三個點沒有越過原點
        {
            return false;
        }
        simplex.vertex.push_back(p);

        if (HandleSimplex(simplex, d))
        {
            return true;
        }
    }
}

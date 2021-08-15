/************************************************
 *
 * file  : QCollision.h
 * author: bobding
 * date  : 2014-10-27
 * detail:
 * from  : https://github.com/boding/2d-collision
************************************************/
#ifndef _COLLISION_H_
#define _COLLISION_H_

#include <QVector2D>
#include <complex>

#define FLT_MIN 1.175494351e-38F
#define FLT_MAX 3.402823466e+38F

class Collision
{
public:
    struct AABB 
    {
        QVector2D lower;
        QVector2D upper;
    };

    struct OBB
    {
        QVector2D pivot;
        QSizeF size;
        float rotation; // in radian
    };

    struct Circle 
    {
        QVector2D pivot;
        float radius;
    };

    struct Convex 
    {
        QVector2D* vertices;
        size_t numVerts;
    };

    static bool isAABBOverlap(const AABB& a1, const AABB& a2)
    {
        QVector2D v1 = a1.lower - a2.upper;
        if (v1.x() > 0 || v1.y() > 0)
        {
            return false;
        }

        QVector2D v2 = a2.lower - a1.upper;
        if (v2.x() > 0 || v2.y() > 0)
        {
            return false;
        }

        return true;
    }

    // use separating axis theorem
    static bool isOBBOverlap(const OBB& o1, const OBB& o2)
    {
        // axes vector
        QVector2D a1( cos(o1.rotation), sin(o1.rotation));
        QVector2D a2(-sin(o1.rotation), cos(o1.rotation));
        QVector2D a3( cos(o2.rotation), sin(o2.rotation));
        QVector2D a4(-sin(o2.rotation), cos(o2.rotation));

        // edge length
        QSizeF l1 = o1.size * 0.5f;
        QSizeF l2 = o2.size * 0.5f;

        // vector between pivots
        QVector2D l = o1.pivot - o2.pivot;

        float r1, r2, r3, r4;

        // project to a1
        r1 = l1.width()  * fabs(QVector2D::dotProduct(a1, a1));
        r2 = l1.height() * fabs(QVector2D::dotProduct(a2, a1));
        r3 = l2.width()  * fabs(QVector2D::dotProduct(a3, a1));
        r4 = l2.height() * fabs(QVector2D::dotProduct(a4, a1));
        if (r1 + r2 + r3 + r4 <= fabs(QVector2D::dotProduct(l, a1)))
        {
            return false;
        }

        // project to a2
        r1 = l1.width()  * fabs(QVector2D::dotProduct(a1, a2));
        r2 = l1.height() * fabs(QVector2D::dotProduct(a2, a2));
        r3 = l2.width()  * fabs(QVector2D::dotProduct(a3, a2));
        r4 = l2.height() * fabs(QVector2D::dotProduct(a4, a2));
        if (r1 + r2 + r3 + r4 <= fabs(QVector2D::dotProduct(l, a2)))
        {
            return false;
        }

        // project to a3
        r1 = l1.width()  * fabs(QVector2D::dotProduct(a1, a3));
        r2 = l1.height() * fabs(QVector2D::dotProduct(a2, a3));
        r3 = l2.width()  * fabs(QVector2D::dotProduct(a3, a3));
        r4 = l2.height() * fabs(QVector2D::dotProduct(a4, a3));
        if (r1 + r2 + r3 + r4 <= fabs(QVector2D::dotProduct(l, a3)))
        {
            return false;
        }

        // project to a4
        r1 = l1.width()  * fabs(QVector2D::dotProduct(a1, a4));
        r2 = l1.height() * fabs(QVector2D::dotProduct(a2, a4));
        r3 = l2.width()  * fabs(QVector2D::dotProduct(a3, a4));
        r4 = l2.height() * fabs(QVector2D::dotProduct(a4, a4));
        if (r1 + r2 + r3 + r4 <= fabs(QVector2D::dotProduct(l, a4)))
        {
            return false;
        }

        return true;
    }

    static bool isCircleOverlap(const Circle& c1, const Circle& c2)
    {
        return (c2.pivot - c1.pivot).lengthSquared() <= (c1.radius - c2.radius) * (c1.radius - c2.radius);
    }

    // reference: http://www.codeproject.com/Articles/15573/D-Polygon-Collision-Detection
    // 1. find the axis perpendicular to the current edge.
    // 2. project both polygons on that axis.
    // 3. if these projections don't overlap, the polygons don't intersect.
    static bool isConvexOverlap(const Convex& c1, const Convex& c2)
    {
        // axes
        size_t numAxes = c1.numVerts + c2.numVerts;
        QVector2D* axes = new QVector2D[numAxes];
        size_t i = 0u, j = 0u;
        for (i = 0u; i < c1.numVerts; ++i, ++j)
        {
            QVector2D v = c1.vertices[i] - c1.vertices[(i + 1) % c1.numVerts];
            v.normalize();
            axes[j] = QVector2D(-v.y(), v.x());
        }

        for (i = 0u; i < c2.numVerts; ++i, ++j)
        {
            QVector2D v = c2.vertices[i] - c2.vertices[(i + 1) % c2.numVerts];
            v.normalize();
            axes[j] = QVector2D(-v.y(), v.x());
        }

        for (auto i = 0u; i < numAxes; ++i)
        {
            // project c1 vertices to axis
            float min1 = FLT_MAX, max1 = -FLT_MAX, ret1;
            for (auto j = 0u; j < c1.numVerts; ++j)
            {
                ret1 = QVector2D::dotProduct(c1.vertices[j], axes[i]);
                min1 = min1 > ret1 ? ret1 : min1;
                max1 = max1 < ret1 ? ret1 : max1;
            }

            // project c2 vertices to axis
            float min2 = FLT_MAX, max2 = -FLT_MAX, ret2;
            for (auto j = 0u; j < c2.numVerts; ++j)
            {
                ret2 = QVector2D::dotProduct(c2.vertices[j], axes[i]);
                min2 = min2 > ret2 ? ret2 : min2;
                max2 = max2 < ret2 ? ret2 : max2;
            }

            // overlap check
            float r1 = max1 - min1;
            float r2 = max2 - min2;
            float r = (max1 > max2 ? max1 : max2) - (min1 < min2 ? min1 : min2);
            if (r1 + r2 <= r)
            {
                delete[] axes;
                return false;
            }
        }

        delete[] axes;
        return true;
    }
};

#endif // _COLLISION_H_

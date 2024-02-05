#include "turtlelib/geometry2d.hpp"
#include <string>
#include <iostream>

namespace turtlelib
{

    double normalize_angle(double rad)
    {
        // Citation ---------- [2] ----------
        return rad - (std::ceil((rad + PI) / (2 * PI)) - 1) * 2 * PI; // 2.0, 1.0 pa
    }
    std::ostream &operator<<(std::ostream &os, const Point2D &p)
    {
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }

    std::istream &operator>>(std::istream &is, Point2D &p)
    {
        if (is.peek() == '[')
        {
            is.get();
        }
        is >> p.x >> p.y;

        if (is.peek() == ']')
        {
            is.get();
        }
        // this has a bug because newlines are left in the stream
        return is;
    }

    Vector2D Vector2D::normalize() const{
        double norm = sqrt(x*x + y*y); // const auto
       return {x/norm,y/norm};
    }

    Vector2D operator-(const Point2D &head, const Point2D &tail)
    {
        return {head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D &tail, const Vector2D &disp)
    {
        return {tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {
        if (is.peek() == '[')
        {
            is.get();
        }
        is >> v.x >> v.y;

        if (is.peek() == ']')
        {
            is.get();
        }

        // this has a bug because newlines are left in the stream
        return is;
    }

}

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <vector>

namespace csf {

struct Point {
    union {
        struct {
            double x;
			double y;
			double z;
        };
		double u[3];
    };

    Point() : x(0), y(0), z(0) {}
};

class PointCloud : public std::vector<Point>{
public:

    void computeBoundingBox(Point& bbMin, Point& bbMax);
};
}


#endif // ifndef _POINT_CLOUD_H_

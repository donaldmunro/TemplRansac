#include <string>
#include <vector>

#include "../Ransac.hh"
//#include "../RansacHelp.h"

struct Point
{
   double x;
   double y;
   Point(double _x, double _y) : x(_x), y(_y) {}
   Point(const Point& other)
   {
      if (&other != this)
      {
         x = other.x;
         y = other.y;
      }
   }

   Point& operator+=(const Point& other) { x += other.x; y += other.y; return *this; }

   const Point operator+(const Point& other) const { return Point(*this) += other; }

   Point& operator-=(const Point& other) { x -= other.x; y -= other.y; return *this; }

   const Point operator-(const Point& other) const { return Point(*this) -= other; }

   Point& operator*=(const double scalar) { x *= scalar;y *= scalar; return *this; }

   const Point operator*(const double scalar) const { return Point(*this) *= scalar; }
};

struct LineModel
{
   LineModel() : m(0), b(0) {}
   LineModel(double _m, double _b) : m(_m), b(_b) {}
   // operator= and copy constructor not needed as trivial bitwise copy should be OK for doubles.
   LineModel(const LineModel&) = default;
   LineModel& operator=(const LineModel&) = default;

   double m;
   double b;
};

struct LineData
{
   explicit LineData(std::vector<Point>& points_) : points(points_) {}

   std::vector<Point> points;
};

class LineRANSACEstimator// : public templransac::RANSACEstimator<LineData, LineModel>
//================================================================================
{
public:
   LineRANSACEstimator(Point u_, Point v_) : p1(u_), p2(v_) {};

   const int estimate(const LineData &samples, const std::vector<size_t> &sampleIndices,
                              templransac::RANSACParams &parameters,
                              std::vector<LineModel> &fitted_models) const;

   const void error(const LineData &samples, const std::vector<size_t> &sampleIndices, LineModel &model,
                            std::vector<size_t> &inlier_indices,
                            std::vector<size_t> &outlier_indices, double error_threshold) const;

private:
   Point p1, p2;
};




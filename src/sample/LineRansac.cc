#include "LineRansac.h"

const int LineRANSACEstimator::estimate(const LineData &samples, const std::vector<size_t> &sampleIndices,
                                        templransac::RANSACParams &parameters,
                                        std::vector<LineModel> &fitted_models) const
//--------------------------------------------------------------------------------------------------------
{
   const Point &P0 = samples.points[sampleIndices[0]];
   const Point &P1 = samples.points[sampleIndices[1]];
   double m = (P1.y - P0.y) / (P1.x - P0.x);
   double b = P1.y - m * P1.x;
   fitted_models.emplace_back(m, b);
   return 1;
}

inline const double dot(const Point& p1, const Point& p2) { return p1.x*p2.x + p1.y*p2.y; }

// Exercise for the reader: Implementing this in CUDA or OpenCL could make a big difference
// (provided the no of samples to check (sampleIndices.size()) was big enough).
const void LineRANSACEstimator::error(const LineData &samples, const std::vector<size_t> &sampleIndices,
                                      LineModel &line, std::vector<size_t> &inlier_indices,
                                      std::vector<size_t> &outlier_indices, double error_threshold) const
//-----------------------------------------------------------------------------------------------------------
{
   const size_t no = sampleIndices.size();
   const double error_threshold_square = error_threshold*error_threshold;
   for (size_t i=0; i<no; i++)
   {
      size_t index = sampleIndices[i];
      const Point &p3 = samples.points[index];
      double t = dot((p2 - p1), (p2-p3)) / dot( (p1-p2), (p1-p2) );
      Point v = p1*t + p2*(1-t) - p3;
      double error = dot(v,v);
      if (error < error_threshold_square)
         inlier_indices.push_back(i);
      else
         outlier_indices.push_back(i);
   }
}




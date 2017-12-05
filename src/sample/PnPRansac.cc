#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include "PnPRansac.h"

const int PnPRANSACEstimator::estimate(const PnPData& samples, const std::vector<size_t>& sampleIndices,
                                    templransac::RANSACParams& parameters, std::vector<PnPModel>& fitted_models) const
//---------------------------------------------------------------------------
{
   std::vector<cv::Point2f> points2d;
   std::vector<cv::Point3f> points3d;
   copy_points(samples, sampleIndices, points2d, points3d);
   cv::Mat rotation_vec, translations3x1;
   if (cv::solvePnP(points3d, points2d, K, D, rotation_vec, translations3x1))
   {
      fitted_models.emplace_back(rotation_vec, translations3x1);
      return 1;
   }
   return 0;
}

// Exercise for the reader: Implementing this in CUDA or OpenCL could make a big difference
// (provided the no of samples to check (sampleIndices.size()) was big enough).
const void PnPRANSACEstimator::error(const PnPData& samples, const std::vector<size_t>& sampleIndices,
                                    PnPModel& model, std::vector<size_t>& inlier_indices,
                                    std::vector<size_t>& outlier_indices,
                                    double error_threshold) const
//-------------------------------------------------------------------------------------------
{
   std::vector<cv::Point2f> points2d;
   std::vector<cv::Point3f> points3d;
   size_t no = copy_points(samples, sampleIndices, points2d, points3d);
   cv::Mat estimated_points(no, 2, CV_32FC1);
   cv::projectPoints(points3d, model.rotation_vec, model.translations3x1, K, D, estimated_points);
   const double error_threshold_square = error_threshold*error_threshold;
   const cv::Point2f* pestimated_pts = estimated_points.ptr<cv::Point2f>();
   for (size_t i=0; i<no; i++)
   {
      cv::Point2f v = points2d[i] - pestimated_pts[i];
      double error = v.dot(v);
      if (error < error_threshold_square)
         inlier_indices.push_back(i);
      else
         outlier_indices.push_back(i);
   }
}

PnPModel::PnPModel(const PnPModel &other)
//----------------------------------------
{
   if (&other != this)
   {
      if (! other.rotation_vec.empty())
         other.rotation_vec.copyTo(rotation_vec);
      if (! other.translations3x1.empty())
         other.translations3x1.copyTo(translations3x1);
   }
}


PnPModel &PnPModel::operator=(const PnPModel &other)
//---------------------------------------------------
{
   if (&other != this)
   {
      if (! other.rotation_vec.empty())
         other.rotation_vec.copyTo(rotation_vec);
      if (! other.translations3x1.empty())
         other.translations3x1.copyTo(translations3x1);
   }
   return *this;
}

//PnPModel &PnPModel::operator=(PnPModel &&other)
//{
//   if (&other != this)
//   {
//      if (! other.rotation_vec.empty())
//         other.rotation_vec.copyTo(rotation_vec);
//      other.rotation_vec.release();
//      if (! other.translations3x1.empty())
//         other.translations3x1.copyTo(translations3x1);
//      other.translations3x1.release();
//   }
//   return *this;
//}

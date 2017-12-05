#ifndef _POSERANSAC_H
#define _POSERANSAC_H

#include <limits>

#include <opencv2/core/core.hpp>
#include <utility>

#include "../Ransac.hh"
//#include "../RansacHelp.h"

struct PnPData
{
   PnPData(std::vector<cv::Point2f> &points2d, std::vector<cv::Point3f> &points3d) : points2d(points2d),
                                                                                     points3d(points3d) {}

   std::vector<cv::Point2f> &points2d;
   std::vector<cv::Point3f> &points3d;
};

struct PnPModel
{
   PnPModel() : rotation_vec(cv::Mat::zeros(4,1, CV_32FC1)), translations3x1(cv::Mat::zeros(3,1, CV_32FC1)) {}
   PnPModel(cv::Mat& rotation_vec_, cv::Mat& translation3x1_) : rotation_vec(rotation_vec_),
                                                                translations3x1(translation3x1_) {}
   PnPModel(const PnPModel& other);
   PnPModel& operator=(const PnPModel& other);
//   PnPModel& operator=(PnPModel&& other);

   cv::Mat rotation_vec, translations3x1;
};

class PnPRANSACEstimator// : public templransac::RANSACEstimator<PnPData, PnPModel>
//================================================================================
{
public:
   explicit PnPRANSACEstimator(cv::Mat K_ =cv::Mat::eye(3,3, CV_64FC1), cv::Mat D_ =cv::Mat::zeros(0, 0, CV_64FC1)) :
   K(std::move(K_)), D(std::move(D_)) {}

   const int estimate(const PnPData& samples, const std::vector<size_t>& sampleIndices,
                      templransac::RANSACParams& parameters, std::vector<PnPModel>& fitted_models) const;

   const void error(const PnPData& samples, const std::vector<size_t>& sampleIndices,
                   PnPModel& model, std::vector<size_t>& inlier_indices,
                   std::vector<size_t>& outlier_indices, double error_threshold) const;

private:
   cv::Mat K, D;

   static inline size_t copy_points(const PnPData& samples, const std::vector<size_t>& sampleIndices,
                                    std::vector<cv::Point2f>& points2d, std::vector<cv::Point3f>& points3d)
   //-----------------------------------------------------------------------------------------------------
   {
      const size_t no = sampleIndices.size();
      points2d.reserve(no);
      points3d.reserve(no);
      for (size_t i=0; i<no; i++)
      {
         size_t index = sampleIndices[i];
         points2d.push_back(samples.points2d[index]);
         points3d.push_back(samples.points3d[index]);
      }
      return no;
   }
};

#endif

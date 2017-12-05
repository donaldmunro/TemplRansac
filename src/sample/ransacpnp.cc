#include <iostream>
#include <sstream>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

#include "PnPRansac.h"

static int pointsCount = 500;
static double epsilon = 1.0e-2;

void generate3DPointCloud(std::vector<cv::Point3f>& points, cv::Point3f pmin = cv::Point3f(-1, -1, 5),
                          cv::Point3f pmax = cv::Point3f(1, 1, 10))
{
   std::random_device rd;
   std::mt19937 g(rd());
   std::uniform_real_distribution<float> x_dist(pmin.x, pmax.x);
   std::uniform_real_distribution<float> y_dist(pmin.y, pmax.y);
   std::uniform_real_distribution<float> z_dist(pmin.z, pmax.z);

   for (size_t i = 0; i < points.size(); i++)
      points[i] = cv::Point3f(x_dist(g), y_dist(g), z_dist(g));
}

void generateCameraMatrix(cv::Mat& cameraMatrix)
{
   const double fcMinVal = 1e-3;
   const double fcMaxVal = 100;
   std::random_device rd;
   std::mt19937 g(rd());
   std::uniform_real_distribution<double> m_dist(fcMinVal, fcMaxVal);
   cameraMatrix.create(3, 3, CV_64FC1);
   cameraMatrix.setTo(cv::Scalar(0));
   cameraMatrix.at<double>(0,0) = m_dist(g);
   cameraMatrix.at<double>(1,1) = m_dist(g);
   cameraMatrix.at<double>(0,2) = m_dist(g);
   cameraMatrix.at<double>(1,2) = m_dist(g);
   cameraMatrix.at<double>(2,2) = 1;
}

void generateDistCoeffs(cv::Mat& distCoeffs)
{
   std::random_device rd;
   std::mt19937 g(rd());
   std::uniform_real_distribution<double> d_dist(0.0, 1.0e-6);
   distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
   for (int i = 0; i < 3; i++)
      distCoeffs.at<double>(i,0) = d_dist(g);
}

void generatePose(cv::Mat& rvec, cv::Mat& tvec)
{
   const double minVal = 1.0e-3;
   const double maxVal = 1.0;
   std::random_device rd;
   std::mt19937 g(rd());
   std::uniform_real_distribution<double> p_dist(minVal, maxVal);
   rvec.create(3, 1, CV_64FC1);
   tvec.create(3, 1, CV_64FC1);
   for (int i = 0; i < 3; i++)
   {
      rvec.at<double>(i,0) = p_dist(g);
      tvec.at<double>(i,0) = p_dist(g);
   }
}

int main(int argc, char **argv) // runTest(RNG& rng, int mode, int method, const vector<Point3f>& points, const double* epsilon, double& maxError)
{
   cv::Mat rvec, tvec;
   std::vector<int> inliers;
   cv::Mat trueRvec, trueTvec;
   cv::Mat intrinsics, distCoeffs;
   std::vector<cv::Point3f> points, points_dls;
   points.resize(pointsCount);
   generate3DPointCloud(points);
   generateCameraMatrix(intrinsics);
   generateDistCoeffs(distCoeffs);
   generatePose(trueRvec, trueTvec);

   std::vector<cv::Point2f> projectedPoints;
   projectedPoints.resize(points.size());
   projectPoints(cv::Mat(points), trueRvec, trueTvec, intrinsics, distCoeffs, projectedPoints);
   std::random_device rd;
   std::mt19937 g(rd());
   std::normal_distribution<double> noise(0.0, 5);
   for (size_t i = 0; i < projectedPoints.size(); i++)
   {
      if (i % 20 == 0)
      {
         projectedPoints[i].x += noise(g);
         projectedPoints[i].y += noise(g);
      }
   }

   solvePnPRansac(points, projectedPoints, intrinsics, distCoeffs, rvec, tvec, false, pointsCount, 0.5f, 0.99, inliers,
                  CV_P3P);

   bool isTestSuccess = inliers.size() >= points.size()*0.95;
   double rvecDiff = norm(rvec-trueRvec), tvecDiff = norm(tvec-trueTvec);
   isTestSuccess = isTestSuccess && rvecDiff < epsilon && tvecDiff < epsilon;
   double error = rvecDiff > tvecDiff ? rvecDiff : tvecDiff;
   std::cout << "OpenCV solvePnPRansac " << ((isTestSuccess) ? "true" : "false") <<  " error " << error << " inliers: "
             << inliers.size() << " / " << points.size() << "(" << points.size()*0.95 << ")" << std::endl;
   std::cout << rvec << std::endl << tvec << std::endl;

   templransac::RANSACParams parameters(0.5, 0.99, 0.5);
   PnPRANSACEstimator estimator(intrinsics, distCoeffs);
   PnPData data(projectedPoints, points);
   std::vector<std::pair<double, PnPModel> > results;
   std::vector<std::vector<size_t>> inlier_indices;
   std::stringstream errs;
   double confidence = templransac::RANSAC(parameters, estimator, data, pointsCount, 4, 1, results, inlier_indices, &errs);
   if (confidence > 0)
   {
      PnPModel m = results[0].second;
      rvec = m.rotation_vec;
      tvec = m.translations3x1;
      rvecDiff = norm(rvec - trueRvec), tvecDiff = norm(tvec - trueTvec);
      error = rvecDiff > tvecDiff ? rvecDiff : tvecDiff;
      std::vector<size_t> inliers2 = inlier_indices[0];
      std::cout << "PnPRansac confidence " << confidence << " error " << error << " inliers: "
                << inliers2.size() << " / " << points.size() << "(" << points.size()*0.95 << ")"
                << " iterations " << parameters.iterations << std::endl;
      for (std::pair<double, PnPModel> pp : results)
         std::cout << pp.second.rotation_vec << std::endl << pp.second.translations3x1 << std::endl;
   }
   return isTestSuccess;
}

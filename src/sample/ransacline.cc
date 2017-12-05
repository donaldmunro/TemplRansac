#include <math.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "LineRansac.h"

int main(int argc, char **argv)
//-----------------------------
{
   Point u(-80, -100), v(50, 100), w(u);
   double t = 0;
   std::vector<Point> all_points;
   std::random_device rd;
   std::mt19937 g(rd());
   std::uniform_real_distribution<double> uniform(0, 1);
   std::normal_distribution<double> small_noise(0.0, 2);
   std::normal_distribution<double> big_noise(0.0, 15.0);
   do
   {
      w = u*(1.0 - t) + v*t;
      double p = uniform(g);
      if (p <= 0.5)
         all_points.emplace_back(w.x, w.y);
      else if ( (p > 0.5) && (p < 0.85) )
         all_points.emplace_back(w.x + small_noise(g), w.y + small_noise(g));
      else
         all_points.emplace_back(w.x + big_noise(g), w.y + big_noise(g));
      t += 0.01;
//      std::cout << w.x << ", " << w.y << std::endl;
   } while (t <= 1);

   templransac::RANSACParams parameters(0.2, 0.99, 0.5);
   LineRANSACEstimator estimator(u, v);
   LineData data(all_points);
   std::vector<std::pair<double, LineModel> > results;
   std::vector<std::vector<size_t>> inlier_indices;
   std::stringstream errs;
   double confidence = templransac::RANSAC(parameters, estimator, data, all_points.size(), 2, 1, results, inlier_indices,
                                           &errs);
   if (confidence > 0)
   {
      LineModel model = results[0].second;
      std::vector<size_t> inliers = inlier_indices[0];
      double m = model.m, b = model.b;
      std::cout << confidence << ": m= " << m << " b = " << b << " inlier count "
                << inliers.size() << "/" << all_points.size() << " iterations "
                << parameters.iterations << std::endl;
      std::ofstream svg("points.svg");
      if (! svg)
      {
         std::cout << "Error creating points.svg" << std::endl;
         return 0;
      }
      int w = 200, h = 200;
      svg << "<svg width=\"" << w << "px\" height=\"" << h << "px\">" << std::endl;
      double zoom = 1.0, wd = w, hd = h;
      for (Point& pt : all_points)
      {
         auto sx = static_cast<int>(round(zoom*pt.x + wd/2.0));
         auto sy = static_cast<int>(round(hd/2.0 - zoom*pt.y));
         svg << "<circle cx=\"" << sx << "\" cy=\"" << sy << "\" r=\"2\" style=\"stroke: red; fill: none;\"/>"
             << std::endl;
      }
      for (size_t i : inliers)
      {
         Point& pt = all_points[i];
         auto sx = static_cast<int>(round(zoom*pt.x + wd/2.0));
         auto sy = static_cast<int>(round(hd/2.0 - zoom*pt.y));
         svg << "<circle cx=\"" << sx << "\" cy=\"" << sy << "\" r=\"2\" style=\"stroke: green; fill: none;\"/>"
             << std::endl;
      }
      svg.close();
      std::cout << "Created points.svg" << std::endl;

      svg.open("line.svg");
      if (! svg)
      {
         std::cout << "Error creating line.svg" << std::endl;
         return 0;
      }
      svg << "<svg width=\"" << w << "px\" height=\"" << h << "px\">" << std::endl;
      double y1 = m * u.x + b, y2 = m * v.x + b;
      auto sx1 = static_cast<int>(round(zoom*u.x + wd/2.0));
      auto sy1 = static_cast<int>(round(hd/2.0 - zoom*y1));
      auto sx2 = static_cast<int>(round(zoom*v.x + wd/2.0));
      auto sy2 = static_cast<int>(round(hd/2.0 - zoom*y2));
      svg << "<line x1=\"" << sx1 << "\" y1=\"" << sy1 << "\" x2=\"" << sx2 << "\" y2=\"" << sy2
          << "\" style=\"stroke: black;\"/>" << std::endl;
      svg << "</svg>" << std::endl;
      svg.close();
      std::cout << "Created line.svg" << std::endl;
   }
}
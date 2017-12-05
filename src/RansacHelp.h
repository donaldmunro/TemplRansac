/*
Copyright (c) 2017 Donald Munro

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#ifndef TEMPLRANSAC_RANSACHELP_H
#define TEMPLRANSAC_RANSACHELP_H

#include <assert.h>

#include <vector>
#include <limits>

namespace templransac
{
   class RANSACParams;


/**
 * A helper class optionally only for use during development. Deriving the estimator from this ensures
 * the correct method definitions for methods  called by the main RANSAC routine by template "duck-typing"
 * are present in the estimator. The inheritance specification of the estimator should be removed
 * from the release code as virtual methods are slower due to the use of a vtable.
 * @tparam D
 * @tparam M
 */
 #ifndef NDEBUG
   template<typename D, typename M>
   class RANSACEstimator
      //===================
   {
   public:
      virtual const int estimate(const D &samples, const std::vector<size_t> &sampleIndices,
                                 RANSACParams &parameters, std::vector<M> &fitted_models) const
      { return 0; }

      virtual const void error(const D &samples, const std::vector<size_t> &sampleIndices,
                               M &model, std::vector<size_t> &inlier_indices,
                               std::vector<size_t> &outlier_indices,
                               double error_threshold) const
      {};
   };
}
 #endif

#endif //TEMPLRANSAC_RANSACHELP_H

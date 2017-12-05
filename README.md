# TemplRansac
TemplRANSAC is a templated C++ generic RANSAC implementation using modern C++ template
"duck-typing" instead of C++ abstract classes to specify the interface to be implemented
by the user. The emphasis is on simplicity and readability in order to make RANSAC
easier to understand.

## Using TemplRansac
The class is defined in a single header file Ransac.hh (in a namespace called templransac) with a single function:

```c++
template<typename Data, typename Model, typename Estimator>
   double RANSAC(RANSACParams& parameters, const Estimator estimator, const Data& data, const size_t datasize,
                 const size_t sample_size, const size_t no_results, std::vector<std::pair<double, Model>>& results,
                 std::vector<std::vector<size_t>>& inliers, std::stringstream* errs = nullptr)
```
The function returns a confidence value between 0 and 1.

### Template Parameters
**The Data template parameter type** acts as a data container container ie it contains containers with the
actual data eg for PnP pose the data class could be defined as

```c++
struct PnPData
{
    std::vector<cv::Point2f> &points2d;
    std::vector<cv::Point3f> &points3d;
};
```

The **Model template parameter type** contains the implementation specific information being optimised.
The class should implement a copy constructor and an operator= (unless it is certain the trivial bitwise copy
will do the right thing).

The **Estimator template parameter type**  contains the implementation specific sample estimation and error
calculations. It should implement two methods namely:

```c++
const int estimate(const D &samples, const std::vector<size_t> &sampleIndices,
                           RANSACParams &parameters, std::vector<M> &fitted_models) const { return 0; }
const void error(const D& samples, const std::vector<size_t>& sampleIndices,
                 M& model, std::vector<double>& errors,
                 std::vector<size_t>& inlier_indices, std::vector<size_t>& outlier_indices,
                 double error_threshold) const {};
```

The methods are called by template 'duck-typing' so the class does not need to be derived from a base class.
In both methods the sampleIndices vector contains indices into the samples data containers which indicate
the data items which the methods should use when executing their calculations.
The estimate method returns the number of models generated (usually 1 but many pose algorithms solving quadratic
or higher polynomials may return more than 1.)  (The RansacHelp.h header does define a abstract base class for
the estimator for use during development to make it easier to check the method signatures when using an IDE
but it should be removed or commented out in release builds.)

### Parameters

**parameters** - The RANSAC parameters. In the interests of readability and ease of use the number of
RANSAC parameters are held to a minimum and reflect those described in 4.7 of Multiple View Geometry
in Computer Vision (2nd Ed):

1. error_threshold:  Error threshold used to distinguish between inliers and outliers. As the user defines
the methods using this parameter it is up to the implementation to decide if it is a squared error or
not (when an L2 norm is used).
2. sample_inlier_probability: The probability that at least one sample is free of outliers
3. inlier_probability: The probability that a given data item is an inlier

**estimator** The RANSAC estimator as described in the Estimator template type above.
**data** The data container container as described in the Data template type above.
**datasize** The size of the data in the Data container
**sample_size** The minimum sample size to be used by RANSAC
**no_results** The number of results to return (allows multiple solutions to be returned sorted by the cost.
**results** The RANSAC results as a vector of pairs of cost and the best Model. If **no_results** > 1 then the
results are sorted in descending order. In both 1 and > 1 cases the best result in results[0].
**inliers** The inliers as a vector of vectors corresponding to the models for the same index in results.
**errs** A pointer to a stringstream which, if not null, will contain a human readable form of any errors.

## Samples
The src/samples directory contains the standard line fitting RANSAC example as well as an OpenCV
PnP example.

#ifndef MCVIOFRONTEND_DATA_H
#define MCVIOFRONTEND_DATA_H

#include "../Utils/EigenTypes.h"

namespace MCVIO
{
    using FeatureTrackerResults = 
        Eigen::aligned_map<int, 
                            Eigen::aligned_vector<Eigen::Matrix<double, 8, 1>>>;
    class FrontEndResult
    {

    };
}

#endif
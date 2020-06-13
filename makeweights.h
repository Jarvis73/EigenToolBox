#pragma once

#include <iostream>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;


//************************************
// Method:    makeweights
// FullName:  makeweights
// Access:    public 
// Returns:   Eigen::ArrayXXf
// Qualifier:
// Parameter: const Ref<const ArrayXXf> & edges
// Parameter: const Ref<const ArrayXXf> & vals
// Parameter: int valScale
// 
// Description: computes weights for a point and edge list based upon element values and 
//				Euclidean distance.The user controls the parameters valScale.
//************************************
ArrayXXf makeweights(const Ref<const ArrayXXf>& edges, const Ref<const ArrayXXf>& vals, int valScale = 90)
{
	ArrayXXf valDistances;
	if (valScale > 0)
	{
		auto left = vals(edges.col(0), all);
		auto right = vals(edges.col(1), all);
		auto temp = (left - right).rowwise().norm().eval();
		auto minVal = temp.minCoeff();
		auto maxVal = temp.maxCoeff();
		valDistances = ((temp - minVal) / (maxVal - minVal)).eval();
	}
	else 
	{
		valDistances = ArrayXXf::Zero(edges.rows(), 1);
		valScale = 0;
	}

	auto weights = (-(valDistances * valScale)).exp().eval();

	return weights;
}

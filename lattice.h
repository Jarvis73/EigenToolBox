#pragma once

#include <Eigen/Core>

#include "types.h"

using namespace Eigen;


//************************************
// Method:    lattice
// FullName:  lattice
// Access:    public 
// Returns:   std::pair<Eigen::ArrayXXf, Eigen::ArrayXXf>
// Qualifier:
// Parameter: Index h
// Parameter: Index w
// Parameter: unsigned int connect, 0 for 4-connected, 1 for 8-connected
// 
// Description: generates a 2D, "4-connected" Cartesian lattice with dimensions h and w.
//************************************
std::pair<ArrayXXf, ArrayXXf> lattice(Index h, Index w, unsigned int connect = 0)
{
	auto X = RowArrayXf::LinSpaced(w, 0, w - 1).replicate(h, 1).eval();
	auto Y = ArrayXf::LinSpaced(h, 0, h - 1).replicate(1, w).eval();

	// Get list of vertices
	auto XX = Map<ArrayXXf>(X.data(), X.size(), 1);
	auto YY = Map<ArrayXXf>(Y.data(), Y.size(), 1);
	ArrayXXf points(X.rows(), X.cols() + Y.cols());
	points << X, Y;

	// Get list of edges
	Index N = h * w;
	auto line = ArrayXf::LinSpaced(N, 0, N - 1).eval();
	auto mat = Map<ArrayXXf>(line.data(), h, w);
	auto clipped = ArrayXXf(mat.topRows(h - 1));
	auto topDownEdges = Map<ArrayXXf>(clipped.data(), (h - 1) * w, 1);
	auto leftRightEdges = Map<ArrayXXf>(line.data(), h * (w - 1), 1);

	Index num_edges = N * 2 - h - w;
	if (connect == 1)
		num_edges += (h - 1) * (w - 1) * 2;
	ArrayXXf edges = ArrayXXf(num_edges, 2);
	edges.topLeftCorner((h - 1) * w, 1) = topDownEdges;
	edges.topRightCorner((h - 1) * w, 1) = topDownEdges + 1;
	if (connect == 1)
	{
		Index ptr = (h - 1) * w;
		Index length = (h - 1) * (w - 1);
		auto bottomLeftBlock = ArrayXXf(mat.bottomLeftCorner(h - 1, w - 1));
		auto slashEdges = Map<ArrayXXf>(bottomLeftBlock.data(), length, 1);
		auto topLeftBlock = ArrayXXf(mat.topLeftCorner(h - 1, w - 1));
		auto backSlashEdges = Map<ArrayXXf>(topLeftBlock.data(), length, 1);
		edges.block(ptr, 0, h * (w - 1), 1) = leftRightEdges;
		edges.block(ptr, 1, h * (w - 1), 1) = leftRightEdges + h;
		ptr += h * (w - 1);
		edges.block(ptr, 0, length, 1) = slashEdges;
		edges.block(ptr, 1, length, 1) = slashEdges + (h - 1);
		edges.bottomLeftCorner(length, 1) = backSlashEdges;
		edges.bottomRightCorner(length, 1) = backSlashEdges + (h + 1);
	}
	else
	{
		edges.bottomLeftCorner(h * (w - 1), 1) = leftRightEdges;
		edges.bottomRightCorner(h * (w - 1), 1) = leftRightEdges + h;
	}

	return { std::move(points), std::move(edges) };
}

/**
 * @file rbpfMatrices.h
 * @brief Typedefs and defines for the ball-model based RBPF system
 * @author Alex Cunningham
 */

#pragma once

#include <LinearAlgebra.hpp>

// Constants for state sizes across all models
// If these are changed, make sure to modify the virtual functions, such as
// transitionModel(), which are written assuming specific state sizes
#define NSIZE 6 // size of state
#define MSIZE 6 // size of control input
#define SSIZE 2 // size of measurement

namespace rbpf {

// typedefs for fixed size matrix classes
// naming convention references N, M, and S from dimensions above, where
// MatrixSNd is an S x N matrix of doubles
typedef Eigen::Matrix<double,NSIZE,1> VectorNd;
typedef Eigen::Matrix<double,MSIZE,1> VectorMd;
typedef Eigen::Matrix<double,NSIZE,NSIZE> MatrixNNd;
typedef Eigen::Vector2d VectorSd;
typedef Eigen::Matrix2d MatrixSSd;
typedef Eigen::Matrix<double,SSIZE,NSIZE> MatrixSNd;
typedef Eigen::Matrix<double,NSIZE,SSIZE> MatrixNSd;

} // \namespace rbpf

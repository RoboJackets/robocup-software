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
// MatrixSNf is an S x N matrix of floats
//typedef Eigen::Matrix<float,NSIZE,1> VectorNf;
//typedef Eigen::Matrix<float,MSIZE,1> VectorMf;
//typedef Eigen::Matrix<float,NSIZE,NSIZE> MatrixNNf;
//typedef Eigen::Vector2f VectorSf;
//typedef Eigen::Matrix2f MatrixSSf;
//typedef Eigen::Matrix<float,SSIZE,NSIZE> MatrixSNf;

// switching to doubles
typedef Eigen::Matrix<double,NSIZE,1> VectorNf;
typedef Eigen::Matrix<double,MSIZE,1> VectorMf;
typedef Eigen::Matrix<double,NSIZE,NSIZE> MatrixNNf;
typedef Eigen::Vector2d VectorSf;
typedef Eigen::Matrix2d MatrixSSf;
typedef Eigen::Matrix<double,SSIZE,NSIZE> MatrixSNf;
typedef Eigen::Matrix<double,NSIZE,SSIZE> MatrixNSf;

} // \namespace rbpf

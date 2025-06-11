//
// Created by asus on 2025/5/20.
//

#ifndef POS_NAV_PROJECT_MATRIX_CALCULATE_H
#define POS_NAV_PROJECT_MATRIX_CALCULATE_H
#include <Eigen/Dense>
#include <iostream>

class Matrix{
public:
    Eigen::MatrixXd My_Matrix;
    Matrix(){}
    Matrix(int rows, int cols) : My_Matrix(rows, cols) {
        My_Matrix.setZero();
    }
    Matrix(int rows, int cols, double* pData) {
        My_Matrix = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(pData, rows, cols);
    }
    Matrix(const Eigen::MatrixXd& other) : My_Matrix(other) {}

    Matrix(int size, bool identity_flag) {
        if (identity_flag)
            My_Matrix = Eigen::MatrixXd::Identity(size, size);
        else
            My_Matrix = Eigen::MatrixXd::Zero(size, size);
    }

    Matrix(int size, double* diagonal_values) {
        My_Matrix = Eigen::MatrixXd::Zero(size, size);
        for (int i = 0; i < size; ++i) {
            My_Matrix(i, i) = diagonal_values[i];
        }
    }


    // 加法
    Matrix operator+(const Matrix& other) const {
        return Matrix(My_Matrix + other.My_Matrix);
    }

    // 减法
    Matrix operator-(const Matrix& other) const {
        return Matrix(My_Matrix - other.My_Matrix);
    }

    // 乘法（矩阵 × 矩阵）
    Matrix operator*(const Matrix& other) const {
        return Matrix(My_Matrix * other.My_Matrix);
    }

    // 标量乘法
    Matrix operator*(double scalar) const {
        return Matrix(My_Matrix * scalar);
    }

    // 转置
    Matrix transpose() const {
        return Matrix(My_Matrix.transpose());
    }

    // 求逆
    Matrix inverse() const {
        return Matrix(My_Matrix.inverse());
    }

    // 显示
    void print() const {
        std::cout << My_Matrix << std::endl;
    }

    // 获取行列数
    int rows() const { return My_Matrix.rows(); }
    int cols() const { return My_Matrix.cols(); }
};


#endif //POS_NAV_PROJECT_MATRIX_CALCULATE_H

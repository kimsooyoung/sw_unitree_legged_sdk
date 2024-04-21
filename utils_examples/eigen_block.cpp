#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix<double, 4, 4> matrix;
    matrix << 1, 2, 3, 4,
              5, 6, 7, 8,
              9, 10, 11, 12,
              13, 14, 15, 16;

    // (2, 1) 위치에서 2x2 크기의 블록 추출
    Eigen::Matrix<double, 2, 2> block = matrix.block<2, 2>(1, 0);

    std::cout << "Block from matrix:\n" << block << std::endl;

    return 0;
}
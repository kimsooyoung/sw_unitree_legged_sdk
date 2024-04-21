#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix<double, 3, 3> matrix1;
    matrix1 << 1, 2, 3,
               4, 5, 6,
               7, 8, 9;

    Eigen::Matrix<double, 3, 3> matrix2;
    matrix2 << 9, 8, 7,
               6, 5, 4,
               3, 2, 1;

    // 요소별 곱셈 수행
    Eigen::Matrix<double, 3, 3> result = matrix1.cwiseProduct(matrix2);

    std::cout << "Result of element-wise multiplication:\n" << result << std::endl;

    return 0;
}

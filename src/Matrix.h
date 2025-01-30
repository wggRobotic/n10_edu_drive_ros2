#ifndef __MATRIX_H
#define __MATRIX_H

#include <vector>
#include <string>

namespace edu
{

using Vec = std::vector<double>;
using Mat = std::vector<Vec>;

/**
 * @class Matrix
 * @brief Simple matrix algebra to avoid the binding to 3rd party libraries. This class is capable of inverting kinemtic matrices as used in our drive stack.
 * @author Stefan May
 * @date 29.01.2025
 */
class Matrix
{

public:

    /**
     * Constructor
     * @param[in] M Matrix elements as 2-dimensional vector
     */
    Matrix(Mat M);

    Matrix(Matrix& M);

    ~Matrix();

    int rows() const;

    int cols() const;

    double& operator () (unsigned int row, unsigned int col);

    friend Matrix operator * (Matrix &A, Matrix &B);

    friend Vec operator * (Matrix &A, Vec &v);

    Matrix subMatrix(int i1, int i2, int j1, int j2);

    Matrix transposed();

    Matrix inverse();

    Matrix pseudoInverse();

    void print(std::string name);

private:

    Matrix compose( const Matrix &A11, const Matrix &A12, const Matrix &A21, const Matrix &A22 );

    std::vector<Vec> _M;

};

} // namespace

#endif // __MATRIX_H
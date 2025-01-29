#include "Matrix.h"
#include <iostream>
#include <iomanip>

namespace edu
{

    Matrix::Matrix(Mat M)
    {
        _M.resize(M.size());
        for(int i=0; i<M.size(); i++)
            _M[i] = M[i];
    }

    Matrix::Matrix(Matrix& M)
    {
        //_M = M._M;
        _M.resize(M.rows());
        for(int i=0; i<M.rows(); i++)
            _M[i] = M._M[i];
    }

    Matrix::~Matrix()
    {
        
    }

    int Matrix::rows() const
    {
        return _M.size();
    }

    int Matrix::cols() const
    {
        if(rows()==0) return 0;
        return _M[0].size();
    }

    double& Matrix::operator () (unsigned int row, unsigned int col)
    {
        return _M[row][col];
    }

    Matrix operator * (Matrix &A, Matrix &B)
    {
        Mat M(A.rows(), Vec(B.cols(), 0.0));
        Matrix C(M);
        for(int i=0; i<A.rows(); i++)
        {
            for(int j=0; j<B.cols(); j++)
            {
                for(int k=0; k<A.cols(); k++)
                {
                    C(i, j) += A(i, k) * B(k, j);
                }
            }
        }
        return C;
    }

    Matrix operator - (Matrix &A, Matrix &B)
    {
        Mat M(A.rows(), Vec(B.cols(), 0.0));
        Matrix C(M);
        for(int r=0; r<A.rows(); r++)
        {
            for(int c=0; c<B.cols(); c++)
            {
                C(r, c) = A(r, c) - B(r, c);
            }
        }
        return C;
    } 

    Matrix Matrix::subMatrix(int i1, int i2, int j1, int j2)
    {
        int rows = i2 - i1 + 1, cols = j2 - j1 + 1;
        Mat M( rows, Vec( cols ) );
        for ( int i = i1, r = 0; i <= i2; i++, r++ )
        {
            auto it1 = _M[i].begin() + j1, it2 = _M[i].begin() + j2 + 1;
            copy( it1, it2, M[r].begin() );
        }
        return Matrix(M);
    }

    Matrix Matrix::compose( const Matrix &A11, const Matrix &A12, const Matrix &A21, const Matrix &A22 )
    {
        int k = A11.rows();           
        int n = k + A22.rows();
        Mat A( n, Vec( n ) );

        for ( int i = 0; i < k; i++ )
        {
            copy( A11._M[i].begin(), A11._M[i].end(), A[i].begin()     );
            copy( A12._M[i].begin(), A12._M[i].end(), A[i].begin() + k );
        }

        for ( int i = k; i < n; i++ )
        {
            copy( A21._M[i-k].begin(), A21._M[i-k].end(), A[i].begin()     );
            copy( A22._M[i-k].begin(), A22._M[i-k].end(), A[i].begin() + k );
        }

        return Matrix(A);
    }

    Matrix Matrix::transposed()
    {
        Mat B(cols(), Vec(rows()));
        for(int r=0; r<rows(); r++)
            for(int c=0; c<cols(); c++)
                B[c][r]= _M[r][c];
        return Matrix(B);
    }

    Matrix Matrix::inverse()
    {
        int r = rows();
        if (r == 1) 
        {
            double value = _M[0][0];
            // avoid division by zero
            if ( std::abs( value ) < 1e-16 )
            {
                std::cerr << "Non-invertible matrix passed" << std::endl;
                return Matrix(Mat{0});
            }
            Mat A(1, Vec( 1, 1.0 / value ));
            return Matrix(A);
        }
        
        // Strassen's matrix multiplication
        // Partition matrix into four sub matrices
        int k = r / 2;
        Matrix A11 = subMatrix(0, k - 1, 0, k - 1 );
        Matrix A12 = subMatrix(0, k - 1, k, r - 1 );
        Matrix A21 = subMatrix(k, r - 1, 0, k - 1 );
        Matrix A22 = subMatrix(k, r - 1, k, r - 1 );
        
        Matrix R1  = A11.inverse();
        Matrix R2  = A21 * R1;
        Matrix R3  = R1 * A12;
        Matrix R4  = A21 * R3;
        Matrix R5  = R4 - A22;
        Matrix R6  = R5.inverse();
        Matrix X12 = R3 * R6;
        Matrix X21 = R6 * R2;
        Matrix R7  = R3 * X21;
        Matrix X11 = R1 - R7;
        Matrix X22(R6);
        for ( auto &row : X22._M )
        {
            for ( auto &e : row ) e = -e;
        }
        return compose(X11, X12, X21, X22);
    }

    void Matrix::print()
    {
        for ( auto &row : _M )
        {
            for ( auto x : row ) std::cout << std::setw( 12 ) << ( std::abs( x ) < 1e-9 ? 0.0 : x );
                std::cout << '\n';
        }
        std::cout << std::endl;
    }

}
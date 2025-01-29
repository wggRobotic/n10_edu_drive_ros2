#include "../Matrix.h"
#include <iostream>

int main()
{
   // Data
   double r = 0.1;
   double lx = 0.3;
   double ly = 0.25;
   //pinv(A) = inv(A'A)*A'
   edu::Mat K = { { 1/r, -1/r, -(lx+ly)/r}, 
                  { -1/r, -1/r, -(lx+ly)/r },
                  { 1/r, 1/r, -(lx+ly)/r},
                  {-1/r, 1/r, -(lx+ly)/r},
                  {1/r, -1/r, -(ly+lx/2)/r},
                  {-1/r, -1/r, -(ly+lx/2)/r},
                  {1/r, 1/r, -(ly+lx/2)/r},
                  {-1/r, 1/r, -(ly+lx/2)/r}
                };
   edu::Matrix A(K);
   edu::Matrix At = A.transposed();
   std::cout << "A:" << std::endl;
   A.print();
   std::cout << "A':" << std::endl;
   At.print();
   edu::Matrix AtA = At * A;
   std::cout << "A'A:" << std::endl;
   AtA.print();
   edu::Matrix B = AtA.inverse();
   std::cout << "inv(A'A):" << std::endl;
   B.print();
   edu::Matrix C = B * At;
   std::cout << "inv(A'A)A':" << std::endl;
   C.print();
   /*matrix At = transpose(A);
   matrix AtA = matmul(At, A);
   matrix B = inverse(AtA);
   matrix C = matmul(B, At);
   
   
   
   //matrix B = inverse( A );
   print( "\nC:", C );
   print( "\nCheck CA=I:", matmul( C, A ) );*/
}
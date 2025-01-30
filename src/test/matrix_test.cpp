#include "../Matrix.h"
#include <iostream>

/**
 * @brief Simple test program to check functionality of Matrix class.
 *        It shows how a kinematic matrix could be inverted in order to
 *        get the inverse for odometry calculation. The model is tested with
 *        2-, 4-, 6- and 8-wheeled systems.
 *        Compile this program in this folder with:
 *        $ g++ matrix_test.cpp ../Matrix.cpp -o matrix_test
 * @author Stefan May
 * @date 30.1.2025
 */
int main ()
{
  double r = 0.1;
  double lx = 0.3;
  double ly = 0.25;
  edu::Mat K = { { 1 / r, -1 / r, -(lx + ly) / r },
                 { -1 / r, -1 / r, -(lx + ly) / r },
                 { 1 / r, 1 / r, -(lx + ly) / r },
                 { -1 / r, 1 / r, -(lx + ly) / r },
                 { 1 / r, -1 / r, -(ly + lx / 2) / r },
                 { -1 / r, -1 / r, -(ly + lx / 2) / r },
                 { 1 / r, 1 / r, -(ly + lx / 2) / r },
                 { -1 / r, 1 / r, -(ly + lx / 2) / r } };
  edu::Matrix A (K);
  A.print ("A =");
  edu::Matrix At = A.transposed ();
  At.print ("A' =");
  edu::Matrix AtA = At * A;
  AtA.print ("A'A =");
  edu::Matrix B = AtA.inverse ();
  B.print ("inv(A'A) =");
  edu::Matrix C = B * At;
  C.print ("inv(A'A)A' =");

  edu::Matrix D = A.pseudoInverse ();
  D.print ("pinv(A)");

  edu::Vec v (A.rows (), 1);
  edu::Vec vTwist = D * v;
  for (unsigned int i = 0; i < vTwist.size (); i++)
  {
    std::cout << vTwist[i] << " ";
  }
  std::cout << std::endl;
}
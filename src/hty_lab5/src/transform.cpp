#include "scan_matching_skeleton/transform.h"
#include <cmath>
#include "ros/ros.h"
#include <eigen3/Eigen/Geometry>
#include <complex>
using namespace std;


// transformer tous les points dans points par t et les strocker dans transformed_points

void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points) {
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++) {
    transformed_points.push_back(t.apply(points[i]));
    //printf("%f %transformed_points.back().r, transformed_points.back().theta);
  }
}
// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d) {
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c/a - b*b/(3*a*a);
  float q = 2*b*b*b/(27*a*a*a) + d/a - b*c/(3*a*a);
  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  complex<float> xi(-.5, sqrt(3)/2);
  complex<float> inside = sqrt(q*q/4 + p*p*p/27);
  complex<float> root;
  for (float k = 0; k < 3; ++k) {
    // get root for 3 possible values of k
    root = -b/(3*a) + pow(xi, k) * pow(-q/2.f + inside, 1.f/3.f) + pow(xi, 2.f*k) * pow(-q/2.f - inside, 1.f/3.f);
    //std::cout<<"RootTemp: "<< root<<sqqtd::endl;
    if (root.imag() != 0) { return root; }
  }
  return root;
}
// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e) {
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;
  // Reduce to depressed Quadratic
  float p = (8*a*c-3*b*b)/(8*a*a);
  float q = (b*b*b-4*a*b*c+8*a*a*d)/(8*a*a*a);
  float r = (-3*b*b*b*b+256*a*a*a*e-64*a*a*b*d+16*a*b*b*c)/(256*a*a*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8*p, 2*p*p-8*r, -q*q);
  complex<float> root1 = -b/(4*a) + ( sqrt(2.f*m) + sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root2 = -b/(4*a) + ( sqrt(2.f*m) - sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root3 = -b/(4*a) + (-sqrt(2.f*m) + sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root4 = -b/(4*a) + (-sqrt(2.f*m) - sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
  vector<complex<float>> roots { root1, root2, root3, root4 };
  float max_real_root = 0;
  for (complex<float> root: roots) {
    if(root.imag()==0){
      max_real_root = max(max_real_root, root.real());
    }
    //std::cout<<"Max real root:" << max_real_root<<std::endl;
  }
  return max_real_root;
}


void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans) {
  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // You can use the helper functions which are defined above for finding roots and transforming points as and when needed.
  // use helper functions and structs in transform.h and correspond.h
  // input : corresponds : a struct vector of Correspondene struct object defined in correspond.
  // input : curr_trans : A Transform object refernece
  // output : update the curr_trans object. Being a call by reference function, Any changes you make to curr_trans will be reflected in the calling function in the scan_match.cpp program/

// You can change the number of iterations here. More the number of iterations, slower will be the convergence but more accurate will be the results. You need to find the right balance.
  int number_iter = 2;

  for(int i = 0; i<number_iter; i++){

    //fill in the values of the matrics
    Eigen::MatrixXf M_i(2, 4);
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i;
    Eigen::Vector2f n_i; 

    // Fill in the values for the matrices
    Eigen::Matrix4f M, W;
    Eigen::MatrixXf g(1, 4);

    M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    g << 0, 0, 0, 0;

    for (Correspondence c : corresponds){

      float pi_0 = c.pj1->getX();
      float pi_1 = c.pj1->getY();
      n_i = c.getNormalNorm(); 
      M_i << 1, 0, c.p->getX(), -c.p->getY(), 0, 1, c.p->getY(), c.p->getX();
      C_i << n_i * n_i.transpose();
      pi_i << c.pj1->getX(), c.pj1->getY();  
      
      // printEigenShape(C_i);
      // printEigenShape(M_i);

      Eigen::MatrixXf tempCxM = C_i * M_i;
      M += M_i.transpose() * tempCxM;
      g -= 2 * pi_i.transpose() * tempCxM;

      // Define sub-matrices A, B, D from M
      Eigen::Matrix2f A, B, D;
      float A0 = 0; float A1 = 0; float A2 = 0; float A3 = 0; 
      float B0 = 0; float B1 = 0; float B2 = 0; float B3 = 0; 
      float D0 = 0; float D1 = 0; float D2 = 0; float D3 = 0;

      // printEigenShape(n_i);
      float temp_n0xn1 = n_i[0] * n_i[1];
      float temp_n0xn0 = n_i[0] * n_i[0];
      float temp_n1xn1 = n_i[1] * n_i[1];
      float temp_p0xn0xn0 = pi_0 * temp_n0xn0;
      float temp_p1xn0xn1 = pi_1 * temp_n0xn1;
      float temp_p1xn1xn1 = pi_1 * temp_n1xn1;
      float temp_p0xn0xn1 = pi_0 * temp_n0xn1;

      A0 += temp_n0xn0;
      A1 += temp_n0xn1;
      A2 += temp_p0xn0xn0 + temp_p1xn0xn1;
      A3 += temp_p1xn1xn1 + temp_p0xn0xn1;
      A << A0, A1, A2, A3;

      B0 += temp_p0xn0xn0 + temp_p1xn0xn1;
      B1 += temp_p1xn1xn1 - temp_p0xn0xn0;
      B2 += temp_p1xn1xn1 + temp_p0xn0xn1;
      B3 += pi_0 * n_i[1] * n_i[1] - temp_p1xn0xn1;
      B << B0, B1, B2, B3;

      D0 += pow(pi_0 * n_i[0] + pi_1 * n_i[1], 2);
      D1 += pi_0 * pi_1 * (n_i[1] * n_i[1] - n_i[0] * n_i[0]) + n_i[0] * n_i[1] * (pi_0 * pi_0 - pi_1 * pi_1);
      D2 += pi_0 * pi_1 * (n_i[1] * n_i[1] - n_i[0] * n_i[0]) + n_i[0] * n_i[1] * (pi_0 * pi_0 - pi_1 * pi_1);
      D3 += pow(pi_1 * n_i[0] - pi_0 * n_i[1], 2);
      D << D0, D1, D2, D3;

      //define S and S_A matrices from the matrices A B and D
      Eigen::Matrix2f S;
      Eigen::Matrix2f S_A;

      S << D-(B.transpose())*(A.inverse())*B;
      S_A << S.determinant() * S.inverse();

      //find the coefficients of the quadratic function of lambda
      float pow_2, pow_1, pow_0;
      Eigen::Matrix4f F_1, F_2, F_3;
      F_1.setZero(4,4);
      F_2.setZero(4,4);
      F_3.setZero(4,4);

      // printEigenShape(A.inverse());
      // printEigenShape(B);

      Eigen::MatrixXf temp_AinvxB = A.inverse()*B;
      Eigen::MatrixXf temp_AinvxBxSa = temp_AinvxB*S_A;   
      Eigen::MatrixXf temp_AinvxBxSaT = temp_AinvxB*S_A.transpose();

      F_1.topLeftCorner(2,2) = temp_AinvxB*B.transpose()*A.inverse().transpose();
      F_1.bottomLeftCorner(2,2) = -(temp_AinvxB).transpose();
      F_1.topRightCorner(2,2) = -temp_AinvxB;
      F_1.bottomRightCorner(2,2).setIdentity();
      
      F_2.topLeftCorner(2,2) = temp_AinvxBxSa*B.transpose()*A.inverse().transpose();
      F_2.bottomLeftCorner(2,2) = -(temp_AinvxBxSa).transpose();
      F_2.topRightCorner(2,2) = -temp_AinvxBxSa;
      F_2.bottomRightCorner(2,2) = S_A;
      
      F_3.topLeftCorner(2,2) = temp_AinvxBxSaT*S_A*B.transpose()*A.inverse().transpose();
      F_3.bottomLeftCorner(2,2) = -(temp_AinvxBxSaT*S_A).transpose();
      F_3.topRightCorner(2,2) = -temp_AinvxBxSaT*S_A;
      F_3.bottomRightCorner(2,2) = S_A.transpose()*S_A;
      
      //cout << g;
      pow_2 = (4*g*F_1*g.transpose())(0);
      pow_1 = (4*g*F_2*g.transpose())(0);
      pow_0 = (g*F_3*g.transpose())(0);

      // find the value of lambda by solving the equation formed. You can use the greatest real root function
      float lambda = greatest_real_root(0, 0, pow_2, pow_1, pow_0);

      //find the value of x which is the vector for translation and rotation
       Eigen::MatrixXf x(1,4);
       
       x = -((2*M + 2*lambda*W).inverse()).transpose()*(g.transpose());
      // Convert from x to new transform

      float theta = atan2(x(3), x(2));
      curr_trans= Transform(x(0), x(1), theta);
    }
  }
}

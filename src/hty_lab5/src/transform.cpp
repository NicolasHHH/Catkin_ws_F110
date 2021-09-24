#include "scan_matching_skeleton/transform.h"
#include <cmath>
#include "ros/ros.h"
#include <Eigen/Geometry>
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
  int number_iter = 1;

  for(int i = 0; i<number_iter; i++){

    //fill in the values of the matrics
    Eigen::MatrixXf M_i(2, 4);
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i;
    M_i<< 0, 0, 0, 0, 0, 0, 0, 0;
    C_i<< 0, 0, 0, 0;
    pi_i<< 0, 0;

    // Fill in the values for the matrices
    Eigen::Matrix4f M, W;
    Eigen::MatrixXf g(4,1);
    M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    g << 0, 0, 0, 0;

    for (Correspondence c: corresponds){
      pi_i = c.v;
      C_i =  c.getNormalNorm()*c.getNormalNorm().transpose();
      M_i << 1,0,pi_i(0),-pi_i(1),
             0,1,pi_i(1),pi_i(0);
      M = M + M_i.transpose()*C_i*M_i;
      g = g - (2 * pi_i.transpose()*C_i*M_i).transpose(); 
    }

    // M = M * 2;
    Eigen::Matrix2f I2;
    I2 << 1,0,0,1;

    // Define sub-matrices A, B, D from M
    Eigen::Matrix2f A, B, D;
    // M + 2*lambda*W = A,B,B.T,D+2lambda*I
    // M = A, B, B.T, D
    A = M.block(0,0,2,2);
    B = M.block(0,2,2,2);
    D = M.block(2,2,2,2);

    //define S and S_A matrices from the matrices A B and D
    Eigen::Matrix2f S; // D-B.T*A-1*B
    S = D - B.transpose()*A.inverse()*B; 
    Eigen::Matrix2f S_A; // det(S)*S-1
    S_A = S.determinant()*S.inverse();
    

    //find the coefficients of the quadratic function of lambda
    Eigen::MatrixXd Middle1(2,4);
    Eigen::MatrixXd Middle2(2,4);
    Eigen::MatrixXd Middle(4,4);
    Eigen::MatrixXd res(1,1);

    float pow_2; // 4*g.T* mat(A-1B*B.TA-T,-A-1B,sym,I)g;
    Middle1 << A.inverse()*B*B.transpose()*A.inverse().transpose(),(-1)*A.inverse()*B,
    Middle2 << (-1)*A.inverse()*B,I2;
    Middle << Middle1.transpose(),Middle2.transpose();
    res = g.transpose()*Middle.transpose()*g;
    pow_2 = 4*res(0,0);

    float pow_1; 
    Middle1 << A.inverse()*B*S_A*B.transpose()*A.inverse().transpose(),(-1)*A.inverse()*B*S_A,
    Middle2 << (-1)*A.inverse()*B*S_A,S_A;
    Middle << Middle1.transpose(),Middle2.transpose();
    res = g.transpose()*Middle.transpose()*g;
    pow_1 = 4*res(0,0);

    float pow_0;
    Middle1 << A.inverse()*B*S_A.transpose()*S_A*B.transpose()*A.inverse().transpose(), (-1)*A.inverse()*B*S_A.transpose()*S_A,
    Middle2 << (-1)*A.inverse()*B*S_A.transpose()*S_A,S_A.transpose()*S_A;
    Middle << Middle1.transpose(),Middle2.transpose();// - (S+2*lambda*I2).determinant()*(S+2*lambda*I2).determinant();
    res = g.transpose()*Middle.transpose()*g;
    pow_0 = 4*res(0,0);

    // find the value of lambda by solving the equation formed. You can use the greatest real root function
    float lambda = greatest_real_root(0,0,pow_2,pow_1,pow_0);


    //find the value of x which is the vector for translation and rotation
    Eigen::Vector4f x;

    Eigen::Matrix4f sum = -(2*M+2*lambda*W);
    x = sum.inverse().transpose()*g;
    // x = -(2M+2lambaW)**-T * g

    // Convert from x to new transform
    float theta = atan2(x(3), x(2));
    curr_trans= Transform(x(0), x(1), theta);
  }
}

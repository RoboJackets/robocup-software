//test program for opencv kalman filters

#include "KalmanFilter.hpp"

#include "opencv/cv.h"

#include <iostream>

using std::cout;
using std::endl;

int main (){
  cout << "Testing opencv" << endl;

  //creating a KalmanFilter
  KalmanFilter k(KalmanFilter::R_P);
  k.printState();

  //create a starting measurement matrix
  float z[] = {0.5, 0.5, 0.1};
  CvMat * meas =  cvCreateMat(3, 1, CV_32FC1);
  memcpy(meas->data.fl, z, sizeof(z));

  cout << "====== Single Measurement =====" << endl;

  for (unsigned int i = 0; i<5; ++i){

    //run predict
    cout << "Predicting..." << endl;
    k.predict();
    //k.printState();
    
    //run correct
    cout << "Correcting..." << endl;
    k.correct(meas);
    k.printState();
  }

  cout << "====== Moving Measurement =====" << endl;
  float zm[3] = {0.5, 0.5, 0.1};

  for (unsigned int i = 0; i<5; ++i){

    //adjust measurement
    zm[0] = zm[0]+0.2;
    zm[1] = zm[1]+0.2;
    zm[2] = zm[2]+0.1;

    CvMat * meas =  cvCreateMat(3, 1, CV_32FC1);
    memcpy(meas->data.fl, zm, sizeof(zm));

    //run predict
    cout << "Predicting..." << endl;
    k.predict();
    //k.printState();
    
    //run correct
    cout << "Correcting..." << endl;
    k.correct(meas);
    k.printState();
  }



  cout << "Test complete" << endl;

  return 0;
}

#include "KalmanFilter.hpp"

#include "opencv/cv.h"
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
// using std::cout;
// using std::endl;

// Constructor
KalmanFilter::KalmanFilter(filterConfig config, int id, string mat_file) {
    // get matrices for various interpolation values from file
    ifstream file;
    file.open(mat_file.c_str());
    Vector<Vector> mat_data;
    string mat_line;
    if (file.isOpen()) {
        while (!file.eof()) {
            getline(file, mat_line);
        }
    }

    // set id for system
    this->id = id;

    // Initialize filter
    // TODO: Adjust this to account for extra state vars in ball
    this->kalman = cvCreateKalman(9, 9, 0);

    // check for type
    this->config = config;
    if (config == ROBOT) {
        cout << "Creating Robot Filter, Entity ID: " << id << endl;
        // create A system matrix

    } else if (config == BALL) {
        cout << "Creating Ball Filter, Entity ID: " << id << endl;
    }
}

KalmanFilter::~KalmanFilter();

// use functions

// Returns the most recent corrected state
pva_point KalmanFilter::getCurState();

// Returns an interpolated prediction
// This value is adjusted to account for time since last
// filter update
pva_point KalmanFilter::getInterpState();

// Adds a new measurement
// Returns the corrected pva state
pva_point KalmanFilter::update(p_point meas);

// /// Basic Constructor
// /// Allows for different filter configurations
// /// Depending on model
// /// TODO: Need to implement more sophisticated models
// KalmanFilter::KalmanFilter(entityType type)
// {
//   this->config = type;

//   //set up the kalman filter with
//   //system models
//   switch (type)
//     {
//     case R_P:
//       cout << "Creating P model for Robot" << endl;
//       this->kalman = cvCreateKalman(3, 3, 0);
//       cvSetIdentity( this->kalman->transition_matrix, cvRealScalar(1));
//       //cvSetIdentity( this->kalman->control_matrix, cvRealScalar(1));
//       break;
//     case R_PV:
//       cout << "Creating PV model for Robot" << endl;
//       cout << "Not Implemented!" << endl;
// //       this->kalman = cvCreateKalman(6, 3, 0);
//       break;
//     case R_PVA:
//       cout << "Creating PVA model for Robot" << endl;
//       cout << "Not Implemented!" << endl;
// //       this->kalman = cvCreateKalman(9, 3, 0);

//       break;
//     case B_P:
//       cout << "Creating P model for Ball" << endl;
//       this->kalman = cvCreateKalman(2, 2, 0);
//       cvSetIdentity( this->kalman->transition_matrix, cvRealScalar(1));
//       cvSetIdentity( this->kalman->control_matrix, cvRealScalar(1));
//       break;
//     case B_PV:
//       cout << "Creating PV model for Ball" << endl;
//       cout << "Not Implemented!" << endl;
// //       this->kalman = cvCreateKalman(4, 2, 0);

//       break;
//     case B_PVA:
//       cout << "Creating PVA model for Ball" << endl;
//       cout << "Not Implemented!" << endl;
// //       this->kalman = cvCreateKalman(6, 2, 0);

//       break;
//     default:
//       cout << "Invalid input!" << endl;
//     }

//   //Use scaled identities for other matrices
//   cvSetIdentity( kalman->measurement_matrix, cvRealScalar(1) );
//   cvSetIdentity( kalman->process_noise_cov, cvRealScalar(1e-5) );
//   cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1e-1) );
//   cvSetIdentity( kalman->error_cov_post, cvRealScalar(1));

//   //set initial state

//   //initialize the timestamp
//   this->updateTimestamp = clock();

//   cout << "Kalman Filter Created!" << endl;

// }

// void KalmanFilter::printState()
// {
//   cout << "Predicted Point:" << endl;
//   cout << "Px = " << cvmGet(this->kalman->state_pre, 0, 0) << endl;
//   cout << "Py = " << cvmGet(this->kalman->state_pre, 1, 0) << endl;
//   cout << "Ptheta = " << cvmGet(this->kalman->state_pre, 2, 0) << endl;

//   cout << "Corrected Point:" << endl;
//   cout << "Px = " << cvmGet(this->kalman->state_post, 0, 0) << endl;
//   cout << "Py = " << cvmGet(this->kalman->state_post, 1, 0) << endl;
//   cout << "Ptheta = " << cvmGet(this->kalman->state_post, 2, 0) << endl;
// }

// void KalmanFilter::printBallState()
// {
//   cout << "Predicted Point:" << endl;
//   cout << "Px = " << cvmGet(this->kalman->state_pre, 0, 0) << endl;
//   cout << "Py = " << cvmGet(this->kalman->state_pre, 1, 0) << endl;

//   cout << "Corrected Point:" << endl;
//   cout << "Px = " << cvmGet(this->kalman->state_post, 0, 0) << endl;
//   cout << "Py = " << cvmGet(this->kalman->state_post, 1, 0) << endl;
// }

// KalmanFilter::~KalmanFilter()
// {
//   cvReleaseKalman(&(this->kalman));
// }

// CvKalman* KalmanFilter::getFilter()
// {
//   return this->kalman;

// }

// ///Predicts the current position, uses time since last predict
// /// returns predicted state
// /// TODO: allow for multiple time steps for interpolation
// void KalmanFilter::predict(const CvMat* control)
// {
//   cvKalmanPredict(this->kalman, control);
// }

// ///Integrates measurement into predicted belief
// /// returns corrected state
// void KalmanFilter::correct(CvMat* meas)
// {
//   cvKalmanCorrect(this->kalman, meas);
// }

// void KalmanFilter::setState(float * state)
// {
//   memcpy(this->kalman->data.fl, state, sizeof(state));
// }

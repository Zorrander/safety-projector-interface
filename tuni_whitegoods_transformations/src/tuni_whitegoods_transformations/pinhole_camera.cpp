#include "tuni_whitegoods_transformations/pinhole_camera.h"

PinholeCamera::PinholeCamera() {
  /*
  K_ = cv::Matx33d(607.4446411132812, 0, 640.501220703125,
                   0, 607.5540771484375, 362.7770080566406,
                   0,         0,                1          );
  */
  K_ = cv::Matx33d(912.3347778320312, 0.0, 956.4423828125, 0.0,
                   912.3259887695312, 543.52294921875, 0, 0, 1);
}

double PinholeCamera::cx() const { return K_(0, 2); }

double PinholeCamera::cy() const { return K_(1, 2); }

double PinholeCamera::fx() const { return K_(0, 0); }

double PinholeCamera::fy() const { return K_(1, 1); }

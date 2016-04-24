#include <vector>
#include <iostream>
#include <libMesaSR.h>
#include <opencv/cv.h>
#include <unistd.h>

// See http://forum.mesa-imaging.ch/viewtopic.php?f=33&t=169
// This will evaluate the camera intrinsics parameters
//
// 1. Fill the range image (obtained with SR_GetImage()) with random 16 bit values.
// 2. Use SR_CoordTrfFlt() to convert the range image to a XYZ image.
// 3. Initialize the camera matrix, which contains the intrinsic parameters fx, fy, cx and cy, to suitable starting values.
// 4. OpenCV calibration using an intrinsic guess.

using namespace cv;
using namespace std;

void getSRParams(SRCAM cam, Mat& cameraMatrix, Mat& distCoeffs, Mat& rvec, Mat& tvec)
{

        int rows = SR_GetRows(cam), cols = SR_GetCols(cam);

        if(rows*cols<=0) {
          cerr << "ERROR: Camera was not opened" << endl;
        }

        // 1. get image pointer and fill with random values
        unsigned short* values = (unsigned short*)SR_GetImage(cam, 0);
        RNG rng;
        Mat _values(rows, cols, CV_16UC1, values);
        cout << "Generate random image" << endl;
        rng.fill(_values, RNG::UNIFORM, Scalar::all(10), Scalar::all(65000));
        const int s = 3*sizeof(float);
        vector<float> xyz(rows*cols*3, 0);

        // 2. convert to an XYZ image
        SR_CoordTrfFlt(cam, &xyz[0], &xyz[1], &xyz[2], s, s, s);

        // 3. initialize camera matrix
        // fit a line to x and y as camera matrix approximation
        // this has to be done in order to run the calibration with non-planar points
        Vec3f* ptr = (Vec3f*)&xyz[0];
        vector<Vec3f> objectPoints(rows*cols);
        vector<Vec2f> imagePoints(rows*cols),
                      normalizedImPointsX(rows*cols),
                      normalizedImPointsY(rows*cols);

        int i=0, j,k;
        for(j=0; j<rows; ++j)
        {
            for(k=0; k<cols; ++k, ++i)
            {
                    objectPoints[i]=ptr[j*cols+k];
                    imagePoints[i]= Vec2f((float)k+0.5f, (float)j+0.5f);
                    normalizedImPointsX[i] = Vec2f(ptr[j*cols+k][0]/ptr[j*cols+k][2], imagePoints[i][0]);
                    normalizedImPointsY[i] = Vec2f(ptr[j*cols+k][1]/ptr[j*cols+k][2], imagePoints[i][1]);
            }
        }
        double p=0, reps=0.01, aeps=0.01, ax, bx, ay, by;
        Vec4f linex, liney;
        cout << "Fit line" << endl;
        fitLine(normalizedImPointsX, linex, CV_DIST_L2, 0, reps, aeps);
        fitLine(normalizedImPointsY, liney, CV_DIST_L2, 0, reps, aeps);
        ax = linex[1]/linex[0]; bx = cols/2;
        ay = liney[1]/liney[0]; by = rows/2;
        cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0));
        double* _cm = (double*)cameraMatrix.ptr();
        _cm[0] = fabs(ax); _cm[2] = bx; _cm[4] = fabs(ay); _cm[5] = by; _cm[8]=1.0;
        const int distCoeffsNum = 8;
        distCoeffs = Mat(1, distCoeffsNum, CV_64FC1, Scalar::all(0));
        rvec = tvec = Mat(1, 3, CV_32FC1, Scalar::all(0));

        // 4. calibrate
        vector<vector<Vec3f> > _op(1, objectPoints);
        vector<vector<Vec2f> > _ip(1, imagePoints);
        vector<Mat> _rvec, _tvec;
        cout << "run camera calibration" << endl;
        calibrateCamera(_op, _ip, Size(cols,rows), cameraMatrix, distCoeffs, _rvec, _tvec, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_RATIONAL_MODEL);
        rvec = _rvec[0];
        tvec = _tvec[0];
}

void printHelp(char** argv) {
  cout << "Usage: " << argv[0] << " {-u | -e ip_addr} " << endl;
  cout << " -u            use usb" << endl;
  cout << " -e ip_addr    use ethernet, the camera is at ip_addr" << endl;
  cout << " -e            show this help" << endl;
}

int main(int argc, char** argv) {


  int c;
  bool useUsb = false;
  bool useEth = false;
  char const * ipAddr = NULL;

  while ((c = getopt (argc, argv, "hue:")) != -1)
    switch (c) {
      case 'u':
        useUsb = true;
        break;
      case 'e':
        useEth = true;
        ipAddr = optarg;
        break;
      case 'h':
        printHelp(argv);
        return 0;
      case '?':
        cerr << "Unable to parse arguments" << endl;
        printHelp(argv);
        return -1;
      default:
        abort ();
    }


  int res = 0;

SRCAM srCam;

  if(useEth) {
      // ---[ set callback function ] ---
    //SR_SetCallback((SR_FuncCB *)SR_ROS_FuncCB);
    cout << "Opening at " << ipAddr << endl;
    res = SR_OpenETH (&srCam, ipAddr);
  } else if (useUsb) {
    res = SR_OpenUSB (&srCam, 0); //returns the device ID used in
  } else {
    cerr << "No connection method (usb, eth) selected. Unable to open camera" << endl;
  }

  if (res <= 0)
  {
    SR_Close (srCam);
    cerr << "Error: Unable to open device" << endl;
    return -1;
  }

  Mat cameraMatrix;
  Mat distCoeffs;
  Mat rvec;
  Mat tvec;

  cout << "Start get params procedure" << endl;
  getSRParams(srCam, cameraMatrix, distCoeffs, rvec, tvec);
  cout << "Done." << endl;
  cout << endl << "Results:" << endl;

  cout << "Camera Matrix: " << endl << endl;
  cout << cameraMatrix << endl << endl;

  cout << "Distortion Coefficients: " << endl;
  cout << distCoeffs << endl << endl ;

  SR_Close(srCam);
  return 0;
}

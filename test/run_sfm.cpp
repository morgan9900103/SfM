#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/highgui/highgui.hpp>

#include "sfm.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    if(argc != 3) {
        cout << "usage: run_sfm image1 image2" << endl;
        return 1;
    }
    
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);  // TUM dataset
    
    SFM::Ptr sfm(new SFM());
    sfm->K_ = K;
    sfm->img_1_ = img_1;
    sfm->img_2_ = img_2;
    sfm->sfm();
    
    return 0;
}

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>

using namespace cv;
using namespace std;

class Recognizer{
    public:
      Ptr<FaceRecognizer> model;
      vector<Mat> imgs;
      vector<int> labels;
      Size im_size;
      string db_abs_path;
       string db_rel_path;
      string fn_csv;
    int class_count;
    Recognizer();
    bool prepare(string);
    bool find_match(Mat im,int& label,double& confidence);
    void find_multiple(Mat im,vector<Rect> faces,vector<int>& prediction,vector<double>& confidence);
    bool add_class( vector<Mat>& imgs);
    string ToString(int n);

};

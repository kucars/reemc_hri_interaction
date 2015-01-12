#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include "Recognizer.h"
using namespace cv;
using namespace std;
Recognizer::Recognizer()
{
    model=createLBPHFaceRecognizer(1,8,8,8,110.0);
    class_count=0;
    im_size=Size();
}

bool Recognizer::prepare(string a)
{

    Mat temp;
    class_count=1;
    int prev_class;
    fn_csv=a;
    db_path=fn_csv.substr(0,fn_csv.rfind("/")+1);
    ifstream file(a.c_str(), ifstream::in);
    try {
    //reading the csv file
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, ';');
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            temp=imread(db_path+path, 0);
            if(temp.empty())
                continue;
            if(imgs.empty()) //first image
                {
                    im_size.width=temp.cols;
                    im_size.height=temp.rows;
                }
            resize(temp,temp,im_size);
            imgs.push_back(temp);
            labels.push_back(atoi(classlabel.c_str()));
            if(atoi(classlabel.c_str())!=prev_class)
            {
                prev_class=atoi(classlabel.c_str());
                class_count++;
            }
        }
    }
    db_path+=path.erase(path.rfind("/"));
    if(imgs.empty() || labels.empty())
        return false;
    } catch (cv::Exception& e) {
        cout << "Error reading file \"" << fn_csv << "\": " << e.msg << endl;
        return false;
    }
    model->train(imgs, labels);
    return true;
}

bool Recognizer::find_match(Mat im,int& prediction,double& confidence)
{
    model->predict(im,prediction,confidence);
    if(prediction==-1)
        return false;
    else
    confidence=confidence/100.0;
    return true;
}

void Recognizer::find_multiple(Mat im,vector<Rect> faces,vector<int>& prediction,vector<double>& confidence)
{
    confidence.clear();
    prediction.clear();
    int pred;
    double con;
    Mat face;
    for(int i=0;i<faces.size();i++)
    {
        face=im(faces[i]);
        resize(face,face,im_size);
        if(find_match(im(faces[i]),pred,con))
        {
            prediction.push_back(pred);
            confidence.push_back(con);
        }
        else
        {
            prediction.push_back(-1);
            confidence.push_back(0.0);
        }

    }
}

bool Recognizer::add_class( vector<Mat>& imgs)
{
    string path=db_path+"/"+ToString(++class_count+1);
    vector<int> params;
    params.push_back(CV_IMWRITE_PXM_BINARY);
    ofstream outfile;
    outfile.open(fn_csv.c_str(), std::ios_base::app);
    if(imgs.size()<10)
    {
        cerr<<"not enough samples \n";
        return false;
    }
    string command="mkdir "+path;
    system(command.c_str());
    for(int i=0;i<imgs.size();i++)
    {
        resize(imgs[i],imgs[i],im_size);
        imwrite(path+"/"+ToString(i+1)+".pgm_",imgs[i],params);
        outfile<<path+"/"+ToString(i+1)+".pgm;"<<class_count<<"\n";
    }
}

string Recognizer::ToString(int number)
{
 stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}


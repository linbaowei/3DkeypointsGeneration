#include <iostream>
#include "GL/glew.h"
#include "SiftGPU/SiftGPU.h"
#include "SiftGPU/GlobalUtil.h"

#include "cv.h"
#include "highgui.h" 
using namespace cv;
using namespace std;
int main(int argc, char **argv) {
    std::cout << argv[1] << std::endl;
    Mat imageForShow = imread(argv[1]);
    if(imageForShow.empty())
	      return -1;	
	unsigned char *data=imageForShow.ptr();    
	
	SiftGPU  sift;
	
	char * argg[] ={ "-fo", "-1", "-v", "1"};//,"-d","10"
	//-fo -1,  starting from -1 octave
	//-v 1,  only print out # feature and overall time
	sift.ParseParam(4, argg);
    
	int support = sift.CreateContextGL();	//call VerfifyContexGL instead if using your own GL context
	//int support = sift->VerifyContextGL();
	
	if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)	
	    return 0;
	sift.RunSIFT(imageForShow.size().width, imageForShow.size().height, data, GL_RGB, GL_UNSIGNED_BYTE);
	int num = sift.GetFeatureNum();
	cout << "sift feature counts: " << num << endl;
	sift.SaveSIFT((string(argv[1]).substr(0,string(argv[1]).length()-4)+".key").c_str());	
	
	
	
//       //allocate memory for readback
// 
//       vector<float> descriptors(128*num);
//       vector<SiftGPU::SiftKeypoint> keys(num);
//       //read back keypoints and normalized descritpros
// 
//       //specify NULL if you don’t need keypionts or descriptors
// 
//       sift.GetFeatureVector(&keys[0], &descriptors[0]);
//       Point2f pt;
//       for(int i = 0; i < keys.size(); i ++)
//       {
// 	pt.x = (int)keys.at(i).x;
// 	pt.y = (int)keys.at(i).y;
// 	circle(imageForShow, pt, 1, CV_RGB(0, 255, 0), -1);
//       }
// 	namedWindow("result",CV_WINDOW_AUTOSIZE);
// 	imshow("result", imageForShow);
// 	imwrite("result.jpg", imageForShow);
    return 0;
}

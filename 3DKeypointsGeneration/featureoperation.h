/*
    Copyright (c) 2012 <Baowei Lin> <lin-bao-wei@hotmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/


#ifndef FEATUREOPERATION_H
#define FEATUREOPERATION_H

#include <iostream>
using namespace std;

#include "cv.h"
#include "highgui.h"
using namespace cv;

#include "rply.h"

struct mats{
	Mat matrix3d;
	Mat matrixcolor;
	Mat normalVec;  //3x points num.
};

typedef struct {
	double x;
	double y;
	double z;
}aPoint;

struct correspondingFrom3dto2d
{
	Point3d point3d;
	Point3d point2d;
	Mat normalVec;
};

struct sift_feature{
	Mat point3d;	//3d points
	Mat point2d;    //keypoint
	double sacle;
	double orientation;
	vector <int> features1;   //line one
	vector <int> features2;	//line two
	vector <int> features3;
	vector <int> features4;
	vector <int> features5;
	vector <int> features6;
	vector <int> features7;    //line seven
};

struct corresponding3D2Dpoints_new{
	cv::Point3d point3D;
	Mat point2d;    //keypoint
	double sacle;
	double orientation;
	Mat normalVec; //3x1
	vector <int> features1;   //line one
	vector <int> features2;	//line two
	vector <int> features3;
	vector <int> features4;
	vector <int> features5;
	vector <int> features6;
	vector <int> features7;    //line seven
};

class featureOperation
{

	double visiableAngle;
	string plypath;
	string allInOne3DpointsFile;
	string trainingImagesPath;
	string traingImagesPosePath;
	string output_folder;
	
public:
	featureOperation();
	featureOperation(double Angle);
	featureOperation (double Angle, 
				string plyFileof3DpointCloudFile, 
				string output_folder, 
				string trainingImagesFolder, 
				string traingImagesPoseFolder);
	
	string DoubleToString(double i);  
	string FloatToString(float i);
	string IntToString(int i); 
	
	mats read3Dpoints(const string &plyFilePath);
	int readPLY(aPoint* points, 
			aPoint* normal, 
			aPoint* color,
			const char* input_ply);
	int readPointnum(aPoint* points, 
				aPoint* normal, 
				const char* input_ply);	
  
	 /**
	 *   return the cos() value of angle which between view direction and normal vector
	 */
	double getNormalVecAngle(Mat viewDirection,
						Mat normalVec);
	
	
	/**
	  *  return projected 2D  points from 3D points after the normal vector calculation
	  */
	vector<correspondingFrom3dto2d> get2DpointsAfterNormalVec(mats matrix,
													Mat projectiomMatrix,
													string image);
	
	/**
	  *  return  one projected 2D point after the normal vector calculation
	  */
	double getOne2DpointsAfterNormalVec(mats matrix,
								      Mat projectiomMatrix);
	
	
	corresponding3D2Dpoints_new average (const corresponding3D2Dpoints_new & arg1,
								const corresponding3D2Dpoints_new   & arg2);
	
	vector<sift_feature> read_siftFeatures(string siftPath);
	vector<corresponding3D2Dpoints_new> read_siftFeatures_new(string siftPath);	
	
	int projection_normal(Mat matrix3D,
					Mat position,
					string testimage, int num);

	int projection_normal();
	
	void generate3DpointswithFeatures(int FLAG,
								int threshhold, 
								double distance);
	void generate3DpointswithFeatures();
	
	void count3Dkeypoints(vector<corresponding3D2Dpoints_new> points3Dofoneimage, 
					int threshold, 
					float distance);
	/**
	*   if there are more than 1 3D points are coressponded to one training image, these 3D points should been reduced.
	*/
	vector<corresponding3D2Dpoints_new> reduceRedundancy( vector<corresponding3D2Dpoints_new> points3Dofoneimage);
	void project3DkeypointstoEachImage();
	
	void read3DFeaturestoVec(const string inputFilename,
						vector<Point3d> &keypointsCoor,
						vector<Point3d> &normalvec,
						vector<unsigned int> &descriptor);
	virtual ~ featureOperation ();
};

#endif // FEATUREOPERATION_H

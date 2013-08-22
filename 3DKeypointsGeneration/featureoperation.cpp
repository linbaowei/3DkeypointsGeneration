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

#include <iostream>
#include <fstream>
using namespace std;

#include "featureoperation.h"
#include "fileoperation.h"

featureOperation::featureOperation ()
{}

featureOperation::featureOperation (double Angle)
{
	visiableAngle = Angle;
}
featureOperation::featureOperation (double Angle, 
							string plyFileof3DpointCloudFile, 
							string output_folder_now, 
							string trainingImagesFolder,
							string traingImagesPoseFolder)
{
	visiableAngle = Angle;
	plypath = plyFileof3DpointCloudFile;
	output_folder = output_folder_now;
	trainingImagesPath = trainingImagesFolder;
	traingImagesPosePath = traingImagesPoseFolder;
}


featureOperation::~featureOperation ()
{}


mats featureOperation::read3Dpoints (const std::string & plyFilePath)
{
	aPoint* points = NULL;
	aPoint* normal = NULL;
	aPoint* color = NULL;
	int pointnum=0;
	pointnum = readPointnum(points, normal ,plyFilePath.c_str());		//入力する点の総数を読み込む
	color  = new aPoint[pointnum];
	points = new aPoint[pointnum];			//3D座標	
	normal = new aPoint[pointnum];			//法線
	
	readPLY(points, normal, color, plyFilePath.c_str());
	Mat matrix3D=Mat::zeros(4,pointnum,CV_64FC1);
	Mat matrixcolor = Mat::zeros(3, pointnum,CV_64FC1);
	Mat normalvec = Mat::zeros(3, pointnum, CV_64FC1);
	for(int i = 0; i < pointnum; i++)
	{
		matrix3D.at<double>(0,i)=points[i].x;	
		matrix3D.at<double>(1,i)=points[i].y;	
		matrix3D.at<double>(2,i)=points[i].z;	
		matrix3D.at<double>(3,i)=1.0;	
		matrixcolor.at<double>(0,i) = color[i].x;
		matrixcolor.at<double>(1,i) = color[i].y;
		matrixcolor.at<double>(2,i) = color[i].z;
		normalvec.at<double>(0, i) = normal[i].x;
		normalvec.at<double>(1, i) = normal[i].y;
		normalvec.at<double>(2, i) = normal[i].z;
		
		//cout <<"nx:"<< normal[i].x << "ny:"<< normal[i].y << "nz:" << normal[i].z <<endl;
	}
  
	delete [] points;
	delete [] normal;
	delete [] color; 
	
	mats mat;
	mat.matrix3d = matrix3D;
	mat.matrixcolor = matrixcolor;
	mat.normalVec = normalvec;
	return mat;
}


static int vertex_cb(p_ply_argument argument)
{
	void *pdata;
	long indexCoord;	
	ply_get_argument_user_data(argument, &pdata, &indexCoord);	
	aPoint *points = *((aPoint**)pdata);	
	long index;
	ply_get_argument_element(argument, NULL, &index);	
	if (indexCoord == 1)
	{
		points[index].x = ply_get_argument_value(argument);
	}
	if (indexCoord == 2)
	{
		points[index].y = ply_get_argument_value(argument);
	}
	if (indexCoord == 3)
	{
		points[index].z = ply_get_argument_value(argument);
	}
	
	return 1;
}

int featureOperation::readPLY(aPoint* points, 
					aPoint* normal,  
					aPoint* color,
					const char* input_ply)
{
	p_ply ply = ply_open(input_ply, NULL);
	if (!ply) return 1;
	if (!ply_read_header(ply)) return 1;
	
	ply_set_read_cb(ply, "vertex", "x", vertex_cb, &points, 1);
	ply_set_read_cb(ply, "vertex", "y", vertex_cb, &points, 2);
	ply_set_read_cb(ply, "vertex", "z", vertex_cb, &points, 3);
	ply_set_read_cb(ply, "vertex", "nx", vertex_cb, &normal, 1);
	ply_set_read_cb(ply, "vertex", "ny", vertex_cb, &normal, 2);
	ply_set_read_cb(ply, "vertex", "nz", vertex_cb, &normal, 3);
	ply_set_read_cb(ply, "vertex", "diffuse_red", vertex_cb, &color, 1);
	ply_set_read_cb(ply, "vertex", "diffuse_green", vertex_cb, &color, 2);
	ply_set_read_cb(ply, "vertex", "diffuse_blue", vertex_cb, &color, 3);
	
	if (!points || !normal|| !color) {
	cout<<"Error: new aPoint.\n"<<endl;
	if(!points) delete [] points;
	if(!normal) delete [] normal;
	if(!color) delete [] color;
	return 1;
	}
	
	if (!ply_read(ply)) return 1;  // read entire data at once
		ply_close(ply);
	
	return 0;
}

int featureOperation::readPointnum(aPoint* points, 
						aPoint* normal, 
						const char* input_ply)
{
	int pointnum = 0;
	
	p_ply ply = ply_open(input_ply, NULL);
	if (!ply) return 1;
	if (!ply_read_header(ply)) return 1;
	
	long nvertices = 
	ply_set_read_cb(ply, "vertex", "x", vertex_cb, &points, 1);
	pointnum = int(nvertices);
	
	points = new aPoint[pointnum];	
	normal = new aPoint[pointnum];	
	
	ply_close(ply);
	
	return pointnum;
}



double featureOperation::getNormalVecAngle(Mat viewDirection, 
								Mat normalVec)
{
	double angle;
	viewDirection = viewDirection / norm(viewDirection);
	normalVec = normalVec / norm(normalVec);
	angle = normalVec.dot(-viewDirection);
	return angle;
}



/**
 *  return  one projected 2D point after the normal vector calculation
 */
double featureOperation::getOne2DpointsAfterNormalVec(mats matrix,
											Mat projectiomMatrix)
{  
	double returnofDot;
	/****to get intrinsic matrix****/
	Mat matrixk=Mat(3,3,CV_64FC1);	//intrinsic matrix
	Mat R_trainingimg=Mat(3,3,CV_64FC1);  
	Mat T_trainingimg4x1=Mat(4,1,CV_64FC1);
	Mat T_trainingimg3x1=Mat(3,1,CV_64FC1);
	cv::decomposeProjectionMatrix(projectiomMatrix,
						matrixk,
						R_trainingimg,
						T_trainingimg4x1);   
	T_trainingimg3x1=matrixk.inv()*projectiomMatrix.col(3);
	
	/***translation vector***/
	Mat Ocw = -R_trainingimg.t() * T_trainingimg3x1;
	
	/***projected 2D points***/
	vector<Point3d> vec2D;
	if(matrix.matrix3d.cols>1)
	{
		cout << "ERROR - more than one points are passed to getOne2DpointsAfterNormalVec()." << endl;
		exit(1);
	}
	for(unsigned int i=0; i<matrix.matrix3d.cols; i++)
	{
		Mat onecol = Mat(3, 1, CV_64FC1);
		onecol.at<double>(0, 0) = matrix.normalVec.at<double>(0, i);
		onecol.at<double>(1, 0) = matrix.normalVec.at<double>(1, i);
		onecol.at<double>(2, 0) = matrix.normalVec.at<double>(2, i);
		Mat viewDirection = R_trainingimg.row(2).t();    
		
		/***return the cos() value of angle which between view direction and normal vector***/
		returnofDot = getNormalVecAngle(viewDirection,
							onecol);
	}
	return returnofDot;
}


/**
 *  return projected 2D  points from 3D points after the normal vector calculation
 */
vector<correspondingFrom3dto2d> featureOperation::get2DpointsAfterNormalVec(mats matrix,
															Mat projectiomMatrix,
															string image)
{  
	/****to get intrinsic matrix****/
	Mat matrixk=Mat(3,3,CV_64FC1);	//intrinsic matrix
	Mat R_trainingimg=Mat(3,3,CV_64FC1);  
	Mat T_trainingimg4x1=Mat(4,1,CV_64FC1);
	Mat T_trainingimg3x1=Mat(3,1,CV_64FC1);
	cv::decomposeProjectionMatrix(projectiomMatrix,
						matrixk,
						R_trainingimg,
						T_trainingimg4x1);   	
	
	/***projected 2D points***/
	vector<Point3d> vec2D;
	vector<Point3d> vec3D;
	vector<correspondingFrom3dto2d> points;
	
	Mat imageForShow = imread(image);
	for(unsigned int i=0; i<matrix.matrix3d.cols; i++)
	{	
		Mat onecol = Mat(3, 1, CV_64FC1);
		onecol.at<double>(0, 0) = matrix.normalVec.at<double>(0, i);
		onecol.at<double>(1, 0) = matrix.normalVec.at<double>(1, i);
		onecol.at<double>(2, 0) = matrix.normalVec.at<double>(2, i);
		Mat viewDirection = R_trainingimg.row(2).t();    
		/***return the cos() value of angle which between view direction and normal vector***/
		double returnofDot = getNormalVecAngle(viewDirection,
							onecol);
		//cout <<  acos(returnofDot)*180/3.1415926 << endl;
		if(returnofDot > cos(visiableAngle*3.1415/180))
		{		
			correspondingFrom3dto2d point;
			point.point2d.x = Mat(projectiomMatrix * matrix.matrix3d.col(i)).at<double>(0, 0)/Mat(projectiomMatrix * matrix.matrix3d.col(i)).at<double>(2, 0);
			point.point2d.y = Mat(projectiomMatrix * matrix.matrix3d.col(i)).at<double>(1, 0)/Mat(projectiomMatrix * matrix.matrix3d.col(i)).at<double>(2, 0);
			point.point2d.z = Mat(projectiomMatrix * matrix.matrix3d.col(i)).at<double>(2, 0)/Mat(projectiomMatrix * matrix.matrix3d.col(i)).at<double>(2, 0);
			
			point.point3d.x = matrix.matrix3d.at<double>(0, i);
			point.point3d.y = matrix.matrix3d.at<double>(1, i);
			point.point3d.z = matrix.matrix3d.at<double>(2, i);
			
			point.normalVec = Mat(3,1,CV_64FC1);
			point.normalVec.at<double>(0, 0) = matrix.normalVec.at<double>(0, i);
			point.normalVec.at<double>(1, 0) = matrix.normalVec.at<double>(1, i);
			point.normalVec.at<double>(2, 0) = matrix.normalVec.at<double>(2, i);
			
			if(point.point2d.x>0 
				&& point.point2d.y>0
				&& point.point2d.y < imageForShow.cols 
				&& point.point2d.y < imageForShow.rows)
			{
				points.push_back(point);
			}      
		}
	}	
	return points;
}





vector<corresponding3D2Dpoints_new> featureOperation::read_siftFeatures_new(string siftPath)
{
	vector<corresponding3D2Dpoints_new> vecsift;
	corresponding3D2Dpoints_new sift_new;
	ifstream standpoing(siftPath.c_str());
	if (standpoing.is_open())
	{
		string line;
		int flag = 0;
		while ( standpoing.good() )
		{	
			getline (standpoing,line);     
			string buf; // Have a buffer string
			stringstream ss(line); // Insert the string into a stream
			vector<string> num; // Create vector to hold our words
			while (ss >> buf)
				num.push_back(buf);
			if(num.size()==8&&flag==0)
			{
				sift_new.point3D.x = atof(num.at(0).c_str());
				sift_new.point3D.y= atof(num.at(1).c_str());
				sift_new.point3D.z= atof(num.at(2).c_str());
				sift_new.normalVec = Mat(3, 1, CV_64FC1);
				sift_new.normalVec.at<double>(0, 0) = atof(num.at(3).c_str());
				sift_new.normalVec.at<double>(1, 0) = atof(num.at(4).c_str());
				sift_new.normalVec.at<double>(2, 0) = atof(num.at(5).c_str());
				sift_new.sacle = atof(num.at(6).c_str());
				sift_new.orientation = atof(num.at(7).c_str());
				flag ++;
			}
			if(num.size()==20)
			{
				if(flag==1)
				{
					for(int i = 0; i < 20; i++)
					{
						sift_new.features1.push_back(atoi(num.at(i).c_str()));
						//cout << atoi(num.at(i).c_str()) <<endl;
					}
					flag ++;
				}
				else if(flag==2)
				{
					for(int i = 0; i < 20; i++)
					{
						sift_new.features2.push_back(atoi(num.at(i).c_str()));	
						//cout << atoi(num.at(i).c_str()) <<endl;
					}
					flag ++;
				}
				else  if(flag==3)
				{
					for(int i = 0; i < 20; i++)
					{
						sift_new.features3.push_back(atoi(num.at(i).c_str()));	
						//cout << atoi(num.at(i).c_str()) <<endl;
					}
					flag ++;
				}
				else  if(flag==4)
				{
					for(int i = 0; i < 20; i++)
					{
						sift_new.features4.push_back(atoi(num.at(i).c_str()));
						//cout << atoi(num.at(i).c_str()) <<endl;
					}
					flag ++;
				}
				else if(flag==5)
				{
					for(int i = 0; i < 20; i++)
					{
						sift_new.features5.push_back(atoi(num.at(i).c_str()));		
						//cout << atoi(num.at(i).c_str()) <<endl;
					}
					flag ++;
				}
				else if(flag==6)
				{
					for(int i = 0; i < 20; i++)
					{
						sift_new.features6.push_back(atoi(num.at(i).c_str()));	
						//cout << atoi(num.at(i).c_str()) <<endl;
					}
					flag ++;
				}
			}
			if(num.size()==8&&flag==7)
			{
				for(int i = 0; i < 8; i++)
				{
					sift_new.features7.push_back(atoi(num.at(i).c_str()));	
					//cout << atoi(num.at(i).c_str()) <<endl;
				}
				flag ++;
			}
			if(flag==8)
			{				
				vecsift.push_back(sift_new);
				//cout << vecsift.size()<< endl;
				//for(int n =0; n< vecsift.at(0).features1.size(); n++)
				//cout << vecsift.at(0).features1.at(n) << " ";
				sift_new.features1.clear();
				sift_new.features2.clear();
				sift_new.features3.clear();
				sift_new.features4.clear();
				sift_new.features5.clear();
				sift_new.features6.clear();
				sift_new.features7.clear();
				flag = 0;				
			}
			//cout << "flag:"<<flag <<endl;
		}
		standpoing.close();
	}    
	else cout << "Unable to open file";  
	return vecsift;
}

vector<sift_feature> featureOperation::read_siftFeatures(string siftPath)
{
	vector<sift_feature> vecsift;
	sift_feature sift;
	ifstream standpoing(siftPath.c_str());
	if (standpoing.is_open())
	{
		string line;
		int flag = 0;
		while ( standpoing.good() )
		{		      
			getline (standpoing,line);     
			string buf; // Have a buffer string
			stringstream ss(line); // Insert the string into a stream
			vector<string> num; // Create vector to hold our words
			while (ss >> buf)
				num.push_back(buf);
			/*for(int m = 0; m < num.size(); m++)
			*		cout << num.at(m)<< " ";*/
			if(num.size() == 0)
				break;
			if(num.size()==2)
				continue;
			if(num.size()==4&&flag==0)
			{
				sift.point2d = Mat(2, 1, CV_64FC1);
				sift.point2d.at<double>(0, 0) = atof(num.at(0).c_str());
				sift.point2d.at<double>(1, 0) = atof(num.at(1).c_str());
				sift.sacle = atof(num.at(2).c_str());
				sift.orientation = atof(num.at(3).c_str());
				flag ++;
			}
			if(num.size()==20)
			{
				if(flag==1)
				{
					for(int i = 0; i < 20; i++)
					{
						sift.features1.push_back(atoi(num.at(i).c_str()));					
					}
					flag ++;
				}
				else if(flag==2)
				{
					for(int i = 0; i < 20; i++)
					{
						sift.features2.push_back(atoi(num.at(i).c_str()));		
					}
					flag ++;
				}
				else  if(flag==3)
				{
					for(int i = 0; i < 20; i++)
					{
						sift.features3.push_back(atoi(num.at(i).c_str()));					
					}
					flag ++;
				}
				else  if(flag==4)
				{
					for(int i = 0; i < 20; i++)
					{
						sift.features4.push_back(atoi(num.at(i).c_str()));					
					}
					flag ++;
				}
				else if(flag==5)
				{
					for(int i = 0; i < 20; i++)
					{
						sift.features5.push_back(atoi(num.at(i).c_str()));					
					}
					flag ++;
				}
				else if(flag==6)
				{
					for(int i = 0; i < 20; i++)
					{
						sift.features6.push_back(atoi(num.at(i).c_str()));					
					}
					flag ++;
				}
			}
			if(num.size()==8&&flag==7){
				for(int i = 0; i < 8; i++)
				{
					sift.features7.push_back(atoi(num.at(i).c_str()));					
				}
				flag ++;
			}
			if(flag == 8)
			{				
				vecsift.push_back(sift);
				//cout << vecsift.size()<< endl;
				//for(int n =0; n< vecsift.at(0).features1.size(); n++)
				//cout << vecsift.at(0).features1.at(n) << " ";
				sift.features1.clear();
				sift.features2.clear();
				sift.features3.clear();
				sift.features4.clear();
				sift.features5.clear();
				sift.features6.clear();
				sift.features7.clear();
				flag = 0;				
			}
		}
		standpoing.close();
	}    
	else cout << "Unable to open file";  
	return vecsift;
}


corresponding3D2Dpoints_new featureOperation::average (const corresponding3D2Dpoints_new & arg1,
										const corresponding3D2Dpoints_new   & arg2)
{
  
	corresponding3D2Dpoints_new average;
	average.point3D.x = (arg1.point3D.x + arg2.point3D.x)/2;
	average.point3D.y = (arg1.point3D.y + arg2.point3D.y)/2;
	average.point3D.z = (arg1.point3D.z + arg2.point3D.z)/2;
	
	average.normalVec = Mat(3, 1, CV_64FC1);
	average.normalVec.at<double>(0, 0) = (arg1.normalVec.at<double>(0, 0) + arg2.normalVec.at<double>(0, 0))/2;
	average.normalVec.at<double>(1, 0) = (arg1.normalVec.at<double>(1, 0) + arg2.normalVec.at<double>(1, 0))/2;
	average.normalVec.at<double>(2, 0) = (arg1.normalVec.at<double>(2, 0) + arg2.normalVec.at<double>(2, 0))/2;
	
	average.sacle = (arg1.sacle + arg2.sacle)/2;
	
	average.orientation = (arg1.orientation + arg2.orientation)/2;
	
	for(int i = 0; i < arg1.features1.size(); i++)
	{
		average.features1.push_back((arg1.features1.at(i) + arg2.features1.at(i))/2);
	}
	for(int i = 0; i < arg1.features2.size(); i++)
	{
		average.features2.push_back((arg1.features2.at(i) + arg2.features2.at(i))/2);
	}
	for(int i = 0; i < arg1.features3.size(); i++)
	{
		average.features3.push_back((arg1.features3.at(i) + arg2.features3.at(i))/2);
	}
	for(int i = 0; i < arg1.features4.size(); i++)
	{
		average.features4.push_back((arg1.features4.at(i) + arg2.features4.at(i))/2);
	}
	for(int i = 0; i < arg1.features5.size(); i++)
	{
		average.features5.push_back((arg1.features5.at(i) + arg2.features5.at(i))/2);
	}
	for(int i = 0; i < arg1.features6.size(); i++)
	{
		average.features6.push_back((arg1.features6.at(i) + arg2.features6.at(i))/2);
	}
	for(int i = 0; i < arg1.features7.size(); i++)
	{
		average.features7.push_back((arg1.features7.at(i) + arg2.features7.at(i))/2);
	}
	
	return average;
}




/**
 *   to generate 3D point cloud file with features.  the points are calculated from the training images. 
 *   if one 3d point could be projected into threshhold images, that is to say, this 3d point is avilable. 
 */
void featureOperation::generate3DpointswithFeatures()
{
	fileOperation file;
	string traingimageProjectionMatrixPath = traingImagesPosePath;
	string visualize = trainingImagesPath;
	
	mats matrix3D=read3Dpoints(plypath);
	vector<string> cameraposefiles = file.getFilenames(traingimageProjectionMatrixPath, "*.txt");
	vector<Mat> matric =file.getCameraPose(traingimageProjectionMatrixPath, cameraposefiles, 1);
	vector<string> imagekeyfiles = file.getFilenames(visualize, "*.key");
	
	vector< vector<corresponding3D2Dpoints_new> > allpoints;
	vector< vector<corresponding3D2Dpoints_new> > allpointstmp;	
	
	vector<float> dist;
	dist.push_back(5.0f);
	dist.push_back(3.0f);
	dist.push_back(1.0f);
	dist.push_back(0.7f);
	dist.push_back(0.5f);
	
	vector< vector<corresponding3D2Dpoints_new> >points3Dofoneimages(dist.size());
  
	for(int j = 0; j < matric.size(); j++)
	{
		for(int p = 0; p < imagekeyfiles.size(); p++)
		{
			string aaa = cameraposefiles.at(j).substr(0, cameraposefiles.at(j).length() - 4);
			string bbb = imagekeyfiles.at(p).substr(0, imagekeyfiles.at(p).length()-4);
			if(abs(atoi(aaa.c_str()) - atoi(bbb.c_str()))<1e-8)
			{	
				cout << aaa << endl;
				cout << bbb << endl;
				string image = visualize + cameraposefiles.at(j).substr(0, cameraposefiles.at(j).length() - 4) + ".jpg";
				vector<correspondingFrom3dto2d> points  = get2DpointsAfterNormalVec(matrix3D, matric.at(j), image);
				
				cout << image << endl;
				//Mat imageForShow = imread(image);
				string imagekey = visualize + imagekeyfiles.at(p);
				vector<sift_feature> imagefeatures = read_siftFeatures(imagekey);	
				vector < vector<corresponding3D2Dpoints_new> >points3Dofoneimageori(dist.size());
				//cout<< imageForShow.cols<<"\t"<<imageForShow.rows<<endl;
				cout << points.size()<< endl; 
				for(int d = 0; d < dist.size(); d ++)
				{
					for(int i = 0; i < points.size() ; i++)
					{
						int count = 0;	  //for each projected 2D point, how many points on images are corresponded to the projected 2D point
						for(int a = 0; a < imagefeatures.size(); a++)
						{	    
							if((points.at(i).point2d.x-imagefeatures.at(a).point2d.at<double>(1, 0))*
								(points.at(i).point2d.x-imagefeatures.at(a).point2d.at<double>(1, 0)) < dist.at(d)&&
								(points.at(i).point2d.y-imagefeatures.at(a).point2d.at<double>(0, 0))*
								(points.at(i).point2d.y-imagefeatures.at(a).point2d.at<double>(0, 0)) < dist.at(d))
							{
								count++;
								if(count >1)
								{
									if(i<points.size()-1)
									{
										i++;
										continue;		      
									}
									else
										continue;
								}
								corresponding3D2Dpoints_new singlecoord;
								singlecoord.point2d = Mat(3,1,CV_64FC1);
								singlecoord.point2d.at<double>(0, 0) = points.at(i).point2d.x;
								singlecoord.point2d.at<double>(1, 0) = points.at(i).point2d.y;
								singlecoord.point2d.at<double>(2, 0) = points.at(i).point2d.z;
								
								singlecoord.point3D = points.at(i).point3d;
								singlecoord.normalVec = points.at(i).normalVec;
								singlecoord.sacle = imagefeatures.at(a).sacle;
								singlecoord.orientation = imagefeatures.at(a).orientation;
								singlecoord.features1 = imagefeatures.at(a).features1;
								singlecoord.features2 = imagefeatures.at(a).features2;
								singlecoord.features3 = imagefeatures.at(a).features3;
								singlecoord.features4 = imagefeatures.at(a).features4;
								singlecoord.features5 = imagefeatures.at(a).features5;
								singlecoord.features6 = imagefeatures.at(a).features6;
								singlecoord.features7 = imagefeatures.at(a).features7;
								points3Dofoneimageori.at(d).push_back(singlecoord);
							}
						}
					}
				}
				
				for(int d = 0;  d < dist.size(); d ++)
				{
					cout  << "3D keypoints number is " <<  points3Dofoneimageori.at(d).size() <<" when pixels are " << dist.at(d); 
					for(int g = 0; g < points3Dofoneimageori.at(d).size(); g++)
					{	      
						points3Dofoneimages.at(d).push_back(points3Dofoneimageori.at(d).at(g));	      
					}	    
					cout  << "        till now, total 3D keypoints number is " << points3Dofoneimages.at(d).size()  <<" when pixels are " << dist.at(d) << endl;
				}
			}
		}
	}
	
	for(int i = 0; i < matric.size(); i++)
	{
		for(int d = 0; d < dist.size(); d++)
		{
			count3Dkeypoints(points3Dofoneimages.at(d), i , dist.at(d));
		}
	}
}


void featureOperation::count3Dkeypoints(vector<corresponding3D2Dpoints_new> points3Dofoneimage, int threshold, float distance)
{
	vector<corresponding3D2Dpoints_new> points3Dofoneimagetmp;
	points3Dofoneimagetmp = points3Dofoneimage;
	vector<corresponding3D2Dpoints_new> fianl3Dpoints;  
	for(int i=0; i < points3Dofoneimage.size(); i++)
	{
		int count =0;
		for(int h = 0; h < points3Dofoneimagetmp.size(); h++)
		{
			if(i==h)
				continue;
			if(points3Dofoneimage.at(i).point3D.x == points3Dofoneimagetmp.at(h).point3D.x
				&&points3Dofoneimage.at(i).point3D.y == points3Dofoneimagetmp.at(h).point3D.y
				&&points3Dofoneimage.at(i).point3D.z == points3Dofoneimagetmp.at(h).point3D.z
				&&points3Dofoneimage.at(i).point3D.x != 1000
				&&points3Dofoneimage.at(i).point3D.y != 1000
				&&points3Dofoneimage.at(i).point3D.z != 1000)
			{
				points3Dofoneimage.at(i) = average(points3Dofoneimage.at(i), points3Dofoneimagetmp.at(h));
				points3Dofoneimage.at(h).point3D.x = 1000;
				points3Dofoneimage.at(h).point3D.y = 1000;
				points3Dofoneimage.at(h).point3D.z = 1000;
				points3Dofoneimagetmp.at(h).point3D.x = 1000;
				points3Dofoneimagetmp.at(h).point3D.y = 1000;
				points3Dofoneimagetmp.at(h).point3D.z = 1000;
				count ++;
			}
		}
    
		if(count > threshold)
			fianl3Dpoints.push_back(points3Dofoneimage.at(i));        
	}
  
	cout << fianl3Dpoints.size() <<" 3D keypoints are in more than " << threshold << " images. pixel distance is " << distance << endl;
	
	Mat matrix3dpoints = Mat(4, fianl3Dpoints.size(), CV_64FC1);
	ofstream outfile (string("output_"+IntToString(threshold)+"_"+FloatToString(distance)).c_str());
	if(!outfile) 
	{
		cout << "!outfile"<< endl;
		exit(1);
	}
	
	for(int i = 0; i< fianl3Dpoints.size(); i++)
	{
		outfile <<  DoubleToString(fianl3Dpoints.at(i).point3D.x) 
		+ " " + DoubleToString(fianl3Dpoints.at(i).point3D.y) 
		+ " " + DoubleToString(fianl3Dpoints.at(i).point3D.z)
		+ " " + DoubleToString(fianl3Dpoints.at(i).normalVec.at<double>(0, 0))
		+ " " + DoubleToString(fianl3Dpoints.at(i).normalVec.at<double>(1, 0))
		+ " " + DoubleToString(fianl3Dpoints.at(i).normalVec.at<double>(2, 0))
		+ " " + DoubleToString(fianl3Dpoints.at(i).sacle)
		+ " " + DoubleToString(fianl3Dpoints.at(i).orientation) << endl;
		matrix3dpoints.at<double>(0, i) = fianl3Dpoints.at(i).point3D.x;
		matrix3dpoints.at<double>(1, i) = fianl3Dpoints.at(i).point3D.y;
		matrix3dpoints.at<double>(2, i) = fianl3Dpoints.at(i).point3D.z;
		matrix3dpoints.at<double>(3, i) = 1;
	for(int j =0; j< fianl3Dpoints.at(i).features1.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features1.at(j))+" ";
	}
		outfile << endl;
	for(int j =0; j< fianl3Dpoints.at(i).features2.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features2.at(j))+" ";
	}
		outfile << endl;
	for(int j =0; j< fianl3Dpoints.at(i).features3.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features3.at(j))+" ";
	}
		outfile << endl;
	for(int j =0; j< fianl3Dpoints.at(i).features4.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features4.at(j))+" ";
	}
		outfile << endl;
	for(int j =0; j< fianl3Dpoints.at(i).features5.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features5.at(j))+" ";
	}
		outfile << endl;
	for(int j =0; j< fianl3Dpoints.at(i).features6.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features6.at(j))+" ";
	}
		outfile << endl;
	for(int j =0; j< fianl3Dpoints.at(i).features7.size(); j++)
	{
		outfile << DoubleToString(fianl3Dpoints.at(i).features7.at(j))+" ";
	}
		outfile << endl;    
	}
		outfile.close();
  
	fileOperation file;
	string traingimageProjectionMatrixPath = traingImagesPosePath;
	string visualize = trainingImagesPath;
	
	mats matrix3D=read3Dpoints(plypath);
	
	vector<string> cameraposefiles = file.getFilenames(traingimageProjectionMatrixPath, "*.txt");
	vector<Mat> matric =file.getCameraPose(traingimageProjectionMatrixPath, cameraposefiles, 1);
	
	
	///////////////////////////////////////
	//project 3D keypoints on each image.
	/*for(int i =0; i< matric.size(); i++)
	{
	if(distance == 5)
		projection_normal(matrix3dpoints, matric.at(i),  visualize + cameraposefiles.at(i).substr(0, cameraposefiles.at(i).length() - 4) + ".jpg", i);
	}*/	
}

void featureOperation::project3DkeypointstoEachImage()
{
	fileOperation file;
	string traingimageProjectionMatrixPath = traingImagesPosePath;
	string visualize = trainingImagesPath;
	
	mats matrix3D;
	
	vector<string> cameraposefiles = file.getFilenames(traingimageProjectionMatrixPath, "*.txt");
	vector<Mat> matric =file.getCameraPose(traingimageProjectionMatrixPath, cameraposefiles, 1);
	
	vector<Mat> matric2 = matric;
	vector<float> dist;
	dist.push_back(5.0f);
	dist.push_back(3.0f);
	dist.push_back(1.0f);
	dist.push_back(0.7f);
	dist.push_back(0.5f);
	// for(int j =0; j < dist.size(); j ++)
	for(int j =4; j <5; j ++)
	{
		//for(int i =0; i< matric.size(); i++)
		for(int i =1; i<7; i++)
		{   
			vector<Point3d> keypoints;
			vector<unsigned int> descriptors;
			vector<Point3d> normalvec;
			string allInOne3Dpoints = "output_"+IntToString(i)+"_"+FloatToString(dist.at(j));
			read3DFeaturestoVec(allInOne3Dpoints, 
						keypoints,
						normalvec,
						descriptors);
			matrix3D.matrix3d= Mat(4, keypoints.size(), CV_64FC1);
			matrix3D.normalVec=Mat(3, keypoints.size(), CV_64FC1);
			for(int a = 0; a < keypoints.size(); a++)
			{
				matrix3D.matrix3d.at<double>(0,a) = keypoints.at(a).x;
				matrix3D.matrix3d.at<double>(1,a) = keypoints.at(a).y;
				matrix3D.matrix3d.at<double>(2,a) = keypoints.at(a).z;
				matrix3D.matrix3d.at<double>(3,a) = 1;
				matrix3D.normalVec.at<double>(0,a) = normalvec.at(a).x;
				matrix3D.normalVec.at<double>(1,a) = normalvec.at(a).y;
				matrix3D.normalVec.at<double>(2,a) = normalvec.at(a).z;	    
			}
			cout << allInOne3Dpoints << endl;
			for(int b = 0; b < matric2.size(); b++)
			{
				vector<correspondingFrom3dto2d> points  = get2DpointsAfterNormalVec(matrix3D, matric2.at(b), "result0.jpg");
				cout << points.size()<< endl;
			}
			cout << endl;
		}
		cout << endl;
	}  
}


void featureOperation::read3DFeaturestoVec(const string inputFilename,
								vector<Point3d> &keypointsCoor,
								vector<Point3d> &normalvec,
								vector<unsigned int> &descriptor)
{
	ifstream ifs ( inputFilename.c_str() );
	if (!ifs)
	{
	cerr << "Cannot open " << inputFilename << endl;
	exit(1);
	}

	for (int j = 0; j <0; j++)
	{
		double num;
		ifs >> num; // just skip
	}

	vector < vector<int> > trainingDataVector;
	Point3d point,norm;
	while (ifs.good()) 
	{
		for (int j = 0; j < 8; j++) 
		{
			double num;
			ifs >> num; // just skip
			if(j == 0)
				point.x = num;
			if(j == 1)
				point.y = num;
			if(j == 2)
			{
				point.z = num;	
			}
			if(j == 3)
				norm.x = num;
			if(j == 4)
				norm.y = num;
			if(j == 5)
			{
				norm.z = num;	
			}		
		}
		if (!ifs.good()) break;

		vector<int> feature;
		for(unsigned int j = 0; j < 128; j++) 
		{
			int k;
			ifs >> k;
			if (!ifs.good()) 
			{
				if (j == 0) break;
					cerr << "Error while reading " << inputFilename << endl;
					exit(1);
			}
			feature.push_back(k);
		} // for j

		if (!ifs.good()) break;
		keypointsCoor.push_back(point);
		normalvec.push_back(norm);
		trainingDataVector.push_back(feature);

	} // end of while


	//
	// copy all features to the argument
	//
	//descriptor = Mat(trainingDataVector.size(), 128, CV_64FC1);
	try {
		for(int i =0; i<trainingDataVector.size(); i++)
		{
			for(int j = 0; j < 128; j ++)
			{
				descriptor.push_back( trainingDataVector.at(i).at(j));
			}
		}
	} catch (bad_alloc e) {
		cerr << "memory allocate error" << endl;
		exit(1);
	}	
	ifs.close();
}


/**
 *   to generate 3D point cloud file with features.  the points are calculated from the training images. 
 *   if one 3d point could be projected into threshhold images, that is to say, this 3d point is avilable. 
 *   the features values of this point is the average of  projected points of threshhold images when 
 *   FLAG is 0,  otherwise, assert all features values to the point.
 *   threshhold is the thresh training images.
 *   (not in use)P_wnum is the 3D points num in 3D point cloud to generate P_w.
 */
// void featureOperation::generate3DpointswithFeatures(int FLAG, int threshhold,double distance)
// {
//   fileOperation file;
//   string traingimageProjectionMatrixPath = traingImagesPosePath;
//   string visualize = trainingImagesPath;
//   
//   mats matrix3D=read3Dpoints(plypath);
//   
//   //Mat P_w  =  matrix3D.matrix3d.col(P_wnum).rowRange(0,3);
//   
//   vector<string> cameraposefiles = file.getFilenames(traingimageProjectionMatrixPath, "*.txt");
//   vector<Mat> matric =file.getCameraPose(traingimageProjectionMatrixPath, cameraposefiles, 1);
//   //cout << matric.size() << endl;
//   vector<string> imagekeyfiles = file.getFilenames(visualize, "*.key");
//   
//   
//   vector< vector<corresponding3D2Dpoints_new> > allpoints;
//   vector< vector<corresponding3D2Dpoints_new> > allpointstmp;
//   vector<corresponding3D2Dpoints_new> points3Dofoneimage;
//   vector<corresponding3D2Dpoints_new> points3Dofoneimagetmp;
//   
//   static int removed2Dnum = 0;
//   static int removed3Dnum = 0;
//   static int totalpointsnum = 0;
//   for(int j = 0; j < matric.size(); j++)
//   {
//     for(int p = 0; p < imagekeyfiles.size(); p++)
//     {
//       if(cameraposefiles.at(j).substr(0, cameraposefiles.at(j).length() - 4) == imagekeyfiles.at(p).substr(0, imagekeyfiles.at(p).length()-4))
//       {
// 	
// 	string image = visualize + cameraposefiles.at(j).substr(0, cameraposefiles.at(j).length() - 4) + ".jpg";
// 	vector<correspondingFrom3dto2d> points  = get2DpointsAfterNormalVec(matrix3D, matric.at(j), image);
// 	vector<corresponding3D2Dpoints_new> points3Dofoneimageori;
// 	cout << image << endl;
// 	//Mat imageForShow = imread(image);
// 	string imagekey = visualize + imagekeyfiles.at(p);
// 	vector<sift_feature> imagefeatures = read_siftFeatures(imagekey);
// 	//cout<< imageForShow.cols<<"\t"<<imageForShow.rows<<endl;
// 	for(int i = 0; i < points.size() ; i++)
// 	{
// 	  for(int a = 0; a < imagefeatures.size(); a++)
// 	  {
// 	    if((points.at(i).point2d.x-imagefeatures.at(a).point2d.at<double>(1, 0))*(points.at(i).point2d.x-imagefeatures.at(a).point2d.at<double>(1, 0)) < distance&&
// 	      (points.at(i).point2d.y-imagefeatures.at(a).point2d.at<double>(0, 0))*(points.at(i).point2d.y-imagefeatures.at(a).point2d.at<double>(0, 0)) < distance)
// 	    {
// 	      corresponding3D2Dpoints_new singlecoord;
// 	      singlecoord.point2d = Mat(3,1,CV_64FC1);
// 	      singlecoord.point2d.at<double>(0, 0) = points.at(i).point2d.x;
// 	      singlecoord.point2d.at<double>(1, 0) = points.at(i).point2d.y;
// 	      singlecoord.point2d.at<double>(2, 0) = points.at(i).point2d.z;
// 	      
// 	      singlecoord.point3D = points.at(i).point3d;
// 	      singlecoord.normalVec = points.at(i).normalVec;
// 	      singlecoord.sacle = imagefeatures.at(a).sacle;
// 	      singlecoord.orientation = imagefeatures.at(a).orientation;
// 	      singlecoord.features1 = imagefeatures.at(a).features1;
// 	      singlecoord.features2 = imagefeatures.at(a).features2;
// 	      singlecoord.features3 = imagefeatures.at(a).features3;
// 	      singlecoord.features4 = imagefeatures.at(a).features4;
// 	      singlecoord.features5 = imagefeatures.at(a).features5;
// 	      singlecoord.features6 = imagefeatures.at(a).features6;
// 	      singlecoord.features7 = imagefeatures.at(a).features7;
// 	      points3Dofoneimageori.push_back(singlecoord);
// 	    }
// 	  }
// 	}
// 	
// 	vector<corresponding3D2Dpoints_new> tmpvec = reduceRedundancy(points3Dofoneimageori); 
// 	for(int g = 0; g < tmpvec.size(); g++)
// 	{
// 	  points3Dofoneimage.push_back(tmpvec.at(g));
// 	}
// 	
// 	points3Dofoneimagetmp = points3Dofoneimage;
// 	// 	removed2Dnum += points3Dofoneimageori.size();
// 	// 	removed3Dnum += tmpvec.size();
// 	// 	totalpointsnum += points.size();
// 	// 	cout << "totalpointsnum: " << totalpointsnum << " " <<points.size() << endl;
// 	// 	cout << "removed multi-corresponded 2D keypoints number: " << removed2Dnum << endl;
// 	//	cout << "removed multi-corresponded 3D keypoints number: " << removed3Dnum << endl;
// 	cout << points3Dofoneimageori.size() << endl;
// 	cout << "totol 3D keypoints number when threshold is 0: " <<points3Dofoneimage.size()<< endl;
// 	//cout << points3Dofoneimagetmp.size()<< endl;
//       }
//     }
//   }
//   
//   
//   
//   ofstream all3Dpoints (string( "allInOne3DpointsFilebeforeThreshing.txt").c_str());
//   if(!all3Dpoints) 
//   {
//     cout << "!all3Dpoints"<< endl;
//     exit(1);
//   }
//   for(int i = 0; i< points3Dofoneimage.size(); i++)
//   {
//     all3Dpoints <<  DoubleToString(points3Dofoneimage.at(i).point3D.x) 
//     + " " + DoubleToString(points3Dofoneimage.at(i).point3D.y) 
//     + " " + DoubleToString(points3Dofoneimage.at(i).point3D.z)
//     + " " + DoubleToString(points3Dofoneimage.at(i).normalVec.at<double>(0, 0))
//     + " " + DoubleToString(points3Dofoneimage.at(i).normalVec.at<double>(1, 0))
//     + " " + DoubleToString(points3Dofoneimage.at(i).normalVec.at<double>(2, 0))
//     + " " + DoubleToString(points3Dofoneimage.at(i).sacle)
//     + " " + DoubleToString(points3Dofoneimage.at(i).orientation) << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features1.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features1.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features2.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features2.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features3.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features3.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features4.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features4.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features5.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features5.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features6.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features6.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     for(int j =0; j< points3Dofoneimage.at(i).features7.size(); j++)
//     {
//       all3Dpoints << DoubleToString(points3Dofoneimage.at(i).features7.at(j))+" ";
//     }
//     all3Dpoints << endl;
//     
//   }
//   all3Dpoints.close();
//   
//   
//   
//   
//   
//   
//   vector<corresponding3D2Dpoints_new> fianl3Dpoints;
//   
//   
//   if(FLAG == 0)
//     //do average of some 2D points which related to one 3D point
//   {
//     for(int i=0; i < points3Dofoneimage.size(); i++)
//     {
//       int count =0;
//       for(int h = 0; h < points3Dofoneimagetmp.size(); h++)
//       {
// 	if(i==h)
// 	  continue;
// 	if(points3Dofoneimage.at(i).point3D.x == points3Dofoneimagetmp.at(h).point3D.x
// 	  &&points3Dofoneimage.at(i).point3D.y == points3Dofoneimagetmp.at(h).point3D.y
// 	  &&points3Dofoneimage.at(i).point3D.z == points3Dofoneimagetmp.at(h).point3D.z
// 	  &&points3Dofoneimage.at(i).point3D.x != 1000
// 	  &&points3Dofoneimage.at(i).point3D.y != 1000
// 	  &&points3Dofoneimage.at(i).point3D.z != 1000)
// 	{
// 	  points3Dofoneimage.at(i) = average(points3Dofoneimage.at(i), points3Dofoneimagetmp.at(h));
// 	  points3Dofoneimage.at(h).point3D.x = 1000;
// 	  points3Dofoneimage.at(h).point3D.y = 1000;
// 	  points3Dofoneimage.at(h).point3D.z = 1000;
// 	  points3Dofoneimagetmp.at(h).point3D.x = 1000;
// 	  points3Dofoneimagetmp.at(h).point3D.y = 1000;
// 	  points3Dofoneimagetmp.at(h).point3D.z = 1000;
// 	  count ++;
// 	}
//       }
//       
//       if(count > threshhold)
// 	fianl3Dpoints.push_back(points3Dofoneimage.at(i));        
//     }
//   }
//   
//   else
//   {
//     for(int i=0; i < points3Dofoneimage.size(); i++)
//     {
//       int count =0;
//       for(int h = 0; h < points3Dofoneimagetmp.size(); h++)
//       {
// 	if(i==h)
// 	  continue;
// 	if(points3Dofoneimage.at(i).point3D.x == points3Dofoneimagetmp.at(h).point3D.x
// 	  &&points3Dofoneimage.at(i).point3D.y == points3Dofoneimagetmp.at(h).point3D.y
// 	  &&points3Dofoneimage.at(i).point3D.z == points3Dofoneimagetmp.at(h).point3D.z
// 	)
// 	{
// 	  count ++;
// 	}
//       }
//       
//       if(count > threshhold)
// 	fianl3Dpoints.push_back(points3Dofoneimage.at(i));        
//     }
//   }
//   cout  << fianl3Dpoints.size() << " points are 3D points in  "<< threshhold << " or more images." << endl;   
//   
//   
//   Mat matrix3dpoints = Mat(4, fianl3Dpoints.size(), CV_64FC1);
//   ofstream outfile (allInOne3DpointsFile.c_str());
//   if(!outfile) 
//   {
//     cout << "!outfile"<< endl;
//     exit(1);
//   }
//   for(int i = 0; i< fianl3Dpoints.size(); i++)
//   {
//     outfile <<  DoubleToString(fianl3Dpoints.at(i).point3D.x) 
//     + " " + DoubleToString(fianl3Dpoints.at(i).point3D.y) 
//     + " " + DoubleToString(fianl3Dpoints.at(i).point3D.z)
//     + " " + DoubleToString(fianl3Dpoints.at(i).normalVec.at<double>(0, 0))
//     + " " + DoubleToString(fianl3Dpoints.at(i).normalVec.at<double>(1, 0))
//     + " " + DoubleToString(fianl3Dpoints.at(i).normalVec.at<double>(2, 0))
//     + " " + DoubleToString(fianl3Dpoints.at(i).sacle)
//     + " " + DoubleToString(fianl3Dpoints.at(i).orientation) << endl;
//     matrix3dpoints.at<double>(0, i) = fianl3Dpoints.at(i).point3D.x;
//     matrix3dpoints.at<double>(1, i) = fianl3Dpoints.at(i).point3D.y;
//     matrix3dpoints.at<double>(2, i) = fianl3Dpoints.at(i).point3D.z;
//     matrix3dpoints.at<double>(3, i) = 1;
//     for(int j =0; j< fianl3Dpoints.at(i).features1.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features1.at(j))+" ";
//     }
//     outfile << endl;
//     for(int j =0; j< fianl3Dpoints.at(i).features2.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features2.at(j))+" ";
//     }
//     outfile << endl;
//     for(int j =0; j< fianl3Dpoints.at(i).features3.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features3.at(j))+" ";
//     }
//     outfile << endl;
//     for(int j =0; j< fianl3Dpoints.at(i).features4.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features4.at(j))+" ";
//     }
//     outfile << endl;
//     for(int j =0; j< fianl3Dpoints.at(i).features5.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features5.at(j))+" ";
//     }
//     outfile << endl;
//     for(int j =0; j< fianl3Dpoints.at(i).features6.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features6.at(j))+" ";
//     }
//     outfile << endl;
//     for(int j =0; j< fianl3Dpoints.at(i).features7.size(); j++)
//     {
//       outfile << DoubleToString(fianl3Dpoints.at(i).features7.at(j))+" ";
//     }
//     outfile << endl;
//     
//   }
//   outfile.close();
//   
//   for(int i =0; i< matric.size(); i++)
//   {
//     projection_normal(matrix3dpoints, matric.at(i),  visualize + cameraposefiles.at(i).substr(0, cameraposefiles.at(i).length() - 4) + ".jpg", i);
//   }
//   
// }
// 

// int featureOperation::projection_normal()
// {
//   string traingimageProjectionMatrixPath = traingImagesPosePath;
//   string visualize = trainingImagesPath;
//   
//   mats matrix3D=read3Dpoints(plypath);
//   
//   //Mat P_w  =  matrix3D.matrix3d.col(P_wnum).rowRange(0,3);
//   fileOperation file;
//   vector<string> cameraposefiles = file.getFilenames(traingimageProjectionMatrixPath, "*.txt");
//   vector<Mat> matric =file.getCameraPose(traingimageProjectionMatrixPath, cameraposefiles, 1);
//   string image = visualize + cameraposefiles.at(7).substr(0, cameraposefiles.at(7).length() - 4) + ".jpg";
//   vector<correspondingFrom3dto2d> matrix2DreX  = get2DpointsAfterNormalVec(matrix3D, matric.at(7), image);
//   
//   Mat imageForShow = imread(image);
//   Point2f pt;
//   int cou = 0;    
//   
//   for(int i = 0; i < matrix2DreX.size() ; i++){
//     matrix2DreX.at(i).point2d.x /= matrix2DreX.at(i).point2d.z;
//     matrix2DreX.at(i).point2d.y /= matrix2DreX.at(i).point2d.z;
//     matrix2DreX.at(i).point2d.z /= matrix2DreX.at(i).point2d.z;
//     
//     if(matrix2DreX.at(i).point2d.x  > 0 
//       && matrix2DreX.at(i).point2d.y> 0 
//       && matrix2DreX.at(i).point2d.x <imageForShow.cols 
//       &&matrix2DreX.at(i).point2d.y<imageForShow.rows)
//     {
//       cou++;
//       pt.x = (int)matrix2DreX.at(i).point2d.x;
//       pt.y = (int)matrix2DreX.at(i).point2d.y;
//       circle(imageForShow, pt, 1, CV_RGB(0, 255, 0), -1);
//     }
//   }
//   cout << cou << endl;
//   namedWindow("result",CV_WINDOW_AUTOSIZE);
//   imshow("result", imageForShow);
//   imwrite("result_normal.jpg", imageForShow);
//   waitKey();
//   
//   return 1;
// }


int featureOperation::projection_normal(Mat matrix3D,
					Mat  position,
					string testimage,
					int num)
{
	Mat matrix2DreX=position * matrix3D;
	Mat imageForShow = imread(testimage);
	Point2f pt;
	int cou = 0;    
	
	for(int i = 0; i < matrix2DreX.cols ; i++)
	{
		matrix2DreX.at<double>(0,i)=matrix2DreX.at<double>(0,i)/matrix2DreX.at<double>(2,i);
		matrix2DreX.at<double>(1,i)=matrix2DreX.at<double>(1,i)/matrix2DreX.at<double>(2,i);
		matrix2DreX.at<double>(2,i)=matrix2DreX.at<double>(2,i)/matrix2DreX.at<double>(2,i);
	
		if(matrix2DreX.at<double>(0,i) > 0 
			&& matrix2DreX.at<double>(1,i) > 0 
			&& matrix2DreX.at<double>(0,i) <imageForShow.cols 
			&& matrix2DreX.at<double>(1,i) <imageForShow.rows)
		{
			cou++;
			pt.x = (int)matrix2DreX.at<double>(0,i);
			pt.y = (int)matrix2DreX.at<double>(1,i);
			circle(imageForShow, pt, 1, CV_RGB(0, 255, 0), -1);
		}
	}
	namedWindow("result",CV_WINDOW_AUTOSIZE);
	imshow("result", imageForShow);
	imwrite("result"+IntToString(num)+".jpg", imageForShow);
	
	return 1;
}


/**
*   if there are more than 1 3D points are coressponded to one training image, these 3D points should been reduced.
*/
vector<corresponding3D2Dpoints_new> featureOperation::reduceRedundancy( vector<corresponding3D2Dpoints_new> points3Dofoneimage)
{
	vector<corresponding3D2Dpoints_new> points3DofoneimageAfterReduced;
	vector<corresponding3D2Dpoints_new> points3Dofoneimage_tmp = points3Dofoneimage;
	for(int i = 0; i < points3Dofoneimage.size(); i ++)
	{
		int flag = 0; 
		for(int j = 0; j < points3Dofoneimage_tmp.size(); j ++)
		{
			if(points3Dofoneimage.at(i).point3D.x == points3Dofoneimage_tmp.at(j).point3D.x 
				&& points3Dofoneimage.at(i).point3D.y == points3Dofoneimage_tmp.at(j).point3D.y 
				&& points3Dofoneimage.at(i).point3D.z == points3Dofoneimage_tmp.at(j).point3D.z )
			{
				flag ++;
			}
		}
		if(flag == 1)
			points3DofoneimageAfterReduced.push_back(points3Dofoneimage.at(i));
	}

	return points3DofoneimageAfterReduced;
}


string featureOperation::DoubleToString(double i) 
//change Double to string type
{
	stringstream strStream;
	strStream<<i;
	string s = strStream.str();
	return s;
} 

string featureOperation::IntToString(int i) 
//change int to string type
{
	stringstream strStream;
	strStream<<i;
	string s = strStream.str();
	return s;
}

string featureOperation::FloatToString(float i)
{
	stringstream strStream;
	strStream<<i;
	string s = strStream.str();
	return s;
}
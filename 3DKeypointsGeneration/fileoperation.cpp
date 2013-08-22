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


#include "fileoperation.h"

fileOperation::fileOperation ()
{}

fileOperation::~fileOperation ()
{}


int cmp( const string &aa, const string &bb ){
	string a = aa;
	string b = bb;
	if( atoi(a.substr(0, a.length()-4).c_str()) < atoi(b.substr(0, b.length()-4).c_str()) )
		return 1;
	else
		return 0;
}


/**
   *  return all filetype's files in containedPath
   */
vector<string> fileOperation::getFilenames(const string &containedPath, 
					   const string &filetype)
{
	vector<string> filenames;
	string filetypetmp;
	//cout << filetype.find_first_of("*") << endl;
	if(filetype.find_first_of("*") == 0)
	{
			filetypetmp =  filetype.substr(1, filetype.length());
			//cout << filetypetmp << endl;
	}
	else
	{
			filetypetmp = filetype;
			//cout << filetypetmp << endl;
	}
	DIR* dirp;
	struct dirent* direntp;
	dirp = opendir(containedPath.c_str());
	if( dirp != NULL )
	{
		for(;;) 
		{
			direntp = readdir( dirp );
			if( direntp == NULL ) break;
			if(string(direntp->d_name).find(filetypetmp) == string(direntp->d_name).length()-filetypetmp.length())
			{
				filenames.push_back(direntp->d_name);
			}
		}
		closedir( dirp );
	}
	sort(filenames.begin(), filenames.end(),cmp);
	return filenames;      
}
  
/**
 *  return cameara poses (projection matrix) from file
 */
vector<Mat> fileOperation::getCameraPose(const string& path,  
					 vector<string> &cameraPoseFileName, 
					 int skipNum)
{
	vector<Mat> cameraPoses;
	for(int f = 0; f < cameraPoseFileName.size(); f++)
	{
		string cameraPoseFilePathAndName = path + cameraPoseFileName.at(f);
		ifstream ifs ( cameraPoseFilePathAndName.c_str() );
		if (!ifs) 
		{
			cerr << "Cannot open " << cameraPoseFilePathAndName << endl;
			exit(1);
		}
		
		for (int j = 0; j <skipNum; j++)
		{
			string num;
			ifs >> num; // just skip
			//cout << num << endl;;
		}

		
		vector<double> element;
		while (ifs.good()) 
		{    
			for(unsigned int j = 0; j < 12; j++)
			{
			double k;
			ifs >> k;
			if (!ifs.good())
			{
				if (j == 0) break;
				cerr << "Error while reading " << cameraPoseFilePathAndName << endl;
				exit(1);
			}
			//cout << k << endl;
			element.push_back(k);
			} // for j

			if (!ifs.good()) break;
		} // end of while


		//
		// copy all features to the argument
		//
		Mat cameraposestmp = Mat(3, 4, CV_64FC1);
		try {
			for(int i =0; i<3; i++)
			{
				for(int j = 0; j < 4; j ++)
				{
				//cout  << element.at(i*4+j) << endl;
				cameraposestmp.at<double>(i, j) = element.at(i*4+j) ;		    
				}
			}
			cameraPoses.push_back(cameraposestmp);
		} catch (bad_alloc e) {
			cerr << "memory allocate error" << endl;
			exit(1);
		}
		ifs.close();
	}
	return cameraPoses;  
}
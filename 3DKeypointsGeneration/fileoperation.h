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


#ifndef FILEOPERATION_H
#define FILEOPERATION_H

#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
using namespace std;

#include "cv.h"
#include "highgui.h"
using namespace cv;

class fileOperation
{
public:
	fileOperation ();  
  
	/**
	*  return all filetype's files in containedPath
	*/
	vector<string> getFilenames(const string &containedPath, const string &filetype);
	
	/**
	*  return cameara poses (projection matrix) from file
	*/
	vector<Mat> getCameraPose(const string& path, vector<string> &cameraPoseFileName, int skipNum); 	
	
	virtual ~ fileOperation ();
};

#endif // FILEOPERATION_H

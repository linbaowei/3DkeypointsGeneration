/*
 *   Copyright (c) 2012 <Baowei Lin> <lin-bao-wei@hotmail.com>
 * 
 *   Permission is hereby granted, free of charge, to any person
 *   obtaining a copy of this software and associated documentation
 *   files (the "Software"), to deal in the Software without
 *   restriction, including without limitation the rights to use,
 *   copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the
 *   Software is furnished to do so, subject to the following
 *   conditions:
 * 
 *   The above copyright notice and this permission notice shall be
 *   included in all copies or substantial portions of the Software.
 * 
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *   OTHER DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include "boost/program_options.hpp"

#include "options.h"
#include "featureoperation.h"

int main(int argc, char **argv)
{	
	options Option = parseOptions(argc, argv);
	featureOperation featureoperation(60.0, 				
							Option.plyFileof3DpointCloudFile, 
							Option.output_folder, 
							Option.trainingImagesFolder, 
							Option.traingImagesPoseFolder);
	featureoperation.generate3DpointswithFeatures();
	return 1;
}


/**
 * Parse command line options.
 * return options struct
 */
options parseOptions(int argc, char** argv)
{	
	namespace po = boost::program_options;	
	po::options_description desc("Options");
	desc.add_options()
	("help", "This help message.")
	("trainingImagesFolder", po::value<string>(), "trainingImagesFolder")
	("traingImagesPoseFolder", po::value<string>(), "traingImagesPoseFolder")
	("plyFileof3DpointCloudFile", po::value<string>(), "plyFileof3DpointCloudFile")
	("output_folder", po::value<string>(), "output_folder")
	("flag", po::value<int>(), "method to assert value to repeated 3D points, defult is 0")
	("repeatImagesThreshold", po::value<int>(), "repeatImagesThreshold, defult is 4")
	("viewPointNum", po::value<int>(), "viewPointNum, defult is 2000")
	("distanceThreshold", po::value<double>(), "distanceThreshold, projection points' distance, defult is 0.7");
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);    
	
	options Opt;	
	if (vm.count("help"))
	{
		cout << desc << endl;
		exit(0);
	}	
	Opt.flag = 0;
	Opt.repeatImagesThreshold = 4;
	Opt.viewPointNum = 2000;
	Opt.distanceThreshold =  0.7;  
	
	cout << endl;
	int flag = 0;
	if(vm.count("trainingImagesFolder"))        		 
		Opt.trainingImagesFolder = vm["trainingImagesFolder"].as<string>();
	else
	{
		cout << "trainingImagesFolder was not set yet." << endl; 
		flag++;		
	}
	
	if(vm.count("traingImagesPoseFolder"))       	
		Opt.traingImagesPoseFolder = vm["traingImagesPoseFolder"].as<string>();
	else
	{
		cout << "traingImagesPoseFolder was not set yet." << endl; 
		flag++;		
	}
		     
	if (vm.count("plyFileof3DpointCloudFile")) 		
		Opt.plyFileof3DpointCloudFile = vm["plyFileof3DpointCloudFile"].as<string>();
	else
	{
		cout << "plyFileof3DpointCloudFile was not set yet." << endl; 
		flag++;		
	}
	    
	if (vm.count("output_folder"))  	
		Opt.output_folder = vm["output_folder"].as<string>();	
	else
	{
		cout << "output_folder was not set yet." << endl; 
		flag++;		
	}
	    
	if(flag >0)
	{		
		    cout << "\nuseage:\n" << argv[0] << "\n\t--trainingImagesFolder" << "\t\t[YOURINPUT]\n"
		    << "\t--traingImagesPoseFolder" << "\t[YOURINPUT]\n"
		    << "\t--plyFileof3DpointCloudFile" << "\t[YOURINPUT]\n"
		    << "\t--output_folder" << "\t\t\t[YOURINPUT]\n" << endl;
		    exit(1);  
	}	    
	
	if (vm.count("flag"))        
		Opt.flag = vm["flag"].as<int>();  
	if (vm.count("repeatImagesThreshold"))     
		Opt.repeatImagesThreshold = vm["repeatImagesThreshold"].as<int>();
	if (vm.count("viewPointNum")) 	
		Opt.viewPointNum = vm["viewPointNum"].as<int>();
	if (vm.count("distanceThreshold"))  	 	
		Opt.distanceThreshold = vm["distanceThreshold"].as<double>();  
	    
	cout << "trainingImagesFolder "		<< Opt.trainingImagesFolder	 << endl;
	cout << "traingImagesPoseFolder " 	<< Opt.traingImagesPoseFolder	 << endl;
	cout << "plyFileof3DpointCloudFile "	<< Opt.plyFileof3DpointCloudFile << endl;
	cout << "output_folder "  			<< Opt.output_folder 			<< endl;
	cout << "flag "					<< Opt.flag 				<< endl;
	cout << "repeatImagesThreshold "	<< Opt.repeatImagesThreshold 	<< endl;
	cout << "viewPointNum "			<< Opt.viewPointNum 		<< endl;
	cout << "distanceThreshold "		<< Opt.distanceThreshold		 << endl; 	    
	    
	return Opt;
}
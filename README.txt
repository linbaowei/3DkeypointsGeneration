3D Keypoints Detection from a 3D Point Cloud for Real-Time Camera Tracking, IEEJ Transactions on Electronics, Information and Systems, Vol. 133, No. 1, pp.84-90 (2013).
https://github.com/linbaowei/3DkeypointsGeneration
Baowei Lin, Toru Tamaki, Marcos Slomp, Bisser Raytchev, Kazufumi Kaneda, Koji Ichii (Hiroshima University)
Contact address: lin-bao-wei@hotmail.com
===================================================================







* What can you get
-------------------------------------------------------------------
	Usually, you can get 3D points by using Bundler and PMVS2 directly. However, there are too many 3D points for real time calculation. So we proposed a method to generate 3D keypoints for camera pose estimation. By using our method, we can reduce the number of 3D points which means the 3D keypoints. Also, the distribution of the 3D keypoints is good. Then by describing the 3D keypoints, good correspondences between 3D keypoints and 2D SIFT (we use 128 deminsion SIFT in our code, technically, any other 2D local feature extraction methods are all OK) are estimated. So, camera poses of the 2D images can be easily and fast estimated.
	
	If you download the files for the proposed method and uncompress it, there are 3 folders as follows:
		1. folder files_for_Bundler		//files for using Bundler and PMVS2
		2. folder sift_for_bundler		//files for using SIFTGPU
		3. folder 3DKeypointsGeneration	//files for 3D keypoints generation
	They are explained in following parts.











* Requirements
-------------------------------------------------------------------
	1. OpenCV 2.4.0
	2. Boost C++ Libraries 1.47.0
	3. SIFTGPU
	4. 3D point cloud
	5. 2D training images used for 3D reconstruction
	6. camera positions for 2D training images
	(4, 5 and 6 could be obtained from Bundler+PMVS2 http://www.cs.cornell.edu/~snavely/bundler/   http://francemapping.free.fr/Portfolio/Prog3D/PMVS2.html)
		
	how to use bundler and PMVS2
	-----------------------------------------------------------
		In order to use Bundler+PMVS2 easily, we provide two folders of files as follows:
			1. folder files_for_Bundler 	//chmod your files to get permission, for example, "chmod 775 ccd_widths.pl ToSiftGPU.sh RunBundler_siftgpu.sh construction_begin_gpu.sh"
				ccd_widths.pl			//save your camera ccd width information (you can find such information from your camera's user instruction)
				sift_for_bundler		//extract sift features by using SIFTGPU, see next "folder sift_for_bundler"
				ToSiftGPU.sh			//call sift_for_bundler and make preparation
				RunBundler_siftgpu.sh	//prepare to implement Bundler and PMVS2
				construction_begin_gpu.sh  	//execute this shell file to begin with Bundler and PMVS2		
			2. folder sift_for_bundler		
				compile this project, need OpenCV 2.4.0 and SIFTGPU, by using follow commands

				bash-4.2$  cd /sift_for_bundler	//depend on where you save our provided files
				bash-4.2$  mkdir build
				bash-4.2$  cd build
				bash-4.2$  cmake ..
				bash-4.2$  make
				bash-4.2$  cp sift_for_bundler ../../files_for_Bundler
		Then, copy the files in /files_for_Bundler to the same parent_path of /traingingImages. Here /trainingImages stores your 2D images used to reconstruct 3D point cloud. 

		Example: suppose you want to reconstruct 3D point cloud from the 2D images stored in /home/user/bundler/trainingImages/, execut the commands as follows:
			bash-4.2$  cd /files_for_Bundler		//depand on where you save our provided files
			bash-4.2$  su
			bash-4.2$  chmod 777 *
			bash-4.2$  exit
			bash-4.2$  cp * /home/user/bundler/
			bash-4.2$  sh construction_begin_gpu.sh










* How to use 3D keypoint generation method
-------------------------------------------------------------------

	usage:
	-----------------------------------------------------------
		./3DKeypointGeneration 
		  --trainingImagesFolder YOURINPUT		//input: 2D training images which used for 3D reconstruction (training images in Bundler)
	 	  --traingImagesPoseFolder YOUR INPUT	//input: camera positions of 2D training images (poses in Bundler)	
	 	  --plyFileof3DpointCloudFile YOURINPUT	//input: generated 3D point clouds file (ply file in Bundler)
	 	  --output_folder YOURINPUT			//output: target folder to save generated 3D keypoints

		./3DKeypointGeneration --help		//check the options from command line


	Example:
	----------------------------------------------------------- 
		./3DKeypointGeneration  --trainingImagesFolder /home/user/bundler/training3Dpoints/PMVS2inputImages/ 
					    --traingImagesPoseFolder /home/user/bundler/training3Dpoints/ProjectionMatrices/ 
					    --plyFileof3DpointCloudFile /home/user/bundler/training3Dpoints/PMVS2output_level2.ply 
					    --output_folder /home/user/bundler/output_folder

		'/training3Dpoints/PMVS2inputImages' is a folder created by our shell file when implementing Bundler and PMVS2. The images in this folder are the images used for 3D reconstruction. We also extract SIFT features for the 2D keypoints for these images. Our method will load these keypoints feature files from this folder.
		'traingImagesPoseFolder' is a folder created by our shell file when implementing Bundler and PMVS2. The files here saving camera positions of the corresponding training images in '/training3Dpoints/PMVS2inputImages'.
		'training3Dpoints/PMVS2output_level2.ply' is generated 3D point cloud by Bundler and PMVS2.
		'output_folder' is the place to save the results of 3D keypoints. Each file is the collection of 3D keypoints by changing thresholds th_v (please refer to our paper at the bottom in this document). The formate of it is almost similar as 2D SIFT feature files'. The difference is that, instead of SIFT keypoints' 2D coordinate, the data of one 3D keypoint is: 
[3Dkeypoint.x 3Dkeypoint.y 3Dkeypoint.z 3Dkeypoint.normalx 3Dkeypoint.normaly 3Dkeypoint.normalz 3Dkeypoint.scale 3Dkeypoint.orientation]
[128 dimension vector]
		



	How to compile 3D keypoint generation project
	-----------------------------------------------------------

		in order to compile 3D keypoint generation project, at least, OpenCV 2.4.0, SIFTGPU and Boost C++ Libraries 1.47.0 are necessary. How to use cmake to compile with OpenCV, SIFTGPU and Boost, Please refer to they homepage.
		bash-4.2$  cd /3DKeypointsGeneration	//depand on where you save our provided files
		bash-4.2$  mkdir build
		bash-4.2$  cd build
		bash-4.2$  cmake ..
		bash-4.2$  make
		bash-4.2$  ./3DkeypointGeneration ......
	




	The outputs of 3D keypoints are a set of files named as: output_[threshold1]_[threshold2]. Depand on the values of thresholds, the number of 3D keypoints are different. Please select appropriate thresholds (appropriate 3D keypoints number) for yourself.

	After generating 3D keypoints, you can use anyway you like to find correspondences between 3D keypoints and 2D keypoints (such as ANN, FLANN libs) and then estimate the camera poses (such as RANSAC, or Opencv mehtod).


* License: MIT
--------------------------------------------------------------------

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



We kindly ask for users to refer
Baowei Lin, Toru Tamaki, Marcos Slomp, Bisser Raytchev, Kazufumi Kaneda, Koji Ichii: 3D Keypoints Detection from a 3D Point Cloud for Real-Time Camera Tracking, IEEJ Transactions on Electronics, Information and Systems, Vol. 133, No. 1, pp.84-90 (2013).
in your paper published by using our implementation. Thank you!







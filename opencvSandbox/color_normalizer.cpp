#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "color_hist_provider.h"
#define DEBUG 0

int main(int argc, char* argv[])
{
	cv::Mat img, imageq, histImage;

	//This takes video from a usb cam connected to the computer. If
	//you have a built in camera, change the argument from 1 to 0

	cv::VideoCapture cap(1);

	// Create an instance of color_hist_provider. See color_hist_provider.h for implementation.	

	color_hist_provider clr_hist;

	while(true)

	{
	
	cap >> img;	

	// Call .generate_hist method in color_hist_provider instance clr_hist. The arguments of this method
	// are destination source image, detination image and the parameter to show.
	// calling .generate_hist with the string "equalize" equalizes the image,
	// calling it with "hist" shows the color histograms.

	clr_hist.generate_hist(img, imageq, "equalize");

	// show original image

	cv::imshow("ORIGINAL", img);

	// show color equalized image. In theory, this should normalize the color distribution in the image
	// and show more vivid colors. 

	cv::imshow("EQUALIZED", imageq);

	cv::waitKey(20);
	
	}

	return 0;

}


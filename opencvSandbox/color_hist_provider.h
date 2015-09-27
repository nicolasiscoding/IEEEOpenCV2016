#ifndef COLOR_HIST_PROVIDER_H
#define COLOR_HIST_PROVIDER_H
#define NORM_MINMAX 32
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <stdexcept>

class color_hist_provider
{

	private:
		std::string hist_or_equalization;

	public:

		color_hist_provider();
		void generate_hist(const cv::Mat&, cv::Mat&, std::string);
};

#endif /* COLOR_HIST_PROVIDER_H */

color_hist_provider::color_hist_provider(){}

void color_hist_provider::generate_hist(const  cv::Mat &Image, cv::Mat &histoImage, std::string input)
{
	this->hist_or_equalization = input;

	if(this->hist_or_equalization == "hist")
		{
		int histSize = 255;

		float range[] = {0, 256};

		const float* histRange = {range};

		bool uniform = true;

		bool accumulate = false;

		cv::Mat b_hist, g_hist, r_hist;

		std::vector<cv::Mat> bgr_planes;

		cv::split (Image, bgr_planes);

		/****************** Compute histograms *****************/

		cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);

		cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);

		cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

		/******************************************************/

		int hist_w = 512;

		int hist_h = 400;

		int bin_w = cvRound((double) hist_w/hist_h);

		cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));

		cv::normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat());

		cv::normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat());

		cv::normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat());

		/************** Draw graph for each channel *************/

		for(int i = 1; i < histSize; i++)
		{
			cv::line(histImage, cv::Point(bin_w*(i-1), 
					hist_h - cvRound(b_hist.at<float>(i-1))), 
					cv::Point (bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))), 
					cv::Scalar(255, 0, 0), 2, 8, 0);

			cv::line(histImage, cv::Point(bin_w*(i-1), 
					hist_h - cvRound(b_hist.at<float>(i-1))), 
					cv::Point (bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))), 
					cv::Scalar(0, 255, 0), 2, 8, 0);

			cv::line(histImage, cv::Point(bin_w*(i-1), 
					hist_h - cvRound(b_hist.at<float>(i-1))), 
					cv::Point (bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))), 
					cv::Scalar(0, 0, 255), 2, 8, 0);
		}

		/******************************************************/

		histoImage = histImage;
	}

	/************* Equalize colors in iamge ****************/

	else if(hist_or_equalization == "equalize")
	{
		std::vector<cv::Mat> bgr_planes;

		cv::split(Image, bgr_planes);

		cv::equalizeHist(bgr_planes[0], bgr_planes[0]);

		cv::equalizeHist(bgr_planes[1], bgr_planes[1]);

		cv::equalizeHist(bgr_planes[2], bgr_planes[2]);

		cv::merge(bgr_planes, histoImage);
	}

	/******************************************************/

	else
	{
		throw std::invalid_argument("generate_hist only takes in 'equalize or 'hist'");
	}
}
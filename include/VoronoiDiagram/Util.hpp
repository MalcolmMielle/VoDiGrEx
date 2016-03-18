#ifndef UTIL22FEB2016_MAPMAKER
#define UTIL22FEB2016_MAPMAKER

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace AASS{
	namespace vodigrex{
	
		//dirty debug function TEST
		inline std::string type2str(int type) {
		std::string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch ( depth ) {
			case CV_8U:  r = "8U"; break;
			case CV_8S:  r = "8S"; break;
			case CV_16U: r = "16U"; break;
			case CV_16S: r = "16S"; break;
			case CV_32S: r = "32S"; break;
			case CV_32F: r = "32F"; break;
			case CV_64F: r = "64F"; break;
			default:     r = "User"; break;
		}

		r += "C";
		r += (chans+'0');
		
		std::cout << "TYPE of MAt is " << r.c_str() << std::endl;

		return r;
		}
	}
}

#endif
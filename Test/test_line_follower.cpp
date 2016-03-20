#include "LineFollower.hpp"
#include "ThinkerVoronoi.hpp"

int main(){
	cv::Mat line = cv::imread("../Test/ObstacleMap.png");
	AASS::vodigrex::LineFollower llll_3;

	llll_3.setD(2);
	llll_3.inputMap(line);
	llll_3.thin();
	
// 	llll_3.printGraph();
	//g.fromCustom2Boost(llll_3.getIntersections());
	
// 	cv::Mat maa_33 = line.clone();
// 	maa_33.setTo(cv::Scalar(0));
// 	llll_3.drawGraph(maa_3);
	
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", line);
	cv::waitKey(0);
// 	
// // 	AASS::vodigrex::LineFollower llll_3;
// 	
// 	llll_3.clear();
// 
// 	cv::Mat bug = cv::imread("../Test/ObstacleMap1.png");
// 	
// 	AASS::vodigrex::ThinkerVoronoi t;
// 	t.setMode(4);
// 	t.setDownSample(1);
// 	t.think(bug);
// 	cv::Mat vlll_3 = t.getResult();
// 	
// 	cv::imshow("yoooo", vlll_3);
// 	cv::waitKey(0);
// 	
// // 	cv::Mat vlll_3;
// // 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
// 	vlll_3.convertTo(vlll_3, CV_8U);
// 	
// 	cv::imshow("yoooo", vlll_3);
// 	cv::waitKey(0);
// 
// 	llll_3.setD(2);
// 	llll_3.inputMap(vlll_3);
// 	llll_3.thin();
// 	
// // 	llll_3.printGraph();
// 	//g.fromCustom2Boost(llll_3.getIntersections());
// 	
// 	cv::Mat maa_3 = vlll_3.clone();
// 	maa_3.setTo(cv::Scalar(0));
// // 	llll_3.drawGraph(maa_3);
// 	
// 	cv::imshow("yoooo", llll_3.getResult());
// // 	cv::imshow("graph", maa_3);
// 	cv::waitKey(0);
// 	
	
	cv::Mat bugtes = cv::imread("../Test/testbug2.png");
	
	llll_3.clear();
	llll_3.setD(2);
	llll_3.inputMap(bugtes);
	llll_3.thin();
	cv::imshow("bug?", llll_3.getResult());
// 	cv::imshow("graph", maa_3);
	cv::waitKey(0);
	
	
	
	
// 	llll_3.reset();	
	
}
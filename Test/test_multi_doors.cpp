#include <iostream>

#include <time.h>
#include <cstdlib>
#include <fstream>
#include <ctime> 

#include "LineFollower.hpp"
#include "ThinkerVoronoi.hpp"
#include "MultipleLineFollowerKeypoints.hpp"
#include "bettergraph/PseudoGraph.hpp"
#include "SimpleNode.hpp"
#include "utils/Utils.hpp"


int main()
{

	
		std::cout << 
	"/********************************************************************"
	<< std::endl << "second REAL test "<<std::endl;
	
	AASS::vodigrex::MultipleLineFollowerKeypoints<> llll_3;
	llll_3.setMarge(10);
	
	std::deque < std::pair < cv::Point, cv::Point > > dpoint;
	std::pair < cv::Point, cv::Point> pair;
	pair.first = cv::Point2i(100 ,0);
	pair.second = cv::Point2i(500 , 500);
	std::pair < cv::Point, cv::Point> pair2;
	pair2.first = cv::Point2i(100 ,500);
	pair2.second = cv::Point2i(100 , 0);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	llll_3.push_back(pair2.first, pair2.second);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;

	cv::Mat bug = cv::imread("../Test/DoorMap.png");
	
	AASS::vodigrex::ThinkerVoronoi  t;
	
	t.think(bug);
	cv::Mat vlll_3 = t.getResult();
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);
	
// 	cv::Mat vlll_3;
// 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
	vlll_3.convertTo(vlll_3, CV_8U);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);

	llll_3.setD(2);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	llll_3.inputMap(vlll_3);
	
	std::cout << "DOOR NUMBER before thin " << llll_3.sizeDoor() << std::endl;
	llll_3.thin();
		
	cv::Mat maa_3 = vlll_3.clone();
	maa_3.setTo(cv::Scalar(0));
	bettergraph::PseudoGraph<AASS::vodigrex::SimpleNodeNamed, AASS::vodigrex::SimpleEdge> graph = llll_3.getGraph(0);
	AASS::vodigrex::draw<AASS::vodigrex::SimpleNodeNamed, AASS::vodigrex::SimpleEdge>(graph, maa_3);

	
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::waitKey(0);
	
	
}
#include "LineFollowerGraph.hpp"
#include "Thinker_Voronoi.hpp"
#include "Utils/Utils.hpp"

int main(){
	cv::Mat line = cv::imread("../Test/ObstacleMap.png");
	AASS::VoDiGrEx::LineFollowerGraph<> llll_3;

	llll_3.setD(2);
	llll_3.inputMap(line);
	llll_3.thin();
	
// 	llll_3.printGraph();
	//g.fromCustom2Boost(llll_3.getIntersections(// 				cv::line(tmp, _graph[src].point, _graph[targ].point, color);
		
// 				cv::imshow("the graph being made", m);
// 				cv::imshow("the graph partial being made", tmp);
// 				cv::waitKey(0);));
	
// 	cv::Mat maa_33 = line.clone();
// 	maa_33.setTo(cv::Scalar(0));
// 	llll_3.drawGraph(maa_3);
	
	bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge> graph11 = llll_3.getGraph();
	
	cv::Mat maa_31 = line.clone();
	maa_31.setTo(cv::Scalar(0));
	draw(graph11, maa_31);
	
	cv::imshow("yoooo", llll_3.getResult());
// 	cv::imshow("graph", line);
	cv::imshow("graph11", maa_31);
	cv::waitKey(0);
	
// 	AASS::VoDiGrEx::LineFollower llll_3;
	llll_3.setMarge(10);
	
	llll_3.clear();

	cv::Mat bug = cv::imread("../Test/ObstacleMap1.png");
	
	AASS::VoDiGrEx::Thinker_Voronoi t;
	t.setMode(4);
	t.setDownSample(1);
	t.think(bug);
	cv::Mat vlll_3 = t.getResult();
	
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);
	
// 	cv::Mat vlll_3;
// 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
	vlll_3.convertTo(vlll_3, CV_8U);
	
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);

	llll_3.setD(2);
	llll_3.inputMap(vlll_3);
	llll_3.thin();
	
// 	llll_3.printGraph();
	//g.fromCustom2Boost(llll_3.getIntersections());
	
	bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge> graph = llll_3.getGraph();
	
	cv::Mat maa_3 = vlll_3.clone();
	maa_3.setTo(cv::Scalar(0));
	draw(graph, maa_3);
	
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::waitKey(0);
	
	
	
// 	llll_3.reset();	
	
}
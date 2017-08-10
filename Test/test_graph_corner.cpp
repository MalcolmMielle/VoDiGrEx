#include "LineFollowerGraphCorners.hpp"
#include "MultipleLineFollowerBase.hpp"
#include "SimpleNode.hpp"
#include "ThinkerVoronoi.hpp"
#include "utils/Utils.hpp"

int main(){
	cv::Mat line = cv::imread("../Test/emergbasement_flipped_nodoor.png");
// 	cv::Mat line = cv::imread("../Test/corner.png");
	AASS::vodigrex::MultipleLineFollowerBase<
		AASS::vodigrex::SimpleNode, 
		AASS::vodigrex::SimpleEdge, 
		typename AASS::vodigrex::LineFollowerGraphCorners<
			AASS::vodigrex::SimpleNode, 
			AASS::vodigrex::SimpleEdge
			> 
		> llll_3;

		AASS::vodigrex::LineFollowerGraphCorners<
			AASS::vodigrex::SimpleNode, 
			AASS::vodigrex::SimpleEdge
			> graph_corners;
		graph_corners.setD(2);
		graph_corners.setMaxDeviation((45 * M_PI) / 180);
		graph_corners.inputMap(line);
		
		graph_corners.print();
		
		graph_corners.thin();
		auto prior_graph = graph_corners.getGraph();
		
		cv::Mat maa_31 = line.clone();
		AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(prior_graph, maa_31);
		std::cout << "Number of nodes " << prior_graph.getNumVertices() << std::endl;
		cv::imshow("graph", maa_31);
		cv::waitKey(0);

	llll_3.setD(2);
// 	llll_3.getLineFollower().setMaxDeviation((80 * M_PI) / 180);
	llll_3.inputMap(line);
	
	llll_3.getLineFollower().print();
	
	llll_3.thin();
	
// 	llll_3.printGraph();
	//g.fromCustom2Boost(llll_3.getIntersections(// 				cv::line(tmp, _graph[src].point, _graph[targ].point, color);
		
// 				cv::imshow("the graph being made", m);
// 				cv::imshow("the graph partial being made", tmp);
// 				cv::waitKey(0);));
	
// 	cv::Mat maa_33 = line.clone();
// 	maa_33.setTo(cv::Scalar(0));
// 	llll_3.drawGraph(maa_3);
	for(size_t i = 0 ; i < llll_3.size() ; ++i){
	
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> graph11 = llll_3.getGraph(i);
		
		cv::Mat maa_31 = line.clone();
		maa_31.setTo(cv::Scalar(0));
		AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(graph11, maa_31);
		
		cv::imshow("yoooo", llll_3.getResult(i));
		cv::imshow("input", line);
	// 	cv::imshow("graph", line);
		cv::imshow("graph11", maa_31);
		cv::waitKey(0);
	}
	
// // 	AASS::vodigrex::LineFollower llll_3;
// 	llll_3.setMarge(10);
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
// 	bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> graph = llll_3.getGraph(0);
// 	
// 	cv::Mat maa_3 = vlll_3.clone();
// 	maa_3.setTo(cv::Scalar(0));
// 	AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(graph, maa_3);
// 	
// 	std::cout << "Number of nodes " << graph.getNumVertices() << std::endl;
// 	cv::imshow("yoooo", llll_3.getResult());
// 	cv::imshow("graph", maa_3);
// 	cv::waitKey(0);
// 	
	
	
// 	llll_3.reset();	
	
}
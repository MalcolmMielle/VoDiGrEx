#include "LineFollowerGraph.hpp"
#include "ThinkerVoronoi.hpp"
#include "utils/Utils.hpp"

int main(){
	
	AASS::vodigrex::LineFollowerGraph<> llll_3;
	cv::Mat bug = cv::imread("../Test/evacuation.png");
	
	AASS::vodigrex::ThinkerVoronoi t;
	t.setMode(4);
	t.setDownSample(1);
	t.setLevel(10);
	t.think(bug);
	cv::Mat vlll_3 = t.getResult();
	
	cv::imshow("yoooo", vlll_3);
	cv::imshow("input", bug);
	cv::waitKey(0);
	
// 	cv::Mat vlll_3;
// 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
	vlll_3.convertTo(vlll_3, CV_8U);

	llll_3.setD(2);
	llll_3.inputMap(vlll_3);
	llll_3.thin();
	
// 	llll_3.printGraph();
	//g.fromCustom2Boost(llll_3.getIntersections());
	
	bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> graph = llll_3.getGraph();
	
	cv::Mat maa_3 = vlll_3.clone();
	maa_3.setTo(cv::Scalar(0));
	AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(graph, maa_3);
	
	graph.print();
	std::cout << "Number of nodes " << graph.getNumVertices() << std::endl;
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::waitKey(0);
	
	
	
// 	llll_3.reset();	
	
}
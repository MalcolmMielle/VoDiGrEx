#include "MultipleLineFollower.hpp"
#include "ThinkerVoronoi.hpp"
#include "utils/Utils.hpp"

int main(){
	
	AASS::vodigrex::MultipleLineFollower<> llll_3;
	cv::Mat bug = cv::imread("../Test/buildingline.png", CV_LOAD_IMAGE_GRAYSCALE);
	
	bug.convertTo(bug, CV_8U);

	llll_3.setD(2);
	llll_3.inputMap(bug);
	llll_3.thin();
	
	bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> graph = llll_3.getGraph(0);
	
	cv::Mat maa_3 = bug.clone();
	maa_3.setTo(cv::Scalar(0));
	AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(graph, maa_3);
	
	graph.print();
	std::cout << "Number of nodes " << graph.getNumVertices() << std::endl;
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::waitKey(0);

}
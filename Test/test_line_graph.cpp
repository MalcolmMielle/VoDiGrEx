#include "LineFollowerGraph.hpp"
#include "Thinker_Voronoi.hpp"



inline void draw(const bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::Vertex& v, bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge> graph, cv::Mat& m)
{
	cv::Scalar color;
	if(m.channels() == 1){
		color = 255;
	}
	else if(m.channels() == 3){
		color[1] = 255;
		color[2] = 255;
		color[3] = 255; 
	}
	cv::Point2i pp;
	pp.x = graph[v].getX();
	pp.y = graph[v].getY();
	cv::circle(m, pp, 5, color);
		
}

		
inline void draw(bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge> graph, cv::Mat& m)
{
	
	std::cout << "DRAW" << std::endl;
	
	cv::Scalar color;
	if(m.channels() == 1){
		color = 255;
	}
	else if(m.channels() == 3){
		color[1] = 255;
		color[2] = 255;
		color[3] = 255; 
	}
	//first is beginning, second is "past the end"
	std::pair<bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::VertexIterator, bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::VertexIterator> vp;
	//vertices access all the vertix
	for (vp = boost::vertices(graph.getGraph()); vp.first != vp.second; ++vp.first) {
		
// 			cv::Mat tmp = cv::Mat::zeros(m.size(), CV_8UC1);
		
// 			std::cout << "NEW VERTEX" << std::endl;
		
		bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::Vertex v = *vp.first;
		cv::Point2i pp;
		pp.x = graph[v].getX();
		pp.y = graph[v].getY();
		cv::circle(m, pp, 5, color);
// 			cv::circle(tmp, _graph[v].point, 5, color);
// 
		bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::EdgeIterator out_i, out_end;
		bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::Edge e;
		
		for (boost::tie(out_i, out_end) = boost::out_edges(v, graph.getGraph()); 
			out_i != out_end; ++out_i) {
// 				std::cout << "NEW Edge" << std::endl;
			e = *out_i;
// 			bettergraph::PseudoGraph<AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::Vertex src = boost::source(e, graph.getGraph()), targ = boost::target(e, graph.getGraph());
// 			
// 			cv::Point2i pp_source;
// 			pp_source.x = graph[src].getX();
// 			pp_source.y = graph[src].getY();
// 			
// 			cv::Point2i pp_target;
// 			pp_target.x = graph[targ].getX();
// 			pp_target.y = graph[targ].getY();
// 			
// 			cv::line(m, pp_source, pp_target, color);
			
			for(size_t i = 0 ; i < graph[e].getLine().size() ; ++i){
				cv::Scalar color;
				if(m.channels() == 1){
					color = 155;
				}
				else if(m.channels() == 3){
					color[1] =150;
					color[2] =55;
					color[3] = 55; 
				}
				cv::Point2i pp;
				pp.x = graph[e].getLine()[i].first;
				pp.y = graph[e].getLine()[i].second;
				cv::circle(m, pp, 2, color);
			}
		}
		
	}
}

int main(){
	cv::Mat line = cv::imread("../Test/ObstacleMap.png");
	AASS::VoDiGrEx::LineFollowerGraph llll_3;

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
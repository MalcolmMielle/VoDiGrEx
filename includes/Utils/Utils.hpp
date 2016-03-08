#ifndef VODIGREX_UTILS_08032016
#define VODIGREX_UTILS_08032016

#include "bettergraph/PseudoGraph.hpp"
#include "SimpleNode.hpp"
#include <opencv2/opencv.hpp>

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

#endif

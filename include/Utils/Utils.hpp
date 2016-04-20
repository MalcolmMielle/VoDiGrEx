#ifndef VODIGREX_UTILS_08032016
#define VODIGREX_UTILS_08032016

#include "bettergraph/PseudoGraph.hpp"
// #include "LineFollower/LineFollowerDoors.hpp"
#include <opencv2/opencv.hpp>

namespace AASS{
	namespace vodigrex{

		template<typename VertexType, typename EdgeType>
		inline void draw(const typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex& v, bettergraph::PseudoGraph<VertexType, EdgeType> graph, cv::Mat& m)
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

		template<typename VertexType, typename EdgeType>	
		inline void draw(const typename bettergraph::PseudoGraph<VertexType, EdgeType> graph, cv::Mat& m)
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
			std::pair<bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::VertexIterator, bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(graph.getGraph()); vp.first != vp.second; ++vp.first) {
				
		// 			cv::Mat tmp = cv::Mat::zeros(m.size(), CV_8UC1);
				
		// 			std::cout << "NEW VERTEX" << std::endl;
				
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
				cv::Point2i pp;
				pp.x = graph[v].getX();
				pp.y = graph[v].getY();
				cv::circle(m, pp, 5, color);
		// 			cv::circle(tmp, _graph[v].point, 5, color);
		// 
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Edge e;
				
				for (boost::tie(out_i, out_end) = boost::out_edges(v, graph.getGraph()); 
					out_i != out_end; ++out_i) {
		// 				std::cout << "NEW Edge" << std::endl;
					e = *out_i;
					bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e, graph);
					cv::Point2i pp3;
					pp3.x = graph[targ].getX();
					pp3.y = graph[targ ].getY();
					cv::circle(m, pp3, 5, color);
					
// 					if(targ == v){
					
						for(int i = 0 ; i < (int) graph[e].getLine().size() - 1 ; ++i){
							
	// 						std::cout << i << " size " << graph[e].getLine().size() << std::endl;
							cv::Scalar color;
							if(m.channels() == 1){
								color = 155;
							}
							else if(m.channels() == 3){
								color[1] =150;
								color[2] =55;
								color[3] =55; 
							}
							cv::Point2i pp1;
							cv::Point2i pp2;
							pp1.x = graph[e].getLine()[i].first;
							pp1.y = graph[e].getLine()[i].second;
							pp2.x = graph[e].getLine()[i+1].first;
							pp2.y = graph[e].getLine()[i+1].second;
							cv::circle(m, pp1, 2, color);
// 							cv::line(m, pp1, pp2, color);
						}
					cv::line(m, pp, pp3, color);
// 					}
// 					cv::imshow("tmp", m);
// 					cv::waitKey(0);
				}
				
			}
		}
	}
}

#endif

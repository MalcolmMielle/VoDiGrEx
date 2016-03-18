#ifndef THINKER_VORNOI_DYNAMIC_14032016
#define THINKER_VORNOI_DYNAMIC_14032016

#include "LineFollowerGraph.hpp"
#include <deque>

namespace AASS{
	namespace vodigrex{
	
		template<typename VertexType = SimpleNode, typename EdgeType = SimpleEdge>
		class DynamicGraphExtractor{
		protected:
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex Vertex;
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::VertexIterator VertexIterator;
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge Edge;
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::EdgeIterator EdgeIterator;
			
			LineFollowerGraph<VertexType, EdgeType> _line_graph;
			cv::Mat _previous_image;
			cv::Mat _image;
			bettergraph::PseudoGraph<VertexType, EdgeType> _graph;
			bettergraph::PseudoGraph<VertexType, EdgeType> _previous_graph;
			std::deque<Edge> _to_remove_edges;
			std::deque<std::pair<Vertex, Vertex> > _vertex_to_search_again;
			
			
		public:
			DynamicGraphExtractor(){};
			void setGraphExtractor(const LineFollowerGraph<>& t){
				_line_graph = t;
			}
			LineFollowerGraph<VertexType, EdgeType>& getLineFollower(){return _line_graph;}
			const LineFollowerGraph<VertexType, EdgeType>& getLineFollower()const {return _line_graph;}
			
			void inputMap(const cv::Mat& in){in.copyTo(_image); _line_graph.inputMap(_image);}
			
			virtual void extract();
			
// 		protected:
			/**
			 * @brief Get all Edge in graph that were not modified in the line _previous_image*
			 */
			void getNonModifiedEdges();
			/**
			 * @brief Remove all non modified edge from the Image
			 */
			void removeEdgesFromImage();
			/**
			 * @brief Get all the vertices from which the algorithm needs to be run again
			 */
			void getAnchorVertices();
			
			void getEdges(const bettergraph::PseudoGraph<VertexType, EdgeType>& graph, cv::Mat& mat_in);
			
		};
		
		template<typename VertexType, typename EdgeType>
		inline void AASS::vodigrex::DynamicGraphExtractor<VertexType, EdgeType>::getNonModifiedEdges()
		{
			
			//Calculate image difference
			cv::Mat to_remove = this->_previous_image - this->_image;
			
			cv::Mat draw_edges = cv::Mat::zeros(to_remove.rows, to_remove.cols, to_remove.type());
			this->getEdges(_previous_graph, draw_edges);
			
			
			
			std::pair<VertexIterator, VertexIterator> vp;
			std::vector<Vertex> vec;
			for (vp = boost::vertices(this->_graph); vp.first != vp.second; ++vp.first) {
				
			}
		}

		template<typename VertexType, typename EdgeType>
		inline void AASS::vodigrex::DynamicGraphExtractor<VertexType, EdgeType>::removeEdgesFromImage()
		{

		}

		template<typename VertexType, typename EdgeType>
		inline void AASS::vodigrex::DynamicGraphExtractor<VertexType, EdgeType>::getAnchorVertices()
		{

		}

		template<typename VertexType, typename EdgeType>
		inline void AASS::vodigrex::DynamicGraphExtractor<VertexType, EdgeType>::extract()
		{

		}
		
		template<typename VertexType, typename EdgeType>
		inline void DynamicGraphExtractor<VertexType, EdgeType>::getEdges(const bettergraph::PseudoGraph< VertexType, EdgeType >& graph, cv::Mat& mat_in)
		{
			
			
			cv::Mat test = cv::Mat::zeros(mat_in.rows, mat_in.cols, mat_in.type());
			std::pair<VertexIterator, VertexIterator> vp;
			std::vector<Vertex> vec;
			//vertices access all the vertix
			//Classify them in order
			int i = 0;
			for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
				Vertex v = *vp.first;	
				EdgeIterator out_i, out_end;
				Edge e;

				for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)._graph); 
					out_i != out_end; ++out_i) {
					e = *out_i;
					//Draw the edge on test
					for(i = 0 ; graph[e].line.size() - 1 ; ++i){
						cv::Point2i p1, p2;
						p1.x = graph[e].line[i].first;
						p1.y = graph[e].line[i].second;
						p2.x = graph[e].line[i+1].first;
						p2.y = graph[e].line[i+1].second;
						cv::line(test, p1, p2, cv::Scalar(255));
					}
					
					//If the edge in test is in _previous_image, it gets removed. Otherwise it stays here and it means the edge does not exist anymore
					cv::Mat diff = test - _previous_image;
					
					//Test for collision and add the edge to the list to remove if it's part of it
					for(int row = 0 ; row < diff.rows ; row++){
						uchar* p = diff.ptr(row); //point to each row
						for(int col = 0 ; col < diff.cols ; col++){
							//p[j] <- how to access element
							if(p[col] > 0){
								this->_to_remove_edges.push_back(e);
// 								this->_vertex_to_search_again.push_back(std::pair<Vertex, Vertex>(v, _graph.getTarget(e, v) ));
							}
						}
					}
				
				}

			}
 
		}
	}
}

#endif
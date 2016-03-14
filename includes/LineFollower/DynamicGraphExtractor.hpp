#ifndef THINKER_VORNOI_DYNAMIC_14032016
#define THINKER_VORNOI_DYNAMIC_14032016

#include "LineFollowerGraph.hpp"

namespace AASS{
	namespace vodigrex{
	
		template<typename VertexType = SimpleNode, typename EdgeType = SimpleEdge>
		class DynamicGraphExtractor{
		protected:
			LineFollowerGraph<VertexType, EdgeType> _line_graph;
			cv::Mat _previous_image;
			
		public:
			DynamicGraphExtractor(){};
			void setGraphExtractor(const LineFollowerGraph<>& t){
				_line_graph = t;
			}
			LineFollowerGraph<VertexType, EdgeType>& getLineFollower(){return _line_graph;}
			const LineFollowerGraph<VertexType, EdgeType>& getLineFollower()const {return _line_graph;}
			
			void inputMap(const cv::Mat& in){_line_graph.inputMap(in);}
			
			virtual void extract();
			
		protected:
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
			
		};
		
		template<typename VertexType, typename EdgeType>
		inline void AASS::vodigrex::DynamicGraphExtractor<VertexType, EdgeType>::getNonModifiedEdges()
		{

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
		
	}
}

#endif
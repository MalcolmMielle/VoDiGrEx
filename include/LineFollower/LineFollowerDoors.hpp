#ifndef LINEFOLLOWERDOOR_POINT_MAP
#define LINEFOLLOWERDOOR_POINT_MAP

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <stdexcept>
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"

#include <opencv2/opencv.hpp>
#include "LineFollowerGraph.hpp"
#include "SimpleNodeNamed.hpp"

namespace AASS{
		
	namespace vodigrex{
		
		

		/**
		* @brief Line follower algorithm to create a graph and a thinned image.
		* 
		* Implementation of a line follower algorithm inspired by the algorithm of 
		* Orit BARUCH - Line Thinning by line following - Pattern Recognition Letters 8 1988
		* This algorithm is iterative and able to adapt to any line size.
		* Create a thinned image and graph of the lines.
		*/
		template<typename VertexType = SimpleNodeNamed, typename EdgeType = SimpleEdge>
		class LineFollowerDoors : public LineFollowerGraph<VertexType, EdgeType>{
			
			
		protected : 
			std::deque < cv::Point > _doors;
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex Vertex;
			
		public:
			
			LineFollowerDoors(){};
			virtual ~LineFollowerDoors(){

			}
			
			void setDoors(const std::deque <cv::Point>& d){ _doors = d;}
			void push_back(const cv::Point& d1, const cv::Point& d){_doors.push_back(d1); _doors.push_back(d);}
			int sizeDoor(){return _doors.size();}
			/**
			* @brief line thining algorithm
			* 
			*/
// 			virtual void thin(){
// 				LineFollowerGraph<VertexType, EdgeType>::thin();
// 			}
			
			bool wCrossDoor();
			bool squareLineCollision (const cv::Point& p1, const cv::Point& p2, float minX, float minY, float maxX, float maxY);
			virtual void clear();
		protected:
// 			void init();

			/**
			* @brief line thining algorithm after init.
			* 
			*/
			virtual void lineThinningAlgo(Vertex& index_dad);

		};
		
		#include "LineFollowerDoors.tpl"

	}
}

#endif
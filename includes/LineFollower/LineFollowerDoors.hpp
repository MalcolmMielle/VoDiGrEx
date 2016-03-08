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

namespace AASS{
		
	namespace VoDiGrEx{

		/**
		* @brief Line follower algorithm to create a graph and a thinned image.
		* 
		* Implementation of a line follower algorithm inspired by the algorithm of 
		* Orit BARUCH - Line Thinning by line following - Pattern Recognition Letters 8 1988
		* This algorithm is iterative and able to adapt to any line size.
		* Create a thinned image and graph of the lines.
		*/
		template<typename VertexType = SimpleNode, typename EdgeType = SimpleEdge>
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
		
		

		template<typename VertexType, typename EdgeType>
		inline void LineFollowerDoors<VertexType, EdgeType>::lineThinningAlgo(Vertex& index_dad)
		{
			
			
			std::cout << "LINE THINING DOOR" << std::endl;
			
			Vertex dad_vertex = index_dad;
			while(this->_LP.x != -1 && this->_LP.y != -1){

				std::vector<cv::Point2i> all_point;
				bool non_dead_end = this->findNextLPRP(all_point);
				
				cv::Size s;
				cv::Point2i p_dyn_window;
				this->_W.locateROI(s, p_dyn_window);
				
				cv::Point2i new_p;
				new_p.x = p_dyn_window.x + (this->_W.cols / 2);
				new_p.y = p_dyn_window.y + (this->_W.rows / 2);
				
				
				/*
				//Crossing a door
				if(wCrossDoor()){
					
					Vertex new_dad;
					bool is_new = loopDetection( new_p, dad_vertex, new_dad);
					
					Vertex created = new_dad;
					std::cout << "GOT A DOOR" << std::endl;
					if(is_new == true){
						std::string s = "Door";
						_graph.addVertex(s, m, new_p, created, new_dad);
					}
					else{
						if(new_dad != dad_vertex){
							_graph.addEdge(new_dad, dad_vertex);
						}
					}
					
					addPoint2Explore(all_point, created);
					removeLineSegment(_W);
					
					if(_LRP_to_explore.size() > 0){
					
						//Access new LP RP
						_RP = _LRP_to_explore[0].first; 
						_LP = _LRP_to_explore[0].second;
						dad_vertex = _dad_vertex.at(0);
						_last_drawing_point = _last_drawing_point_deque[0];
						//Remove them
						_LRP_to_explore.pop_front();
						_dad_vertex.pop_front();
						_last_drawing_point_deque.pop_front();
						
						drawLine();
						moveForward();
					}
					//END
					else{
	// 					std::cout << "size is : " << all_point.size() << std::endl;
	// 					std::cout << "end"<<std::endl;
						_LP.x = -1;
						_LP.y = -1;
					}
					
					//Needed to avoid an infinite loop :
					type = typeOfIntersection(_W);
					
				}
				*/
				
				
				

				//Intersection or dead end
				if( all_point.size() > 2 || non_dead_end == false || wCrossDoor()){
					
					//USE : _all_crossings
					Vertex new_dad;
					bool already_exist = this->loopDetection(new_p, new_dad);

					//New intersection
					if(already_exist == false){
						this->addVertex(dad_vertex, new_dad);
					}
					//Not a new intersection but still an intersection
					else{
						if(new_dad != dad_vertex){
							bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
							SimpleEdge sed;
							sed.setLine(this->_line);
							this->_line.clear();
							this->_graph.addEdge(ed, new_dad, dad_vertex, sed);
						}
					}
					
					this->addPoint2Explore(all_point, new_dad);
					this->getNewBranch(dad_vertex);
							
					
				}
				else{				

					//Making sure the line isn't actually a previsouly erased crossing.
					Vertex loop_vertex; 
					bool already_seen = this->loopDetection(new_p, loop_vertex);
					
					if(already_seen == true){
						if(loop_vertex != dad_vertex){
							bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
							SimpleEdge sed;
							sed.setLine(this->_line);
							this->_line.clear();
							this->_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
							dad_vertex = loop_vertex;
						}
					}
					
					this->_LP = all_point[0];
					this->_RP = all_point[1];
					this->moveForward();		
				}
			}
		}


		template<typename VertexType, typename EdgeType>
		inline void LineFollowerDoors<VertexType, EdgeType>::clear()
		{
			LineFollowerGraph<VertexType, EdgeType>::clear();
			_doors.clear();
		}

		
		template<typename VertexType, typename EdgeType>
		inline bool LineFollowerDoors<VertexType, EdgeType>::squareLineCollision (const cv::Point& p1, const cv::Point& p2, float minX, float minY, float maxX, float maxY) {  
			// Completely outside.
			if ((p1.x <= minX && p2.x <= minX) || (p1.y <= minY && p2.y <= minY) || (p1.x >= maxX && p2.x >= maxX) || (p1.y >= maxY && p2.y >= maxY))
				return false;

			//TODO straight line !
			if(p2.x - p1.x != 0 ){
				float m = (p2.y - p1.y) / (p2.x - p1.x);

	// 			std::cout << "M " << m << std::endl;
				float y = m * (minX - p1.x) + p1.y;
				if (y > minY && y < maxY) return true;

				y = m * (maxX - p1.x) + p1.y;
				if (y > minY && y < maxY) return true;

				float x = (minY - p1.y) / m + p1.x;
				if (x > minX && x < maxX) return true;

				x = (maxY - p1.y) / m + p1.x;
				if (x > minX && x < maxX) return true;

				return false;
			}
			else{
				if(p1.y <= minY && p2.y >= maxY){
					return true;
				}
				else if(p1.y >= maxY && p2.y <= minY){
					return true;
				}
				else if(p1.y <= maxY && p1.y >= minY){
					if(p2.y > maxY){
						return true;
					}
				}
				else if(p2.y <= maxY && p2.y >= minY){
					if(p1.y < minY){
						return true;
					}
				}
				return false;
				
			}
		}

		template<typename VertexType, typename EdgeType>
		inline bool LineFollowerDoors<VertexType, EdgeType>::wCrossDoor()
		{
			bool res = false;
			
			cv::Point2i p;
			cv::Size s;
			this->_W.locateROI(s, p);
			int height = this->_W.rows;
			int width = this->_W.cols;
			
// 			std::cout << "trying the door detection with " << _doors.size() << std::endl;
			
			for(size_t i = 0 ; i < _doors.size() ; i = i+2){
// 				std::cout << " at : " << _doors[i].x << " at : " << _doors[i].y << std::endl;
// 				std::cout <<  p.x << " " << p.y << " " << p.x + width << " " <<p.y + height << std::endl;
				if(res == false){
					res = squareLineCollision(_doors[i], _doors[i + 1], p.x, p.y, p.x + width, p.y + height);
// 					std::cout << "it is " <<res << std::endl;
				}
				else{
					break;
				}
					
			}
			return res;

		}

		
	}
}

#endif
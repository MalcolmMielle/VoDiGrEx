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
			typedef typename LineFollowerGraph<VertexType, EdgeType>::Vertex Vertex;
			
		public:
			
			LineFollowerDoors() : LineFollowerGraph(){};
			virtual ~LineFollowerDoors(){

			}
			
			void setDoors(const std::deque <cv::Point>& d){ _doors = d;}
			void push_back(const cv::Point& d1, const cv::Point& d){_doors.push_back(d1); _doors.push_back(d);}
			int sizeDoor(){return _doors.size();}
			/**
			* @brief line thining algorithm
			* 
			*/
// 			void thin();
			
			bool wCrossDoor();
			bool squareLineCollision (const cv::Point& p1, const cv::Point& p2, float minX, float minY, float maxX, float maxY);
			void clear();
		protected:
			void init();

			/**
			* @brief line thining algorithm after init.
			* 
			*/
			void lineThinningAlgo(topologicalmap::Vertex index_dad);

		};

		
		
// 		template<typename VertexType, typename EdgeType>
// 		inline void LineFollowerDoors<VertexType, EdgeType>::thin()
// 		{
// 			try{
// 				cv::copyMakeBorder( _map_in, _map_in, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0 );
// 		// 		std::cout << "Init " << std::endl;
// 				//Build the first ROI
// 				init();
// 				//labelSegments();
// 			
// 		// 			std::cout << "out of init " << std::endl;
// 				
// 				//TODO need to be more global than that.
// 				//This find new points
// 				//Init the first drawing point
// 				cv::Point2i p;
// 				cv::Size s;
// 				_W.locateROI(s, p);
// 				
// 				_last_drawing_point = cv::Point(p.x + (_W.rows/2), p.y + (_W.cols/2));
// 				
// 		// 			std::cout << "We start " << " at " << p.x << " " << p.y << " dyn wind " << _W.rows << " " << _W.cols << " draw oint " << _last_drawing_point << std::endl;
// 				
// 				std::vector<cv::Point2i> all_point;
// 				findNextLPRP(all_point);
// 				_LP = all_point[0];
// 				_RP = all_point[1];
// 				
// 				Vertex dad;
// 				
// 				cv::Mat m = _W.clone();
// 				//Is a dead end
// 				if(all_point.size() == 2){
// 					cv::Mat m = _W.clone();
// 					std::string s = "DeadEnd";
// 					_graph.addVertex(s, m, _RP, dad);
// 					lineThinningAlgo(dad);
// 				}
// 				//Line
// 				else{
// 		// 				addPoint2Explore(all_point, dad);
// 					if(all_point.size() == 4){
// 		// 					std::cout << "Not a good start" << std::endl;
// 						std::string s = "WrongCrossing";
// 						_graph.addVertex(s, m, _RP, dad);
// 		// 					addPoint2Explore(all_point, dad);
// 						lineThinningAlgo(dad);
// 					}
// 					else{
// 						if(all_point.size() == 6){
// 		// 						std::cout <<"Adding a T" << std::endl;
// 							std::string s = "TCrossing";
// 							_graph.addVertex(s, m, _RP, dad);
// 		// 						addPoint2Explore(all_point, dad);
// 		// 						printGraph();
// 						}
// 						if(all_point.size() == 8){
// 		// 						std::cout <<"Adding a X" << std::endl;
// 							std::string s = "XCrossing";
// 							_graph.addVertex(s, m, _RP, dad);
// 		// 						printGraph();
// 		// 					addPoint2Explore(all_point, index, -1);
// 						}
// 						else{
// 		// 						std::cout <<"Adding a N" << std::endl;
// 							std::string s = "NCrossing";
// 							_graph.addVertex(s, m, _RP, dad);
// 		// 						printGraph();
// 		// 					addPoint2Explore(all_point, index, -1);
// 						}
// 						addPoint2Explore(all_point, dad);
// 						lineThinningAlgo(dad);
// 					}
// 					
// 				}
// 			
// 			}
// 			catch(const std::exception &e){
// 				std::cout << "the unthinkable happened during voronoi landmark detection : " << e.what() << std::endl;
// 			}
// 		}

		template<typename VertexType, typename EdgeType>
		inline void LineFollowerDoors<VertexType, EdgeType>::lineThinningAlgo(Vertex index_dad)
		{
			
			
			Vertex dad_vertex = index_dad;
			while(_LP.x != -1 && _LP.y != -1){
				
				int type = typeOfIntersection(_W);
				//If we lost it we upsize W
				
				while(type == 0){
					upResize();
					type = typeOfIntersection(_W);
				}

				std::vector<cv::Point2i> all_point;
				bool non_dead_end = findNextLPRP(all_point);
				
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
				
				cv::Point2i new_p;
				new_p.x = p_dyn_window.x + (_W.cols / 2);
				new_p.y = p_dyn_window.y + (_W.rows / 2);
				
				
				
				//Crossing a door
				if(wCrossDoor()){
					cv::Mat m = _W.clone();
					Vertex new_dad;
					bool is_new = _graph.loopDetection( new_p, dad_vertex, new_dad);
					
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

				if( all_point.size() > 2 ){
					
					//USE : _all_crossings
					Vertex new_dad;
					bool already_exist = loopDetection(new_p, new_dad);

					//New intersection
					if(already_exist == false){
						addVertex(dad_vertex, new_dad);
					}
					//Not a new intersection but still an intersection
					else{
						if(new_dad != dad_vertex){
							bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
							SimpleEdge sed;
							sed.setLine(_line);
							_line.clear();
							_graph.addEdge(ed, new_dad, dad_vertex, sed);
						}
					}
					
					addPoint2Explore(all_point, new_dad);
					cv::Point2i olddrawpoint = _last_drawing_point;
					removeLineSegment(_W);
					
					if(_LRP_to_explore.size() > 0){
					
						_last_drawing_point = olddrawpoint;
						//Access new LP RP
						_RP = _LRP_to_explore[0].first; 
						_LP = _LRP_to_explore[0].second;
						
		// 						index = _dads_index[0];
						dad_vertex = _dad_vertex.at(0);
						
						_last_drawing_point = _last_drawing_point_deque[0];
						//Remove them
						_LRP_to_explore.pop_front();
		// 					_dads_index.pop_front();
						_dad_vertex.pop_front();
						_last_drawing_point_deque.pop_front();
						
						drawLine();
						moveForward();
					}
					//END
					else{
						_LP.x = -1;
						_LP.y = -1;
					}
					
					//Needed to avoid an infinite loop :
					type = typeOfIntersection(_W);				
					
				}
				else{				
					if(non_dead_end == false){
		// 					std::cout << "reach a dead end" << std::endl;
						cv::Mat m = _W.clone();

						Vertex loop_vertex; 
						bool already_exist = loopDetection(new_p, loop_vertex);
												
						if(already_exist == false){
							addVertex(dad_vertex, loop_vertex);
						}
						else{
							
							if(loop_vertex != dad_vertex){
		// 							boost::add_edge(loop_index, index , _graph);
								bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
								SimpleEdge sed;
								sed.setLine(_line);
								_line.clear();
								_graph.addEdge(ed, loop_vertex, dad_vertex, sed);

							}
						}
					
						if(_LRP_to_explore.size() == 0){
							_LP.x = -1;
							_LP.y = -1;
						}
						else{
							
							_RP = _LRP_to_explore[0].first; 
							_LP = _LRP_to_explore[0].second;
							dad_vertex = _dad_vertex.at(0);
							_last_drawing_point = _last_drawing_point_deque[0];

							_LRP_to_explore.pop_front();
							_dad_vertex.pop_front();
							_last_drawing_point_deque.pop_front();

							//Move the dynamic window
							moveForward();
						}

						
					}
					
					else{
		// 					std::cout << "Moving forward " << std::endl;
						_LP = all_point[0];
						_RP = all_point[1];
						
						Vertex loop_vertex; 
						bool already_seen = loopDetection(new_p, loop_vertex);
						
						//Making sure the line isn't actually a previsouly erased crossing.
						if(already_seen == true){
							if(loop_vertex != dad_vertex){
		// 							boost::add_edge(loop_index, index , _graph);
								bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
								SimpleEdge sed;
								sed.setLine(_line);
								_line.clear();
								_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
								dad_vertex = loop_vertex;
							}
						}
						
						drawLine();
						removeLineSegment(_W);
						moveForward();
						
						type = typeOfIntersection(_W);
						
						
					}			
				}
			}
		}

		
		template<typename VertexType, typename EdgeType>
		inline void LineFollowerDoors<VertexType, EdgeType>::init()
		{
			
			/*
			* Build the first window
			*/
			//Clean up old results ;
			LineFollower::reset();
			
			//Get the first white point of the image
			int i = -1, j = -1;
			for(int row = 0 ; row < _map_in.rows ; row++){
				uchar* p = _map_in.ptr(row); //point to each row
				for(int col = 0 ; col < _map_in.cols ; col++){
					//p[j] <- how to access element
	// 				std::cout << (int)p[j]<< std::endl;
					if(p[col] > _value_of_white_min){
						i = col;
						j = row;
						
						//OUT
						col = _map_in.cols;
						row = _map_in.rows;
					}
				}
			}
			
	//  		std::cout << " Building the window around " << i << " " << j << std::endl;
			if(i != -1 && j !=-1){
				//Build a window around this point until the get at least 2 crossing so we can determine the direction.
				int radius_min_width = 0;
				int radius_min_height = 0;
				int radius_max_width = 0;
				int radius_max_height = 0;
				//Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
				//Rect got everything inversed. It needs a point with first dim as col and second as row
				int type = 0;
				
	// 			std::cout << _W << std::endl;
				
				while(type < 1){
					
					if( j - radius_min_height -1 >= 0){
						radius_min_height++;
					}
					if( j +radius_max_height +1 <= _map_in.size().height-1){
						radius_max_height++;
					}
					if( i - radius_min_width -1 >= 0){
						radius_min_width ++;
					}
					if( i +radius_max_width +1 <= _map_in.size().width-1){
						radius_max_width++;
					}
					
	//  			std::cout << " i and j " << i << " " <<j <<"values "<< i - radius_min_width << " " << j - radius_min_height << " " << i + radius_max_width << " " << j + radius_max_height << " and ma size " << _map_in.size() << std::endl;
					//Test new window
					_W = _map_in(cv::Rect( cv::Point(i - radius_min_width , j - radius_min_height), cv::Point( i + radius_max_width , j + radius_max_height ) ));
	//  				std::cout << "type : " << typeOfIntersection(_W) << std::endl;
					type = typeOfIntersection(_W);
					
	// 				std::cout << "Start : " << type <<std::endl;
	// 				cv::imshow("Test", _W);
	// 				cv::waitKey(0);
					
					if(_W.rows == _map_in.rows-1 && _W.cols == _map_in.cols-1){
						std::cout << "No line on the image " <<std::endl;
						throw std::runtime_error("No Line found in the init proccess everything is white");
					}
					
					
				}
				
			}
			
			else{
				std::cout << "No line on the image " <<std::endl;
				throw std::runtime_error("No Line found in the init proccess");
			}
			
			//Defined LP and RP
			//Only do the first and last col and row
			//doing all cols		
			
			//When we finally got etiher an intersection or a full line, we launch the algorithm
			
			
		}
		
		

		template<typename VertexType, typename EdgeType>
		inline void LineFollowerDoors<VertexType, EdgeType>::clear()
		{
			LineFollower::clear();
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
			_W.locateROI(s, p);
			int height = _W.rows;
			int width = _W.cols;
			
// 			std::cout << "trying the door detection with " << _doors.size() << std::endl;
			
			for(size_t i = 0 ; i < _doors.size() ; i = i+2){
// 				std::cout << " at : " << _doors[i].first << " at : " << _doors[i].second << std::endl;
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
#ifndef LINEFOLLOWERGRAPH_POINT_MAP_07032016
#define LINEFOLLOWERGRAPH_POINT_MAP_07032016

#include "LineFollower.hpp"
#include "bettergraph/PseudoGraph.hpp"


#include <bettergraph/PseudoGraph.hpp>
#include "SimpleNode.hpp"

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
		class LineFollowerGraph: public LineFollower{
			
			

		protected : 
// 			
			typedef bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Vertex Vertex;
			
			/// @brief deque of all parent vertex with line to explore.
			std::deque< Vertex > _dad_vertex;
			/// @brief Final graph
			bettergraph::PseudoGraph<SimpleNode, SimpleEdge> _graph;
// 			VertexMaker _vertex_maker;
// 			EdgeMaker _edge_maker;
			///@brief List of all position of corssings to detect the nodes
// 			std::deque<cv::Point2i> _all_crossings;	
			
			
			std::vector< std::pair<int, int> > _line;
			
			
		public:
			
			LineFollowerGraph() :LineFollower(){};
			virtual ~LineFollowerGraph(){
				//Clean up the vector of intersection
				reset();
			}

			
			virtual const bettergraph::PseudoGraph<SimpleNode, SimpleEdge>& getGraph() const {return _graph;}
			virtual bettergraph::PseudoGraph<SimpleNode, SimpleEdge> getGraph() {return _graph;}
			virtual void reset();
			
			void printIntersection(){
				for(size_t i = 0 ; i < _LRP_to_explore.size() ; i++){
					std::cout << " point 1 " << _LRP_to_explore[i].first << " point 2 " << _LRP_to_explore[i].second ;
					std::cout << " AT DAD " << _graph.getGraph()[_dad_vertex.at(i)].getX() << " " << _graph.getGraph()[_dad_vertex.at(i)].getY() << std::endl;
				}
			}
			
			/**
			* @brief line thining algorithm
			* 
			*/
			void thin();
			
			/**
			 * @brief return true if new_p is in _all_crossings*/
			bool loopDetection(cv::Point2i new_p, Vertex& dad_vertex);
			
			
		protected:
			void moveForward();
			void setDynamicWindow(cv::Mat& m){_W = m;}
			
			/**
			*@brief This function return the number of line crossing in a square.
			* 1 means a (at init : dead else line)
			* 2 means a (at init : line else T crossing)
			* more means a N crossing
			* 0 means that we are not completely on at least one of the branch
			* -1 means that there is no line at all
			* -2 means that we can only see white
			*/
// 			int typeOfIntersection(cv::Mat& roi);
			void init();
			
			/**
			 * @brief move the dynamic window forward and resize it to match the new anchor points
			 */
// 			void moveForward();
// 			void drawLine();
// 			bool findNextLPRP(std::vector< cv::Point2i >& all_points);
			void addPoint2Explore(const std::vector< cv::Point2i >& all_points, const Vertex& loop);
// 			void removeLineSegment(cv::Mat& c);
// 			void upResize();

			/**
			* @brief line thining algorithm after init.
			* 
			*/
			void lineThinningAlgo(Vertex& index_dad);
			
			/**
			* @brief return the mininmal distance between to point of input vector. 
			* 
			* @param[in] all_points : vector of cv::Point2i
			* @return minimum distance
			*/
			double calculateDistance(std::vector<cv::Point2i>& all_points);
			
			void addVertex(const Vertex& vertex_parent, Vertex& vertex_out){
// 				VertexType vtype = _vertex_maker.make(this);
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
// 				std::cout << "AT " << p_dyn_window.x << " " << p_dyn_window.y << std::endl;
// 				std::cout <<" dad " << _graph[vertex_parent].getX()<< " " << _graph[vertex_parent].getY() << std::endl;
				SimpleNode vtype;
				vtype.setX(p_dyn_window.x + (_W.size().width/2));
				vtype.setY(p_dyn_window.y + (_W.size().height/2));
				
				SimpleEdge sed;
				sed.setLine(_line);
				_line.clear();
				
				_graph.addVertex(vertex_out, vertex_parent, vtype, sed);
							
			}
			
			void addVertex(Vertex& vertex_out){
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
// 				std::cout << "AT " << p_dyn_window.x << " " << p_dyn_window.y << std::endl;
				SimpleNode vtype;
				vtype.setX(p_dyn_window.x + (_W.size().width/2));
				vtype.setY(p_dyn_window.y + (_W.size().height/2));
				_graph.addVertex(vertex_out, vtype);
			}
			
		};

		
		
		inline void LineFollowerGraph::thin()
		{
			try{
				cv::copyMakeBorder( _map_in, _map_in, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0 );
		// 		std::cout << "Init " << std::endl;
				//Build the first ROI
				init();
				//labelSegments();
			
		// 			std::cout << "out of init " << std::endl;
				
				//TODO need to be more global than that.
				//This find new points
				//Init the first drawing point
				cv::Point2i p;
				cv::Size s;
				_W.locateROI(s, p);
				
				_line.push_back(std::pair<int, int>(p.x, p.y));
				
				_last_drawing_point = cv::Point(p.x + (_W.rows/2), p.y + (_W.cols/2));
				
		// 			std::cout << "We start " << " at " << p.x << " " << p.y << " dyn wind " << _W.rows << " " << _W.cols << " draw oint " << _last_drawing_point << std::endl;
				
				std::vector<cv::Point2i> all_point;
				findNextLPRP(all_point);
				_LP = all_point[0];
				_RP = all_point[1];
				
				Vertex dad;
				
				cv::Mat m = _W.clone();
				//Is a dead end
				if(all_point.size() == 2){
// 					VertexType vtype = _vertex_maker.make(this);
					addVertex(dad);
					lineThinningAlgo(dad);
				}
				//Line
				else{
// 						VertexType vtype = _vertex_maker.make(this);
					addVertex(dad);
					addPoint2Explore(all_point, dad);
					lineThinningAlgo(dad);					
				}
			
			}
			catch(const std::exception &e){
				std::cout << "the unthinkable happened during voronoi landmark detection : " << e.what() << std::endl;
			}
		}

		
		inline void LineFollowerGraph::lineThinningAlgo(Vertex& index_dad)
		{
			
	// 		std::cout << "line Thining algo vrai" << std::endl;
			
	// 		printGraph();
	// 		printIntersection();
	// 		std::cout << "Value " << _graph.getGraph()[index_dad].point.x << std::endl;
			
	// 		int index = index_dad;
			Vertex dad_vertex = index_dad;
			while(_LP.x != -1 && _LP.y != -1){
				
				int type = typeOfIntersection(_W);
				//If we lost it we upsize W
				
				while(type == 0){
					upResize();
					type = typeOfIntersection(_W);
				}
				
	// 			std::cout << "RP : "<< _RP <<std::endl;
				
	// 			std::cout << "after while"<< " " << _LP.x << " " << _LP.y << " type " << type << std::endl;
	// 			std::cout << " W is this : " << _W << std::endl << " type " << type  << std::endl;
	// 			printIntersection();
				
				std::vector<cv::Point2i> all_point;
				bool non_dead_end = findNextLPRP(all_point);
				
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
				
				cv::Point2i new_p;
				new_p.x = p_dyn_window.x + (_W.cols / 2);
				new_p.y = p_dyn_window.y + (_W.rows / 2);
					
	// 			std::cout << "Size points is : " << all_point.size() << std::endl;
	// 			printIntersection();
	// 			std::cout << "And we are at LP " << _LP.x << " " << _LP.y << " RP "<< _RP.x << " " << _RP.y  << " type " << type << std::endl;
				if( all_point.size() > 2 ){
// 					std::cout << "HOY INTERSECTION : " << type << std::endl;
// 					std::cout <<" dad " << _graph[dad_vertex].getX()<< " " << _graph[dad_vertex].getY() << std::endl;
					
// 					cv::Mat m = _W.clone();
// 					cv::imshow("intersection", _W);
// 					cv::waitKey(0);

	// 				//Find all possible lines. this function check as well if we need to take a "large view" on the intersection (risk of false positive if the _W is small)
	// 				std::cout << "Loop detection in FIND NEXT LPRP" << std::endl;
// 					std::cout <<" dad " << _graph[dad_vertex].getX()<< " " << _graph[dad_vertex].getY() << std::endl;
					
					//USE : _all_crossings
					Vertex new_dad;
					bool already_exist = loopDetection(new_p, new_dad);
					
// 					std::cout <<" dad " << _graph[dad_vertex].getX()<< " " << _graph[dad_vertex].getY() << std::endl;
					
	// 				std::cout << "Before value" << std::endl;
	// 				std::cout << " valuuuues " << _graph.getGraph()[new_dad].point.y << std::endl;
					
// 					Vertex created = new_dad;
					//New intersection
						
					if(already_exist == false){
// 						std::cout <<" dad " << _graph[dad_vertex].getX()<< " " << _graph[dad_vertex].getY() << std::endl;
						addVertex(dad_vertex, new_dad);
					}
					//Not a new intersection but still an intersection
					else{
						if(new_dad != dad_vertex){
// 							boost::add_edge(loop_index, index , _graph);
							bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
							SimpleEdge sed;
							sed.setLine(_line);
							_line.clear();
							_graph.addEdge(ed, new_dad, dad_vertex, sed);
// 							printGraph();
// 							cv::waitKey(0);
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
	// 					std::cout << "size is : " << all_point.size() << std::endl;
	// 					std::cout << "end"<<std::endl;
						_LP.x = -1;
						_LP.y = -1;
					}
					
					//Needed to avoid an infinite loop :
					type = typeOfIntersection(_W);
							
	// 				cv::Mat maa_3 = _map_in.clone();
	// 				maa_3.setTo(cv::Scalar(0));
	// 				drawGraph(maa_3);
	// 				cv::imshow("graph", maa_3);
	// 				std::cout << "Printing the graph" << std::endl;
	// 				printGraph();
	// 				cv::waitKey(0);
					
					
				}
				else{				
					if(non_dead_end == false){
	// 					std::cout << "reach a dead end" << std::endl;
						cv::Mat m = _W.clone();

	// 					std::cout << "Loop detection in DEADEND" << std::endl;
						
						Vertex loop_vertex; 
						bool already_exist = loopDetection(new_p, loop_vertex);
												
						if(already_exist == false){
							addVertex(dad_vertex, loop_vertex);
						}
						else{
							
// 							std::cout << "not new" << std::endl;
							if(loop_vertex != dad_vertex){
	// 							boost::add_edge(loop_index, index , _graph);
								bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
								SimpleEdge sed;
								sed.setLine(_line);
								_line.clear();
								_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
	// 							printGraph();
	// 							cv::waitKey(0);
							}
							else{
	// 							std::cout << "Loop and dad are the same" << std::endl;
							}
						}
 					
						if(_LRP_to_explore.size() == 0){
	// 						std::cout << "Last dead end " << std::endl;
							_LP.x = -1;
							_LP.y = -1;
						}
						else{
							
	// 						printIntersection();
							
	// 						int a; std::cin >> a;
							
							_RP = _LRP_to_explore[0].first; 
							_LP = _LRP_to_explore[0].second;
	// 						index = _dads_index[0];
	// 						std::cout << "Move later " << std::endl;
							dad_vertex = _dad_vertex.at(0);
							_last_drawing_point = _last_drawing_point_deque[0];
							//Remove them
	// 						std::cout << "Popping LRP " << std::endl;
							_LRP_to_explore.pop_front();
	// 						std::cout << "Popping dads " << std::endl;
	// 						_dads_index.pop_front();
	// 						std::cout << "Popping Vertex " << std::endl;
							_dad_vertex.pop_front();
	// 						std::cout << "Popping drawing points " << std::endl;
							_last_drawing_point_deque.pop_front();
	// 						std::cout << "Move later " << std::endl;
							
							//Move the dynamic window
							moveForward();
	// 						std::cout << "Move later " << std::endl;
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
	// 							printGraph();
								dad_vertex = loop_vertex;
	// 							cv::waitKey(0);
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

		
		
		inline void LineFollowerGraph::init()
		{
			
			/*
			* Build the first window
			*/
			//Clean up old results ;
			LineFollowerGraph::reset();
			
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
// 					
// 					std::cout << "Start : " << type <<std::endl;
// 					cv::imshow("Test", _W);
// 					cv::waitKey(0);
					
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
		
		
		
		inline void LineFollowerGraph::addPoint2Explore(const std::vector< cv::Point2i >& all_points, const bettergraph::PseudoGraph< AASS::VoDiGrEx::SimpleNode, AASS::VoDiGrEx::SimpleEdge>::Vertex& loop)
		{
			//TODO probably don't need this
			if(all_points.size() >= 2){
				//index goes up
				//dad_index++;
				for(size_t i = 0 ; i < all_points.size() ; i=i+2){
					_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(all_points[i], all_points[i+1]));
					_dad_vertex.push_back(loop);
					_last_drawing_point_deque.push_back(_last_drawing_point);
					
				}
			}
		}


		
		
		inline void LineFollowerGraph::reset()
		{
			//reset Boost graph
			_graph.clear(); 
			_dad_vertex.clear();
			LineFollower::clear();
		}

		//To slow
	

		
		inline bool LineFollowerGraph::loopDetection(cv::Point2i new_p, AASS::VoDiGrEx::LineFollowerGraph::Vertex& dad_vertex)
		{
			for(size_t i = 0 ; i < _dad_vertex.size(); ++i){
				if(_graph[_dad_vertex[i]].getX() <= new_p.x + _marge &&
					_graph[_dad_vertex[i]].getX() >= new_p.x - _marge &&
					_graph[_dad_vertex[i]].getY() <= new_p.y + _marge &&
					_graph[_dad_vertex[i]].getY() >= new_p.y - _marge){
					dad_vertex = _dad_vertex[i];
					return true;
				}
			}
			return false;
		}


		inline void LineFollowerGraph::moveForward()
		{
			LineFollower::moveForward();
			cv::Point2i p;
			cv::Size s;
			_W.locateROI(s, p);
			_line.push_back(std::pair<int, int>(p.x + (_W.size().width/2), p.y + (_W.size().height/2)));

		}

	}
}

#endif
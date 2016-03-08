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
		template<typename VertexType = SimpleNode, typename EdgeType = SimpleEdge>
		class LineFollowerGraph: public LineFollower{
			
			

		protected : 
// 			
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex Vertex;
			
			/// @brief deque of all parent vertex with line to explore.
			std::deque< Vertex > _dad_vertex;
			/// @brief Final graph
			bettergraph::PseudoGraph<VertexType, EdgeType> _graph;
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
			void init();
			void addPoint2Explore(const std::vector< cv::Point2i >& all_points, const typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex& loop);

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
			
			void getNewBranch(Vertex& parent);
			
		};

#include "LineFollowerGraph.tpl"
		
		
		template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::thin()
{
	try{
		cv::copyMakeBorder( _map_in, _map_in, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0 );
// 		std::cout << "Init " << std::endl;
		//Build the first ROI
		init();
		cv::Point2i p;
		cv::Size s;
		_W.locateROI(s, p);
		_line.push_back(std::pair<int, int>(p.x, p.y));
		_last_drawing_point = cv::Point(p.x + (_W.rows/2), p.y + (_W.cols/2));
						
		std::vector<cv::Point2i> all_point;
		findNextLPRP(all_point);
		_LP = all_point[0];
		_RP = all_point[1];
		
		Vertex dad;
		
		cv::Mat m = _W.clone();
		//Is a dead end
		if(all_point.size() == 2){
			addVertex(dad);
			lineThinningAlgo(dad);
		}
		//Line
		else{
			addVertex(dad);
			addPoint2Explore(all_point, dad);
			lineThinningAlgo(dad);					
		}
	
	}
	catch(const std::exception &e){
		std::cout << "the unthinkable happened during voronoi landmark detection : " << e.what() << std::endl;
	}
}

template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::lineThinningAlgo(Vertex& index_dad)
{

	Vertex dad_vertex = index_dad;
	while(_LP.x != -1 && _LP.y != -1){

		std::vector<cv::Point2i> all_point;
		bool non_dead_end = findNextLPRP(all_point);
		
		cv::Size s;
		cv::Point2i p_dyn_window;
		_W.locateROI(s, p_dyn_window);
		
		cv::Point2i new_p;
		new_p.x = p_dyn_window.x + (_W.cols / 2);
		new_p.y = p_dyn_window.y + (_W.rows / 2);

		//Intersection or dead end
		if( all_point.size() > 2 || non_dead_end == false){
			
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
			getNewBranch(dad_vertex);
					
			
		}
		else{				

			//Making sure the line isn't actually a previsouly erased crossing.
			Vertex loop_vertex; 
			bool already_seen = loopDetection(new_p, loop_vertex);
			
			if(already_seen == true){
				if(loop_vertex != dad_vertex){
					bettergraph::PseudoGraph<SimpleNode, SimpleEdge>::Edge ed;
					SimpleEdge sed;
					sed.setLine(_line);
					_line.clear();
					_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
					dad_vertex = loop_vertex;
				}
			}
			
			_LP = all_point[0];
			_RP = all_point[1];
			moveForward();		
		}
	}
}


template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::init()
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

template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::getNewBranch(Vertex& parent)
{
	if(_dad_vertex.size() > 0){
		parent = _dad_vertex.at(0);
		_dad_vertex.pop_front();
	}
	LineFollower::getNewBranch();
}


template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::addPoint2Explore(const std::vector< cv::Point2i >& all_points, const typename bettergraph::PseudoGraph< VertexType, EdgeType>::Vertex& loop)
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



template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::reset()
{
	//reset Boost graph
	_graph.clear(); 
	_dad_vertex.clear();
	LineFollower::clear();
}

//To slow


template<typename VertexType, typename EdgeType>
inline bool LineFollowerGraph<VertexType, EdgeType>::loopDetection(cv::Point2i new_p, typename AASS::VoDiGrEx::LineFollowerGraph<VertexType, EdgeType>::Vertex& dad_vertex)
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

template<typename VertexType, typename EdgeType>
inline void LineFollowerGraph<VertexType, EdgeType>::moveForward()
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
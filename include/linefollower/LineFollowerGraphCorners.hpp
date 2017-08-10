#ifndef LINEFOLLOWERGRAPHCORNERS_POINT_MAP_21072016
#define LINEFOLLOWERGRAPHCORNERS_POINT_MAP_21072016

//TODO : humhum...
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "LineFollowerGraph.hpp"
#include "bettergraph/PseudoGraph.hpp"
#include "SimpleNode.hpp"
#include "utils/Utils.hpp"

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
		template<typename VertexType = SimpleNode, typename EdgeType = SimpleEdge>
		class LineFollowerGraphCorners: public LineFollowerGraph<VertexType, EdgeType>{

		protected : 
			typedef typename LineFollowerGraph<VertexType, EdgeType>::Vertex Vertex;
			cv::Vec2d _directional_vector;
			cv::Vec2d _original_direction;
// 			cv::Point2i _key_point_base;
			//Reference direction
			float _direction_base[2];
			//Keypoint used for creating the vector to compare to reference
			cv::Point2i _key_point;
			cv::Point2i _middle_point;
			//Flag to know if we just created a node and we need a new direction
			bool _direction_init;
			double _deviation_angle_in_rad;
			
			
			std::deque<cv::Mat> _last_Ws;
			int _max_distance_bounding_box;
			
			
			
			
		public:
			LineFollowerGraphCorners() : _deviation_angle_in_rad(M_PI / 4), _max_distance_bounding_box(20){};

// 			TODO : change name 
			///@brief Set the minimum deviation before a line because a corner. In Rad.
			void setMaxDeviation(double dev){_deviation_angle_in_rad = dev;};
			void setMinNumberOfBoundingBox(int min){assert(min > 3); _max_distance_bounding_box = min;}
			
			void print(){std::cout << "LF :-> deviation " << _deviation_angle_in_rad << " max dist bounding box " << _max_distance_bounding_box << std::endl; }
			
		protected:
			void lineThinningAlgo(Vertex& index_dad);
			bool isCorner(const std::vector< cv::Point2i >& all_point, cv::Point2f& corner_out);
			bool directionChanged(cv::Point2f& corner_out);
			Eigen::Vector3d collisionRay(const Eigen::Vector3d& ray_direction, const Eigen::Vector3d& ray_point, const Eigen::Vector3d& ray_direction_second, const Eigen::Vector3d& ray_point_second);
			
			
			
			bool testNoDuplicate(){
		
// 				std::cout << this->_all_time_dad_vertex.size() << "==" << this->_graph.getNumVertices() << std::endl;
				
// 				assert(this->_all_time_dad_vertex.size() == this->_graph.getNumVertices());
				
				std::pair< 
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::VertexIterator, 
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::VertexIterator 
				> vp;
				for (vp = boost::vertices(this->_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex v = *vp.first;
					
					std::pair< 
						typename bettergraph::PseudoGraph<VertexType, EdgeType>::VertexIterator, 
						typename bettergraph::PseudoGraph<VertexType, EdgeType>::VertexIterator 
					> vp2;
					for (vp2 = vp; vp2.first != vp2.second; ++vp2.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
						typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex v2 = *vp2.first;
						if(vp2 != vp){
// 							std::cout << "Not the same" << std::endl;
// 							std::cout << this->_graph[v].x <<"=="<< this->_graph[v2].x <<"&&"<< this->_graph[v].y <<" == "<< this->_graph[v2].y << std::endl;
							if (this->_graph[v].x == this->_graph[v2].x && this->_graph[v].y == this->_graph[v2].y){
								return false;
							}
						}
						else{
// 							std::cout << "SKIP" << std::endl;
						}
					}
				}
				
				return true;
			
			}
			
			
			
		};
		
		
		
		
		
	
		
		
		template<typename VertexType, typename EdgeType>
		inline void LineFollowerGraphCorners<VertexType, EdgeType>::lineThinningAlgo(Vertex& index_dad){
		// 	std::cout << "LINE THINING GRAPH" << std::endl;

			
			cv::Point2i p;
			cv::Size s;
			this->_W.locateROI(s, p);
			//new keypoint
			_key_point = p;
			_direction_init = true;
			
			Vertex dad_vertex = index_dad;
			while(this->_LP.x != -1 && this->_LP.y != -1){

				
				assert(testNoDuplicate() == true);
				
				std::vector<cv::Point2i> all_point;
				bool non_dead_end = this->findNextLPRP(all_point);
				
				bool iscorner = false;
				cv::Point2f corner_turn(-1, -1);
				if(all_point.size() == 2){
					iscorner = isCorner(all_point, corner_turn);
				}
				
				cv::Size s;
				cv::Point2i p_dyn_window;
				this->_W.locateROI(s, p_dyn_window);
				
				cv::Point2i new_p;
				new_p.x = p_dyn_window.x + (this->_W.cols / 2);
				new_p.y = p_dyn_window.y + (this->_W.rows / 2);
				
		// #ifdef DEBUG
		// 		cv::Mat print;
		// 		this->_map_in.copyTo(print);
		// // 		cv::Size s;
		// // 		cv::Point2i p_dyn_window;
		// // 		_W.locateROI(s, p_dyn_window);
		// 		cv::Point2i second_point = p_dyn_window;
		// 		second_point.x = second_point.x + _W.size().width;
		// 		second_point.y = second_point.y + _W.size().height;
		// 		cv::rectangle( print, p_dyn_window, second_point, cv::Scalar( 255 ), 1, 4 );
		// 		cv::imshow("tmp", print);
		// 		cv::waitKey(1);
		// #endif

				//Intersection or dead end or corner
				if( all_point.size() > 2 || non_dead_end == false || iscorner == true){
					
// 					cv::waitKey(0);
					
					Vertex new_dad;
					//I used just a certain number of move but it should use the number of move + the distance travelled by the edge for more robustness. To avoid useless self loops, the number of move needs to be above a certain threshold. Thanks to nature of the algorithm and the moveForward function, it _should_ be ok, by just considering that the bounding box needs to do at all least one jump forward.
					bool already_exist = this->loopDetection(new_p, new_dad);
					
// 					std::cout << "Testing existance of " << new_p << " : " << already_exist << std::endl;

					//New intersection
					if(already_exist == false){
						if(iscorner == true){
							assert(corner_turn.x != -1);
							assert(corner_turn.y != -1);
// 							std::cout << "Corner ! " << corner_turn << std::endl;;
							this->addVertex(dad_vertex, new_dad, corner_turn);
							_last_Ws.clear();
							
						}
						else{
// 							std::cout << "not a Corner ! " << std::endl;;
							this->addVertex(dad_vertex, new_dad);
							_last_Ws.clear();
						}
						
						//TODO : remove the test
						
					}
					//Not a new intersection but still an intersection
					else{
						//Should be allowed to have self loop since it's a pseudo graph
						//I used just a certain number of move but it should use the number of move + the distance travelled by the edge for more robustness. To avoid useless self loops, the number of move needs to be above a certain threshold. Thanks to nature of the algorithm and the moveForward function, it _should_ be ok, by just considering that the bounding box needs to do at all least one jump forward.
						if(this->_line.size() > 1){
							typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
							EdgeType sed;
							sed.setLine(this->_line);
							this->_graph.addEdge(ed, new_dad, dad_vertex, sed);
						}
		// 				std::cout << "Clear" << std::endl;
						this->_line.clear();
						_last_Ws.clear();
					}
					
					this->addPoint2Explore(all_point, new_dad);
					this->getNewBranch(dad_vertex);
					
					assert(testNoDuplicate() == true);
					
					cv::Point2i p;
					cv::Size s;
					this->_W.locateROI(s, p);
					//new keypoint
					_key_point = p;
					_direction_init = true;
					
		// #ifdef DEBUG
// 				cv::Mat print;
// 				this->_map_in.copyTo(print);
// 				cv::Size s2;
// 				cv::Point2i p_dyn_window2;
// 				this->_W.locateROI(s2, p_dyn_window2);
// 				cv::Point2i second_point = p_dyn_window2;
// 				second_point.x = second_point.x + this->_W.size().width;
// 				second_point.y = second_point.y + this->_W.size().height;
// 				cv::rectangle( print, p_dyn_window2, second_point, cv::Scalar( 255 ), 1, 4 );
// 				cv::imshow("tmp", print);
// 				
// 				bettergraph::PseudoGraph<VertexType, EdgeType> graph = this->getGraph();
// 				cv::Mat maa_3 = this->_map_in.clone();
// 				maa_3.setTo(cv::Scalar(0));
// 				AASS::vodigrex::draw<VertexType, EdgeType>(graph, maa_3);
// 				cv::imshow("tmp graph", maa_3);
// 				
// 				cv::waitKey(0);
		// #endif
							
					
				}
				else{				

					//Making sure the line isn't actually a previsouly erased crossing.
					Vertex loop_vertex; 
					bool already_seen = this->loopDetection(new_p, loop_vertex);
					
					if(already_seen == true){
						//not zero because after switching branch we always move forward
						if(/*loop_vertex != dad_vertex*/ this->_line.size() > 1 ){
							typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
							EdgeType sed;
							sed.setLine(this->_line);
							this->_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
						}
		// 				std::cout << "clear " << std::endl;
						this->_line.clear();
						dad_vertex = loop_vertex;
					}
					
					this->_LP = all_point[0];
					this->_RP = all_point[1];
					this->moveForward();		
				}
			}
		}
		
		template<typename VertexType, typename EdgeType>
		inline bool LineFollowerGraphCorners<VertexType, EdgeType>::isCorner(const std::vector< cv::Point2i >& all_point, cv::Point2f& corner_out)
		{
			
			cv::Point2i p;
			cv::Size s;
			this->_W.locateROI(s, p);
			
// // 			p.x = p.x + (s.width/2);
// // 			p.y = p.y + (s.height/2);
// 			
// 			cv::imshow("Dyn", this->_W);
// 			cv::waitKey(0);
// 			
// 			float a[2] = {this->_LP.x - p.x, this->_LP.y - p.y};
// 			float b[2] = {all_point[0].x - p.x, all_point[0].y - p.y};
// 			
// 			float a_norm = std::sqrt(a[0]*a[0] + a[1]*a[1]);
// 			float b_norm = std::sqrt(b[0]*b[0] + b[1]*b[1]);
// 
// 			cv::Mat AA(1,2,CV_32FC1,a);
// 			cv::Mat BB(1,2,CV_32FC1,b);
// // 			angle 
// 			double d = AA.dot(BB);
// 			double l = a_norm * b_norm;			
// 			
// 			std::cout <<" Center : " << p << " points " <<this->_LP << " " << all_point[0] << " Size " << this->_W.size()  <<std::endl;
// 
// 			std::cout <<" Center : " << p << " points " << a[0] << ":" << a[1] << " " << b[0] << ":" << b[1] <<std::endl;
// 			std::cout << "Norms a b " << a_norm << " " << b_norm << std::endl;
// 			std::cout << " d : " << d << " norm " << l << std::endl;
// 			double ac = d/l;
// 			ac = std::acos(ac);
// 			std::cout << "Angle : "<< ac << " conmpare to " << (M_PI  / 2) << std::endl;
// 			if(ac >= (M_PI / 2) - 0.1 && ac < (M_PI  / 2) + 0.1) return true;
// 			return false;
			
			
			return directionChanged(corner_out);
		}
	
	
		template<typename VertexType, typename EdgeType>
		inline bool LineFollowerGraphCorners<VertexType, EdgeType>::directionChanged(cv::Point2f& corner_out){
			_last_Ws.push_back(this->_W);
			
			assert(_max_distance_bounding_box >= 0);
			
			if(_last_Ws.size() > (unsigned int) _max_distance_bounding_box){
				
				_last_Ws.pop_front();
				
				//Localise 3 bounding boxes
				
				cv::Point2i p;
				cv::Size s;
				this->_W.locateROI(s, p);
				
				cv::Point2i p_end;
				cv::Size s_end;
				_last_Ws[0].locateROI(s_end, p_end);
				
				cv::Point2i p_middle;
				cv::Size s_middle;
				_last_Ws[(_last_Ws.size()/2) - 1].locateROI(s_middle, p_middle);
				
				
				
				//////////////////////////////DEBUG draw
				
// 				cv::Mat line;
// 				this->_map_in.copyTo(line);
// // 				line.setTo(cv::Scalar(0));
// 				cv::Point pp; pp.x = p.x + this->_W.rows; pp.y = p.y + this->_W.cols;
// 				cv::rectangle(line, p, pp, cv::Scalar(100, 50, 0), 5);
// 				
// 				cv::Point ppp; ppp.x = p_middle.x + _last_Ws[(_last_Ws.size()/2) - 1].rows; ppp.y = p_middle.y + _last_Ws[(_last_Ws.size()/2) - 1].cols;
// 				cv::rectangle(line, p_middle, ppp, cv::Scalar(100, 50, 0), 5);
// 				
// 				cv::Point pppp; pppp.x = p_end.x + _last_Ws[0].rows; pppp.y = p_end.y + _last_Ws[0].cols;
// 				cv::rectangle(line, p_end, pppp, cv::Scalar(100, 50, 0), 5);
// 				
// // 				bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> graph11 = llll_3.getGraph(i);
// 		
// 				AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(this->_graph, line);
// 				
// 				cv::imshow("tmp, lines", line);
// 				cv::waitKey(1);
// 				
				////////////////////////////////////
				
				float first_direction[2]; 
				first_direction[0] = p_middle.x - p_end.x; 
				first_direction[1] = p_middle.y - p_end.y;
				
				float second_direction[2]; 
				second_direction[0] = p.x - p_middle.x;
				second_direction[1] = p.y - p_middle.y;
				
				float direction_norm = std::sqrt( ( first_direction[0] * first_direction[0] ) + ( first_direction[1] * first_direction[1]) );
				
				//Reference vector
				float second_direction_norm = std::sqrt( ( second_direction[0] * second_direction[0] ) + ( second_direction[1] * second_direction[1] ) );
				
				first_direction[0] = first_direction[0] / direction_norm;
				first_direction[1] = first_direction[1] / direction_norm;
				second_direction[1] = second_direction[1] / second_direction_norm;
				second_direction[0] = second_direction[0] / second_direction_norm;

				cv::Mat AA(1, 2, CV_32FC1, first_direction);
				cv::Mat BB(1, 2, CV_32FC1, second_direction);
				
				double d = AA.dot(BB);
// 				double l = direction_norm * second_direction_norm;	
				
				//Truncating
// 				std::cout << "Value before " <<  d  << std::endl;
				d = d * 1000;
// 				std::cout << "Value before " <<  d  << std::endl;
				int d_temp = d;
// 				std::cout << "Value before " <<  d_temp  << std::endl;
				d = d_temp;
// 				std::cout << "Value before " <<  d  << std::endl;
				d = d / 1000;
// 				int l_temp = l * 4;
// 				l = l_temp;
				
// 				std::cout <<" Center : " << p << " points " <<this->_LP << " " << all_point[0] << " Size " << this->_W.size()  <<std::endl;
// 				std::cout <<" Center : " << p << " points " << _direction_base[0] << ":" << _direction_base[1] << " " << b[0] << ":" << b[1] << " distance " << distance <<std::endl;
// 				std::cout << "Norms a b " << direction_norm << " " << b_norm << std::endl;
// 				std::cout << " d : " << d << " norm " << l << std::endl;
// 				double ac = d/l;
				
				double ac = d;
// 				std::cout << "Value before acos " << ac << " " << d  << std::endl;
				assert(ac<=1); assert(ac >= -1);
				//Angle between reference direction and old direction
				ac = std::acos(ac);
// 				std::cout << ac << " " << d << std::endl;
				assert(ac >= 0);
				assert(ac <= M_PI);

// 				if(ac <= (M_PI  / 2) + 0.25 && ac > (M_PI  / 2) - 0.25){
				//If the line change by more than 25deg.
				if(ac >= _deviation_angle_in_rad){
					corner_out = p_middle;
					return true;
					
					
					
				}
				
				return false;
				
			}

			return false;

		}
		
		template<typename VertexType, typename EdgeType>
		inline Eigen::Vector3d LineFollowerGraphCorners<VertexType, EdgeType>::collisionRay(
			const Eigen::Vector3d& ray_direction, 
			const Eigen::Vector3d& ray_point, 
			const Eigen::Vector3d& ray_direction_second, 
			const Eigen::Vector3d& ray_point_second){
			
			Eigen::Matrix3d A ;//THree rows and 2 cols ;
			Eigen::Vector3d b;
			
			A << - ray_direction(0), ray_direction_second(0), 0,
				- ray_direction(1), ray_direction_second(1), 0,
				- ray_direction(2), ray_direction_second(2), 0;
			b << ray_point(0) - ray_point_second(0),
				ray_point(1) - ray_point_second(1),
				ray_point(2) - ray_point_second(2);
				
// 			std::cout << "Here is the matrix A:\n" << A << std::endl;
// 			std::cout << "Here is the vector b:\n" << b << std::endl;
			Eigen::Vector3d t_n = A.colPivHouseholderQr().solve(b);
// 			std::cout << "The solution is:\n" << t_n << std::endl;
			
			//Calculate collision point using running parameter t_n
			Eigen::Vector3d x;
			x << std::floor( (( ray_direction(0) * t_n(0) ) + ray_point(0) ) * 10 + 0.5)/10,
				 std::floor( (( ray_direction(1) * t_n(0) ) + ray_point(1) ) * 10 + 0.5)/10,
				 std::floor( (( ray_direction(2) * t_n(0) ) + ray_point(2) ) * 10 + 0.5)/10;
				 
			Eigen::Vector3d x_second;
			x_second << std::floor( (( ray_direction_second(0) * t_n(1) ) + ray_point_second(0)) * 10 + 0.5)/10,
						std::floor( (( ray_direction_second(1) * t_n(1) ) + ray_point_second(1)) * 10 + 0.5)/10,
						std::floor( (( ray_direction_second(2) * t_n(1) ) + ray_point_second(2)) * 10 + 0.5)/10;
				 
// 			std::cout << x << " and " << x_second << std::endl;
			
			//TODO: Make this test better :S
// 			assert(x == x_second);
				 
			return x;
			
		}
		
		
	}
}

#endif
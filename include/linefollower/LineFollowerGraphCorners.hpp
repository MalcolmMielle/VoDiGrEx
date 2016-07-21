#ifndef LINEFOLLOWERGRAPHCORNERS_POINT_MAP_21072016
#define LINEFOLLOWERGRAPHCORNERS_POINT_MAP_21072016

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
			//Flag to know if we just created a node and we need a new direction
			bool _direction_init;
			
			
		public:
			LineFollowerGraphCorners(){};
			
			
		protected:
			void lineThinningAlgo(Vertex& index_dad);
			bool isCorner(const std::vector<cv::Point2i>& all_point);
			bool directionChanged();
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

				std::vector<cv::Point2i> all_point;
				bool non_dead_end = this->findNextLPRP(all_point);
				
				bool iscorner = false;
				if(all_point.size() == 2){
					iscorner = isCorner(all_point);
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
					Vertex new_dad;
					//I used just a certain number of move but it should use the number of move + the distance travelled by the edge for more robustness. To avoid useless self loops, the number of move needs to be above a certain threshold. Thanks to nature of the algorithm and the moveForward function, it _should_ be ok, by just considering that the bounding box needs to do at all least one jump forward.
					bool already_exist = this->loopDetection(new_p, new_dad);

					//New intersection
					if(already_exist == false){
						this->addVertex(dad_vertex, new_dad);
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
					}
					
					this->addPoint2Explore(all_point, new_dad);
					this->getNewBranch(dad_vertex);
					
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
		inline bool LineFollowerGraphCorners<VertexType, EdgeType>::isCorner(const std::vector< cv::Point2i >& all_point)
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
			
			
			bool changed = directionChanged();
			if(changed == true){
				return true;
			}

		}
	
	
		template<typename VertexType, typename EdgeType>
		inline bool LineFollowerGraphCorners<VertexType, EdgeType>::directionChanged(){
			
			cv::Point2i p;
			cv::Size s;
			this->_W.locateROI(s, p);
			double firsty = (p.x - _key_point.x)*(p.x - _key_point.x);
			double secondy = (p.y - _key_point.y)*(p.y - _key_point.y);
// 			std::cout << "fand s : " <<firsty << " and " << secondy << std::endl;
			int distance = std::sqrt(firsty + secondy);
			
// 			std::cout << " distance " << distance <<std::endl;
// 			std::cout <<" Center< : " << p << " keypoint" << _key_point << std::endl;
			
			//If it needs to be initialized
			if(distance > 5 && _direction_init == true){
// 				std::cout << " INIT" << std::endl;
				_direction_base[0] = p.x - _key_point.x;
				_direction_base[1] = p.y - _key_point.y;
				_direction_init = false;
// // 				std::cout <<" Center< : " << p << " keypoint" << _key_point << " points " << _direction_base[0] << ":" << _direction_base[1] << " distance " << distance <<std::endl;
// 				int a;
// 				std::cin >> a;
			}
			//Not too short;
			if(distance > 5){
				
				
// 				cv::Mat print;
// 				this->_map_in.copyTo(print);
// 				cv::Size s2;
// 				cv::Point2i p_dyn_window2;
// 				this->_W.locateROI(s2, p_dyn_window2);
// 				cv::Point2i second_point = p_dyn_window2;
// 				second_point.x = second_point.x + this->_W.size().width;
// 				second_point.y = second_point.y + this->_W.size().height;
// 				cv::rectangle( print, p_dyn_window2, second_point, cv::Scalar( 255 ), 1, 4 );
// 				cv::circle(print, _key_point, 5, cv::Scalar(200), 3);
// 				cv::Point2i tmp;
// 				
// 				cv::circle(print, p, 5, cv::Scalar(255), 3);
// 				cv::imshow("tmp", print);
// 				
// 				bettergraph::PseudoGraph<VertexType, EdgeType> graph = this->getGraph();
// 				cv::Mat maa_3 = this->_map_in.clone();
// 				maa_3.setTo(cv::Scalar(0));
// 				AASS::vodigrex::draw<VertexType, EdgeType>(graph, maa_3);
// 				cv::imshow("tmp CORNER", maa_3);
// 				
// 				cv::waitKey(0);
				
				
// 				cv::imshow("Dyn", this->_W);
// 				cv::waitKey(0);
				
				//Current vector
				float b[2] = {p.x - _key_point.x, p.y - _key_point.y};
				//Reference vector
				float direction_norm = std::sqrt(_direction_base[0]*_direction_base[0] + _direction_base[1]*_direction_base[1]);
				float b_norm = std::sqrt(b[0]*b[0] + b[1]*b[1]);

				cv::Mat AA(1,2,CV_32FC1,_direction_base);
				cv::Mat BB(1,2,CV_32FC1,b);
				
				double d = AA.dot(BB);
				double l = direction_norm * b_norm;			
				
// 				std::cout <<" Center : " << p << " points " <<this->_LP << " " << all_point[0] << " Size " << this->_W.size()  <<std::endl;

// 				std::cout <<" Center : " << p << " points " << _direction_base[0] << ":" << _direction_base[1] << " " << b[0] << ":" << b[1] << " distance " << distance <<std::endl;
// 				std::cout << "Norms a b " << direction_norm << " " << b_norm << std::endl;
// 				std::cout << " d : " << d << " norm " << l << std::endl;
				double ac = d/l;
				//Angle between reference direction and old direction
				ac = std::acos(ac);
// 				std::cout << "Angle : "<< ac << " conmpare to " << (M_PI  / 2) << std::endl;
				
				//If to long distance: update the keypoint so the vector we compare grow forward
				if(distance > 20){
// 					_direction_base[0] = {p.x - _direction_base[0], p.y - _direction_base[1]};
// 					_direction_base[1] = {p.x - _direction_base[0], p.y - _direction_base[1]};
					_key_point = p;
// 					std::cout <<" New keypoint : " << p << " keypoint" << _key_point <<std::endl;
// 					int a;
// 					std::cin >> a;
				}
				
				if(ac >= 0.5/* && ac < (M_PI  / 2) + 0.1*/){
// 					int a;
// 					std::cin >> a;
// 					std::cout << "Return true" << std::endl;
					return true;
				}
				return false;
				}
			return false;

		}
	}
}

#endif
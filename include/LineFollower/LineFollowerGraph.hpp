#ifndef LINEFOLLOWERGRAPH_POINT_MAP_07032016
#define LINEFOLLOWERGRAPH_POINT_MAP_07032016

#include "LineFollower.hpp"
#include "bettergraph/PseudoGraph.hpp"
#include "SimpleNode.hpp"
#include "Utils/Utils.hpp"

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
		class LineFollowerGraph: public LineFollower{

		protected : 
 			
			typedef typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex Vertex;
			
			/// @brief deque of all parent vertex with line to explore. The same vertex is inputted here as much time as it has exit branches
			std::deque< Vertex > _dad_vertex;
			/// @brief Final graph
			bettergraph::PseudoGraph<VertexType, EdgeType> _graph;
			/// @brief Every two point closer than _marge are to be fused as one.
			int _marge;
			/// @brief Edge points
			std::vector< std::pair<int, int> > _line;
			
			
		public:
			
			LineFollowerGraph() :LineFollower(), _marge(10){};
			virtual ~LineFollowerGraph(){
				//Clean up the vector of intersection
				clear();
			}
			
			const bettergraph::PseudoGraph<VertexType, EdgeType>& getGraph() const {return _graph;}
			bettergraph::PseudoGraph<VertexType, EdgeType> getGraph() {return _graph;}
			virtual void clear();
			
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
			virtual void thin();
			/**
			 *@brief set the minimum distance between two node*/
			void setMarge(int m){_marge = m;}
			int getMarge(){return _marge;}
			
			
			
		protected:
			void moveForward();
			void addPoint2Explore(const std::vector< cv::Point2i >& all_points, const typename bettergraph::PseudoGraph<VertexType, EdgeType>::Vertex& loop);
			/**
			 * @brief return true if new_p is in _all_crossings*/
			bool loopDetection(cv::Point2i new_p, Vertex& dad_vertex);

			/**
			* @brief line thining algorithm after init.
			* 
			*/
			virtual void lineThinningAlgo(Vertex& index_dad);
			
			//TODO vertex_out should be consistent
			void addVertex(const Vertex& vertex_parent, Vertex& vertex_out){
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
				VertexType vtype;
				vtype.setX(p_dyn_window.x + (_W.size().width/2));
				vtype.setY(p_dyn_window.y + (_W.size().height/2));
				
				EdgeType sed;
				sed.setLine(_line);
				_line.clear();
				
				_graph.addVertex(vertex_out, vertex_parent, vtype, sed);
							
			}
			
			void addVertex(Vertex& vertex_out){
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
				VertexType vtype;
				vtype.setX(p_dyn_window.x + (_W.size().width/2));
				vtype.setY(p_dyn_window.y + (_W.size().height/2));
				_graph.addVertex(vertex_out, vtype);
			}
			
			virtual void getNewBranch(Vertex& parent);
			
		};

#include "LineFollowerGraph.tpl"

	}
}

#endif
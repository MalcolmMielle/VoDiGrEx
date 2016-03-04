#ifndef GRAPHLIST_LINEFOLLOWER_POINT_MAP
#define GRAPHLIST_LINEFOLLOWER_POINT_MAP

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <stdexcept>
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"

#include <assert.h>

#include <opencv2/opencv.hpp>

#include "Global.hpp"
	
namespace AASS{
		
	namespace VoDiGrEx{
		
		/**
		* @brief Graph structure for line extraction
		* 
		* Vertices type is topologicalmap::Intersection_Graph
		* 
		* Edge type is topologicalmap::EdgeElement
		* 
		*/
		
		class GraphList{
		protected :
			//Define the marge for the loop detection
			int _marge;
			Graph_boost _graph;
			
		public :
			GraphList() : _marge(10) {};
			
			virtual void setMarge(int m){_marge = m;};
			virtual int getNumVertices() const {return boost::num_vertices(_graph);}
			virtual int getNumEdges() const {return boost::num_edges(_graph);}
			virtual int getNumEdges(Vertex v) const;
			
			virtual const Graph_boost& getGraph() const {return _graph;}
			virtual void addEdge(Vertex& loop_index, Vertex& index){
				
	// 			print();
				
				bool exist = boost::edge(loop_index, index , _graph).second;
				
				//ATTENTION : WORKAROUND -> with the really big graph sometime line follower ask to link twice two edge. My guess is the problem is something like they are too close and thus, it feels like they are explored twice :
				/* Find Vertex 0
				* Add vertex 0
				* Find Vertex 1
				* Vertex 1 is going to Vertex 2
				* Vertex 2 is actually Vertex 3
				* Loop detection wasn't made correctly and it was linked twice
				* 
				* It's pretty hard to debug
				*/
				
				if(exist == true){
					std::cout << "EDGE ALREADY EXIST. Didnt add anything." << std::endl;

				}
				else{
					boost::add_edge(loop_index, index , _graph).first;
	// 				 _graph[e].number = 0;
				}
				
				
			}
			virtual void addVertex(std::string& type, const cv::Mat& m, cv::Point2i p, Vertex& vertex_out, const Vertex& dad);
			virtual void addVertex(std::string& type, const cv::Mat& m, cv::Point2i p, Vertex& vertex_out);
			virtual void removeVertex(Vertex& v);
			virtual void print() const;
			virtual void print(const Vertex& v) const;
			virtual void draw(cv::Mat& m);
			virtual void draw(const Vertex& v, cv::Mat& m);
			virtual bool loopDetection(const cv::Point2i& p, const Vertex& dad, Vertex& vertex_out);
			virtual void clear(){_graph.clear();}
			
			/** @brief Scale every vertex position by a certain factor
			* 
			* @param[in] scale : scale factor
			*/
			virtual void scale(double scale);
			/// @brief Return the euclidean distance between two vertex
			virtual float distanceSquared(const Vertex& v, const Vertex& v2);
			virtual void getPoint(const Vertex& v, cv::Point& p) const;
			virtual void setPoint(Vertex& v, const cv::Point& p){_graph[v].point = p;}
			/// @brief Return all vertices linked to a vertex
			virtual void getAllVertexLinked(const topologicalmap::Vertex& v, std::deque< topologicalmap::Vertex >& all_vertex);
			/// @brief Return all edges and vertices linked to a vertex
			virtual void getAllEdgeLinked(const topologicalmap::Vertex& v, std::deque< std::pair< topologicalmap::Edge, topologicalmap::Vertex > >& all_edge) const;
			
			virtual Intersection_Graph& operator[](const Vertex& v);
			virtual const Intersection_Graph& operator[] (const Vertex& v) const ;
			/// @brief Fuse v2 and v1 into one vertex with both attributes.
			virtual void fuse(topologicalmap::Vertex& v, topologicalmap::Vertex v_to_remove);
			
			
		};
		
		
		
		
		inline void GraphList::addVertex(std::string& type, const cv::Mat& m, cv::Point2i p, Vertex& vertex_out, const Vertex& dad)
		{
			addVertex(type, m, p, vertex_out);
			boost::add_edge(vertex_out, dad, _graph).first;
		}
		
		inline void GraphList::addVertex(std::string& type, const cv::Mat& m, cv::Point2i p, Vertex& vertex_out)
		{
			vertex_out = boost::add_vertex(_graph);
			int num_vertice = boost::num_vertices(_graph);
			_graph[vertex_out].type = type;
			_graph[vertex_out].mat = m;
			_graph[vertex_out].point = p;
			_graph[vertex_out].index = num_vertice - 1;
			_graph[vertex_out].force = 1;
		}

		
		inline void GraphList::print() const
		{
			//first is beginning, second is "past the end"
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
				Vertex v = *vp.first;
				
				print(v);
				
			}
	// 		std::cout << std::endl;
		}
		inline void GraphList::draw(const topologicalmap::Vertex& v, cv::Mat& m)
		{
			cv::Scalar color;
			if(m.channels() == 1){
				color = 255;
			}
			else if(m.channels() == 3){
				color[1] = 255;
				color[2] = 255;
				color[3] = 255; 
			}
			cv::circle(m, _graph[v].point, 5, color);
				
		}

		
		inline void GraphList::draw(cv::Mat& m)
		{
			
			std::cout << "DRAW" << std::endl;
			
			cv::Scalar color;
			if(m.channels() == 1){
				color = 255;
			}
			else if(m.channels() == 3){
				color[1] = 255;
				color[2] = 255;
				color[3] = 255; 
			}
			//first is beginning, second is "past the end"
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
				
	// 			cv::Mat tmp = cv::Mat::zeros(m.size(), CV_8UC1);
				
	// 			std::cout << "NEW VERTEX" << std::endl;
				
				Vertex v = *vp.first;
				cv::circle(m, _graph[v].point, 5, color);
	// 			cv::circle(tmp, _graph[v].point, 5, color);
	// 
				EdgeIterator out_i, out_end;
				Edge e;
				
				for (boost::tie(out_i, out_end) = boost::out_edges(v, _graph); 
					out_i != out_end; ++out_i) {
	// 				std::cout << "NEW Edge" << std::endl;
					e = *out_i;
					Vertex src = boost::source(e, _graph), targ = boost::target(e, _graph);
					cv::line(m, _graph[src].point, _graph[targ].point, color);
	// 				cv::line(tmp, _graph[src].point, _graph[targ].point, color);
				
	// 				cv::imshow("the graph being made", m);
	// 				cv::imshow("the graph partial being made", tmp);
	// 				cv::waitKey(0);
				}
				
			}
		}
		
		inline bool GraphList::loopDetection(const cv::Point2i& p, const topologicalmap::Vertex& dad, topologicalmap::Vertex& vertex_out)
		{
			//Handle loops
			//loop detection
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
	// 		std::cout << "Marge : " << _marge << std::endl; 
			for (vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
				Vertex v = *vp.first;
				
	// 			std::cout << "comparing " << p << " to " << _graph[v].point << std::endl;
				
				if(_graph[v].point.x  < p.x + _marge && _graph[v].point.x > p.x - _marge){
					if(_graph[v].point.y  < p.y + _marge && _graph[v].point.y > p.y - _marge){
	// 					std::cout << "same point ! " <<_graph[v].index << " "<< dad_index << " at " << p.x << " " <<p.y << std::endl;
						//TODO use boost graph index method instead
	// 					std::cout << "SAME so it's a loop" << std::endl;
						vertex_out = v;
						return  false;
									
					}	
				}
			}
	// 		std::cout << " NOT SAME returning dad" << std::endl;
			vertex_out = dad;
			return true;
		}
		
		
		inline void GraphList::scale(double scale)
		{
				
			//first is beginning, second is "past the end"
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
				Vertex v = *vp.first;
	// 			std::cout << "before " << _graph[v].point;
				_graph[v].point = _graph[v].point * scale;
	// 			std::cout << " after " << _graph[v].point << std::endl;
			}
		}


		inline void GraphList::getPoint(const topologicalmap::Vertex& v, cv::Point& p) const
		{
			p = _graph[v].point;
		}
		
		inline void GraphList::getAllVertexLinked(const topologicalmap::Vertex& v, std::deque<Vertex>& all_vertex)
		{
			
			EdgeIterator out_i, out_end;
			Edge e;
	// 		std::cout << "ALL THE LINKED VERTEX" << std::endl;
			for (boost::tie(out_i, out_end) = boost::out_edges(v, _graph); 
				out_i != out_end; ++out_i) {
				e = *out_i;
				Vertex targ = boost::target(e, _graph);
				
	// 			print(targ);
				all_vertex.push_back(targ);
			}
		}
		
		inline void GraphList::getAllEdgeLinked(const topologicalmap::Vertex& v, std::deque< std::pair <topologicalmap::Edge, topologicalmap::Vertex> >& all_edge) const
		{
			EdgeIterator out_i, out_end;
			Edge e;
			
			for (boost::tie(out_i, out_end) = boost::out_edges(v, _graph); 
				out_i != out_end; ++out_i) {
				e = *out_i;
				Vertex targ = boost::target(e, _graph);
			
			
				all_edge.push_back(std::pair<topologicalmap::Edge, topologicalmap::Vertex> (e, targ) );
			}
		}



		inline void GraphList::removeVertex(topologicalmap::Vertex& v)
		{
	// 		//Need to remove the edges as well
	// 		EdgeIterator out_i, out_end, out_i_first, out_end_first, out_t;
	// 		Edge e;
	// 		
	// 		boost::tie(out_i_first, out_end_first) = boost::out_edges(v, _graph);
	// 		bool flag = false;
	// 		
	// 		for (boost::tie(out_i, out_end) = boost::out_edges(v, _graph); 
	// 			out_i != out_end; ) {
	// 			
	// 			out_t = out_i;
	// 			e = *out_i; // <- got to the next one when suppressed.
	// 			Vertex src = boost::source(e, _graph), targ = boost::target(e, _graph);
	// // 			std::cout << "remove edge" << std::endl;
	// 			boost::remove_edge(src, targ, _graph);
	// 
	// 			
	// 			out_i++;
	// 			if(flag == true){
	// // 				std::cout << "FLAG detected " << std::endl;
	// 				out_i = out_end;
	// 			}
	// 			else if(flag == false && out_i == out_end_first){
	// // 				std::cout << "FLAG " << std::endl;
	// 				flag = true;
	// 				out_i--;
	// 			}
	// 			else if(flag == false && out_i != out_i_first){
	// // 				std::cout << "go back " << std::endl;
	// 				out_i--;
	// 			}				
	// 		}
			
			//Same as all the code before
			boost::clear_vertex(v, _graph);
			
	// 		int degree_out = boost::out_degree(v, _graph);
	// 		int degree_in = boost::in_degree(v, _graph);
	// 		
	// 		assert(degree_out == 0);
	// 		assert(degree_in == 0);
			
			boost::remove_vertex(v, _graph);
			
		}
		
		inline void GraphList::print(const topologicalmap::Vertex& v) const
		{
			std::cout << "Vertex : ";
			std::cout << _graph[v].type << " with index " << _graph[v].index << " at position " << _graph[v].point << " ";

			EdgeIterator out_i, out_end;
			Edge e;
			for (boost::tie(out_i, out_end) = boost::out_edges(v, _graph); 
				out_i != out_end; ++out_i) {
				e = *out_i;
				Vertex src = boost::source(e, _graph), targ = boost::target(e, _graph);
				std::cout << "(" << _graph[src].index << "," 
						<< _graph[targ].index << ") number : " << _graph[e].number << " || " ;
			}
			std::cout << std::endl;
		}
		
		inline float GraphList::distanceSquared(const topologicalmap::Vertex& v, const topologicalmap::Vertex& v2)
		{
			float dst_x = _graph[v].point.x - _graph[v2].point.x;
			float dst_y = _graph[v].point.y - _graph[v2].point.y;
			dst_x = dst_x * dst_x;
			dst_y = dst_y * dst_y;
			return dst_x + dst_y;	
		}

		inline Intersection_Graph& GraphList::operator[](const Vertex& v)
		{
			return _graph[v];
		}
		
		inline const Intersection_Graph& GraphList::operator[](const Vertex& v) const
		{
			return _graph[v];
		}
		
		
		inline void GraphList::fuse(topologicalmap::Vertex& v, topologicalmap::Vertex v_to_remove)
		{
			cv::Point2i other;
			getPoint(v_to_remove, other);
			cv::Point2i place;
			getPoint(v, place);
			
			int force = _graph[v].force;
			int force2 = _graph[v_to_remove].force;
			
			cv::Point2i new_pos(( (force * place.x) + (force2* other.x)) / (force + force2), ((force * place.y) + (force2* other.y)) / (force + force2));
			
			setPoint(v, new_pos);
			
			//Get all linked vertex of removed vertex :
			std::deque<Vertex> all_vertex_linked_to_remove;
			getAllVertexLinked(v_to_remove, all_vertex_linked_to_remove);
						
			for(size_t j = 0; j < all_vertex_linked_to_remove.size() ; j++){
				
				//Add edge between old node and all vertice linked to the one to remove.
				if(v != all_vertex_linked_to_remove.at(j)){
					addEdge(v, all_vertex_linked_to_remove.at(j));
				}

			}
			
			_graph[v].force =  force + force2;
			
			removeVertex(v_to_remove);
		}
		
		
		inline int GraphList::getNumEdges(topologicalmap::Vertex v) const
		{
			return boost::out_degree(v, _graph);
		}


	}
}
#endif
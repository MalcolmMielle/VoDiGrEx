#ifndef MULTILINEFOLLOWERBASE_POINT_MAP_21072016
#define MULTILINEFOLLOWERBASE_POINT_MAP_21072016
 
#include <opencv2/opencv.hpp>
#include <bettergraph/PseudoGraph.hpp>

// #include "SimpleNode.hpp"
// #include "utils/Utils.hpp"

namespace AASS{
		
	namespace vodigrex{

		
		/**
		* @brief Multiple-Lines follower algorithm to extract multiple graphs.
		* TODO template it to the Line follower and add Vertex and edge element in LineFollower
		*/
		template<typename VertexType, typename EdgeType, typename LineFollowerGraphType>
		class MultipleLineFollowerBase{
		protected :
			/// @brief deque of all topologicalmap::GraphList extracted from the image
			std::deque< bettergraph::PseudoGraph<VertexType, EdgeType> > _dgraphlist;
			///@brief deque with all the associated Mat for each graph
			std::deque<cv::Mat> _map_results;
			LineFollowerGraphType _line_follower;
			
		public :
			MultipleLineFollowerBase(){};
			
			void setLineFollower(const LineFollowerGraphType& lin){_line_follower = lin;}
			
			void inputMap(const cv::Mat& m){
				_line_follower.inputMap(m);
			}
			
			LineFollowerGraphType& getLineFollower(){return _line_follower;}
			
			/// @brief Return the cv::Mat result with the line drawn.
			cv::Mat& getResult(int i){return _map_results[i];}
			const cv::Mat& getResult(int i) const {return _map_results[i];}
			/// @brief Return unput cv::Mat
			cv::Mat& getMatIn(){return _line_follower.getMatIn();}
			void setD(int d){_line_follower.setD(d);}
			int getD(){return _line_follower.getD();}
			void setMinValueWhite(int w){_line_follower.setMinValueWhite(w);}
			int getMinValueWhite(){return _line_follower.getMinValueWhite();}
			void setMarge(int m){_line_follower.setMarge(m);}
			int getMarge(){return _line_follower.getMarge();}
			size_t size(){return _dgraphlist.size();}
			/**
			* @brief thinning algorithm
			* 
			* Extract all graph of the image and sort them by size. The algorithm is launched until all lines have been extracted.
			*/
			virtual void thin();
			
			/// @brief sort _dgraphlist by size of graphs
			virtual void sort();
			
			/**
			* @brief check if the image still countain some lines
			* 
			* @return true if the input map is black, false if it still countains some lines. 
			*/
			virtual bool isBlack();
			/// @brief clear all
			virtual void clear();
			
			///@brief return the i-est graph of the deque
			virtual const bettergraph::PseudoGraph<VertexType, EdgeType>& getGraph(int i) const {return _dgraphlist[i];}
			///@brief return the i-est graph of the deque
			virtual bettergraph::PseudoGraph<VertexType, EdgeType> getGraph(int i) {return _dgraphlist[i];}
			
		};
		
		template<typename VertexType, typename EdgeType, typename LineFollowerGraphType>
		inline void MultipleLineFollowerBase<VertexType, EdgeType, LineFollowerGraphType>::thin()
		{
// 			std::cout << "Multiple Thin" << std::endl;
// 			std::cout << std::endl;
			//while image is not dark.
	// 		int i = 0;
			while (isBlack() == false){
				
				_line_follower.thin();
				
	// 			std::cout << "Size of in : " << _dgraphlist.size() << std::endl;
				
	// 			cv::Mat bug = cv::imread("../Test/Sequences/Seq1/0012.jpg");
	// 			cv::Mat maa_3 = bug.clone();
	// 			maa_3.setTo(cv::Scalar(0));
	// 			LineFollower::drawGraph(maa_3);
	// 			cv::imshow("res", maa_3);
	// 			std::cout << _graph.getNumVertices() << std::endl;
	// 			cv::waitKey(0);
				
				bettergraph::PseudoGraph<VertexType, EdgeType> gl = _line_follower.getGraph();
	// 			std::cout << "NUM VERT " << gl.getNumVertices() << std::endl;
				_dgraphlist.push_back(gl);
				cv::Mat res;
				_line_follower.getResult().copyTo(res);
// 				cv::imshow("res", _line_follower.getResult());
// 				cv::waitKey(0);
				_map_results.push_back(res);
				
				
// 	// 			cv::Mat bug = cv::imread("../Test/Sequences/Seq1/0012.jpg");
// 	// 			cv::Mat maa_3 = bug.clone();
// 	// 			maa_3.setTo(cv::Scalar(0));
// 	// 			gl.draw(maa_3);
// 				cv::imshow("res2", res);
// 	// 			std::cout << _graph.getNumVertices() << std::endl;
// 				std::cout << "Size of : " <<_line_follower.getGraph().getNumVertices() << std::endl;
// 				cv::waitKey(0);
// 				cv::Mat maa_31 = this->_line_follower.getMatIn().clone();
// 				maa_31.setTo(cv::Scalar(0));
// 				AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(_line_follower.getGraph(), maa_31);
// 				cv::imshow("graph11", maa_31);
				
				
				_line_follower.clear();
				
				
			}
	// 		std::cout << "Size of : " << _dgraphlist.size() << std::endl;
			sort();
	// 		for(size_t i = 0 ; i < _dgraphlist.size() ; i++){
	// 			std::cout << "Size of biggest: " << _dgraphlist[i].getNumVertices() << std::endl;
	// 		}
		}

		template<typename VertexType, typename EdgeType, typename LineFollowerGraphType>
		inline void MultipleLineFollowerBase<VertexType, EdgeType, LineFollowerGraphType>::sort()
		{
			
			assert(_dgraphlist.size() == _map_results.size());
			
			//classify them in a clock wise manner
			typename std::deque<	
				bettergraph::PseudoGraph<VertexType, EdgeType>
			>::iterator dgraph_ite;
			
			typename std::deque<	
				bettergraph::PseudoGraph<VertexType, EdgeType>
			>::iterator dgraph_ite_2;
			
			typename std::deque<cv::Mat>::iterator map_ite;
			typename std::deque<cv::Mat>::iterator map_ite_2;
			
			bettergraph::PseudoGraph<VertexType, EdgeType> copy;
			cv::Mat map_copy;
			
			map_ite = _map_results.begin() + 1;
			for(dgraph_ite =  _dgraphlist.begin()+1 ; dgraph_ite !=  _dgraphlist.end() ; dgraph_ite++){
				
				dgraph_ite_2 = dgraph_ite ;
				copy = *dgraph_ite;
				
				map_ite_2 = map_ite;
				(*map_ite).copyTo(map_copy);
				
		// 			std::cout << "tsart" << std::endl;
		// 			std::cout << "something" <<  ( * (hypothesis_final_ite_2 - 1) ).getDist() << " < " << (*hypothesis_final_ite_2).getDist()  << std::endl;
				while( dgraph_ite_2 !=  _dgraphlist.begin() && (*(dgraph_ite_2 - 1)).getNumVertices() < copy.getNumVertices()){

					*( dgraph_ite_2 ) = *( dgraph_ite_2-1 );
					dgraph_ite_2 = dgraph_ite_2 - 1;
					
					(*(map_ite_2-1)).copyTo(*map_ite_2);
					map_ite_2 = map_ite_2 - 1;
				}
				
				*(dgraph_ite_2) = copy;
				map_copy.copyTo(*map_ite_2);
				
				map_ite++;
				
			}

		}
		
		template<typename VertexType, typename EdgeType, typename LineFollowerGraphType>
		inline bool MultipleLineFollowerBase<VertexType, EdgeType, LineFollowerGraphType>::isBlack()
		{
			for(int row = 0 ; row < _line_follower.getMatIn().rows ; row++){
				uchar* p = _line_follower.getMatIn().ptr(row); //point to each row
				for(int col = 0 ; col < _line_follower.getMatIn().cols ; col++){
					//p[j] <- how to access element
	// 				std::cout << (int)p[j]<< std::endl;
					if(p[col] > _line_follower.getMinValueWhite()){
						return false;
					}
				}
			}
			return true;

		}
		
		template<typename VertexType, typename EdgeType, typename LineFollowerGraphType>
		inline void MultipleLineFollowerBase<VertexType, EdgeType, LineFollowerGraphType>::clear()
		{
			//clear Boost graph
			_line_follower.clear();
			_dgraphlist.clear();
			
		}
		

		
	}
}

#endif
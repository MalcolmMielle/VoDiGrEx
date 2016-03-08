#ifndef MULTILINEFOLLOWER_POINT_MAP
#define MULTILINEFOLLOWER_POINT_MAP
 
#include <opencv2/opencv.hpp>
#include "LineFollower.hpp"
		
namespace AASS{
		
	namespace vodigrex{

		
		/**
		* @brief Multiple-Lines follower algorithm to extract multiple graphs.
		* 
		*/
		
		class MultipleLineFollower : public LineFollower{
		protected :
			/// @brief deque of all topologicalmap::GraphList extracted from the image
			std::deque< topologicalmap::GraphList > _dgraphlist;
			
		public :
			MultipleLineFollower() : LineFollower() {};
			
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
			virtual void reset();
			void drawGraph(cv::Mat& m, int i);
			virtual void printGraph(int i);
			
			///@brief return the i-est graph of the deque
			virtual const GraphList& getGraph(int i) const {return _dgraphlist[i];}
			///@brief return the i-est graph of the deque
			virtual GraphList getGraph(int i) {return _dgraphlist[i];}
			
		};
		
		inline void MultipleLineFollower::thin()
		{
			std::cout << "Multiple Thin" << std::endl;
			std::cout << std::endl;
			//while image is not dark.
	// 		int i = 0;
			while (isBlack() == false){
				
				LineFollower::thin();
				
	// 			std::cout << "Size of in : " << _dgraphlist.size() << std::endl;
				
	// 			cv::Mat bug = cv::imread("../Test/Sequences/Seq1/0012.jpg");
	// 			cv::Mat maa_3 = bug.clone();
	// 			maa_3.setTo(cv::Scalar(0));
	// 			LineFollower::drawGraph(maa_3);
	// 			cv::imshow("res", maa_3);
	// 			std::cout << _graph.getNumVertices() << std::endl;
	// 			cv::waitKey(0);
				
				topologicalmap::GraphList gl = _graph;
	// 			std::cout << "NUM VERT " << gl.getNumVertices() << std::endl;
				_dgraphlist.push_back(gl);
				LineFollower::reset();
				
				
	// 			cv::Mat bug = cv::imread("../Test/Sequences/Seq1/0012.jpg");
	// 			cv::Mat maa_3 = bug.clone();
	// 			maa_3.setTo(cv::Scalar(0));
	// 			gl.draw(maa_3);
	// 			cv::imshow("res", maa_3);
	// 			std::cout << _graph.getNumVertices() << std::endl;
	// 			std::cout << "Size of : " << _dgraphlist.size() << std::endl;
	// 			cv::waitKey(0);
				
				
			}
	// 		std::cout << "Size of : " << _dgraphlist.size() << std::endl;
			sort();
	// 		for(size_t i = 0 ; i < _dgraphlist.size() ; i++){
	// 			std::cout << "Size of biggest: " << _dgraphlist[i].getNumVertices() << std::endl;
	// 		}
		}

		
		inline void MultipleLineFollower::sort()
		{
			//classify them in a clock wise manner
			std::deque<	
				GraphList
			>::iterator dgraph_ite;
			
			std::deque<	
				GraphList
			>::iterator dgraph_ite_2;
			
			GraphList copy;
			
			for(dgraph_ite =  _dgraphlist.begin()+1 ; dgraph_ite !=  _dgraphlist.end() ; dgraph_ite++){
				
				dgraph_ite_2 = dgraph_ite ;
				copy = *dgraph_ite;
		// 			std::cout << "tsart" << std::endl;
		// 			std::cout << "something" <<  ( * (hypothesis_final_ite_2 - 1) ).getDist() << " < " << (*hypothesis_final_ite_2).getDist()  << std::endl;
				while( dgraph_ite_2 !=  _dgraphlist.begin() && (*(dgraph_ite_2 - 1)).getNumVertices() < copy.getNumVertices()){

					*( dgraph_ite_2 ) = *( dgraph_ite_2-1 );
					dgraph_ite_2 = dgraph_ite_2 - 1;
				}
				*(dgraph_ite_2) = copy;
			}

		}
		
		inline bool MultipleLineFollower::isBlack()
		{
			for(int row = 0 ; row < getMatIn().rows ; row++){
				uchar* p = getMatIn().ptr(row); //point to each row
				for(int col = 0 ; col < getMatIn().cols ; col++){
					//p[j] <- how to access element
	// 				std::cout << (int)p[j]<< std::endl;
					if(p[col] > getMinValueWhite()){
						return false;
					}
				}
			}
			return true;

		}
		
		inline void MultipleLineFollower::reset()
		{
			//reset Boost graph
			LineFollower::reset();
			_dgraphlist.clear();
			
		}

		//To slow
		inline void MultipleLineFollower::printGraph(int i)
		{

			_dgraphlist[i].print();
		}
		
		inline void MultipleLineFollower::drawGraph(cv::Mat& m, int i)
		{
			_dgraphlist[i].draw(m);
		}
		

		
	}
}

#endif
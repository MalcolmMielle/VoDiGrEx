#ifndef LINEFOLLOWER_POINT_MAP
#define LINEFOLLOWER_POINT_MAP

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <stdexcept>

#include <opencv2/opencv.hpp>

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
		//USE THE CLASS MADE FOR THE BETTER GRAPH HERE TOO
		class LineFollower{
	
			
		protected : 
			/// @brief Map to thin
			cv::Mat _map_in;
			/// @brief Map with the thinned lines
			cv::Mat _map_result; 
			/// @brief Pointer to the left edge of a line
			cv::Point2i _LP; 
			/// @brief Pointer to the right edge of a line
			cv::Point2i _RP; 
			/// @brief Last Pointer to the left edge of the line
			cv::Point2i _LLP; 
			/// @brief Last Pointer to the right side of the line
			cv::Point2i _LRP; 
			/// @brief Dynamic windows to draw the center of the line
			cv::Mat _W; 
			/// @brief Parameter for the size of the dynamique window
			int _d; 
			/// @brief last drawing point
			cv::Point2i _last_drawing_point;
			/// @brief Double point Lp and RP
			std::deque<std::pair< cv::Point2i, cv::Point2i > > _LRP_to_explore; 
			/// @brief parent index of the Lp and Rp explored
// 			std::deque< int > _dads_index;
			/// @brief deque of all point in space of the parent of line to explore
			std::deque<cv::Point2i> _last_drawing_point_deque;	
			/// @brief Trigger for the white color for the line detection. value are between 0 (pitch dark) and 255 (full white)
			int _value_of_white_min;
			
			
			
		public:
			
			LineFollower() : _d(2), _value_of_white_min(100){};
			virtual ~LineFollower(){
				//Clean up the vector of intersection
				clear();
			}
			
			void inputMap(const cv::Mat& m){
				cv::Mat tmp, tmp2;
				
				if(m.channels() == 3){
					cv::cvtColor(m, tmp2, CV_RGB2GRAY);
				}
				else{
					m.copyTo(tmp2);
				}				
				//Convert to the good type of matric for element access
				tmp2.convertTo(tmp, CV_8UC1);
				
				_map_in = tmp.clone();
	// 			cv::imshow("map in line follower " , m);
				_map_result = _map_in.clone();
				_map_result = cv::Scalar(0);
				
			}
			/// @brief Return the cv::Mat result with the line drawn.
			cv::Mat& getResult(){return _map_result;}
			/// @brief Return unput cv::Mat
			cv::Mat& getMatIn(){return _map_in;}
			void setD(int d){_d = d;}
			int getD(){return _d;}
			void setMinValueWhite(int w){_value_of_white_min = w;}
			int getMinValueWhite(){return _value_of_white_min;}

			virtual void clear();
			
			void printIntersection(){
				for(size_t i = 0 ; i < _LRP_to_explore.size() ; i++){
					std::cout << " point 1 " << _LRP_to_explore[i].first << " point 2 " << _LRP_to_explore[i].second ;
				}
			}
			
			/**
			* @brief line thining algorithm
			* 
			*/
			virtual void thin();
			
			
		protected:
			cv::Mat& getDynamicWindow(){return _W;}
			cv::Point2i getLP(){return _LP;}
			cv::Point2i getRP(){return _RP;}
			
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
			int typeOfIntersection(cv::Mat& roi);
			void init();
			
			/**
			 * @brief move the dynamic window forward and resize it to match the new anchor points
			 */
			void moveForward();
			void moveForward(bool erase_W);
			bool testNewBranchNotBlackandMoveForward();
			void drawLine();
			bool findNextLPRP(std::vector< cv::Point2i >& all_points);
			void addPoint2Explore(const std::vector< cv::Point2i >& all_points);
			void removeLineSegment(cv::Mat& c);
			/**
			 * @brief Resize the dynamic windows the side here is starting with north and going clockwise
			 */
			void upResize(bool north, bool est, bool south, bool west);

			/**
			* @brief line thining algorithm after init.
			* 
			*/
			virtual void lineThinningAlgo();
			
			/**
			* @brief return the mininmal distance between to point of input vector. 
			* 
			* @param[in] all_points : vector of cv::Point2i
			* @return minimum distance
			*/
			double calculateDistance(std::vector<cv::Point2i>& all_points);
			
			virtual void getNewBranch();
			
		};

		


	}
}

#endif
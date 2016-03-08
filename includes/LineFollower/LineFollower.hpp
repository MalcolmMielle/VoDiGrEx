#ifndef LINEFOLLOWER_POINT_MAP
#define LINEFOLLOWER_POINT_MAP

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <stdexcept>
// #include "boost/graph/adjacency_list.hpp"
// #include "boost/graph//*topological_sort*/.hpp"

#include <opencv2/opencv.hpp>

#ifdef UNIT_TEST

namespace unit_test {   // Name this anything you like
	struct LineFollowerTester; // Forward declaration for befriending
}

#endif

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
		//USE THE CLASS MADE FOR THE BETTER GRAPH HERE TOO
		class LineFollower{
			
		//Friend for unit testing of private function
		#ifdef UNIT_TEST
			friend struct unit_test::LineFollowerTester;
		#endif
			
			
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
			std::deque< int > _dads_index;
			/// @brief deque of all point in space of the parent of line to explore
			std::deque<cv::Point2i> _last_drawing_point_deque;	
			/// @brief Trigger for the white color for the line detection. value are between 0 (pitch dark) and 255 (full white)
			int _value_of_white_min;
			/// @brief Every two point closer than _marge are to be fused as one.
			int _marge;
			
			
		public:
			
			LineFollower() : _d(2), _value_of_white_min(100), _marge(10){};
			virtual ~LineFollower(){
				//Clean up the vector of intersection
				clear();
			}
			
			virtual void inputMap(const cv::Mat& m){
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
			virtual cv::Mat& getResult(){return _map_result;}
			/// @brief Return unput cv::Mat
			virtual cv::Mat& getMatIn(){return _map_in;}
			virtual void setD(int d){_d = d;}
			virtual int getD(){return _d;}
			virtual void setMinValueWhite(int w){_value_of_white_min = w;}
			virtual int getMinValueWhite(){return _value_of_white_min;}
			virtual void setMarge(int m){_marge = m;}
			
			virtual cv::Mat& getDynamicWindow(){return _W;}
			cv::Point2i getLP(){return _LP;}
			cv::Point2i getRP(){return _RP;}

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
			void thin();
			
			
		protected:
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
			void drawLine();
			bool findNextLPRP(std::vector< cv::Point2i >& all_points);
			void addPoint2Explore(const std::vector< cv::Point2i >& all_points);
			void removeLineSegment(cv::Mat& c);
			void upResize();

			/**
			* @brief line thining algorithm after init.
			* 
			*/
			void lineThinningAlgo();
			
			/**
			* @brief return the mininmal distance between to point of input vector. 
			* 
			* @param[in] all_points : vector of cv::Point2i
			* @return minimum distance
			*/
			double calculateDistance(std::vector<cv::Point2i>& all_points);
			
			void getNewBranch();
			
		};

		

		inline void LineFollower::thin()
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
				
				_last_drawing_point = cv::Point(p.x + (_W.rows/2), p.y + (_W.cols/2));
				
		// 			std::cout << "We start " << " at " << p.x << " " << p.y << " dyn wind " << _W.rows << " " << _W.cols << " draw oint " << _last_drawing_point << std::endl;
				
				std::vector<cv::Point2i> all_point;
				findNextLPRP(all_point);
				_LP = all_point[0];
				_RP = all_point[1];
				
// 				Vertex dad;
				
				if(all_point.size() == 2){
					lineThinningAlgo();
				}
				//Line
				else{
					addPoint2Explore(all_point);
					lineThinningAlgo();	
				}
			
			}
			catch(const std::exception &e){
				std::cout << "the unthinkable happened during voronoi landmark detection : " << e.what() << std::endl;
			}
		}

		
		inline void LineFollower::lineThinningAlgo()
		{
			
			while(_LP.x != -1 && _LP.y != -1){

				//Get all the next line to explore
				std::vector<cv::Point2i> all_point;
				bool non_dead_end = findNextLPRP(all_point);
				
				//Get the point of the window
				cv::Size s;
				cv::Point2i p_dyn_window;
				_W.locateROI(s, p_dyn_window);
				
				cv::Point2i new_p;
				new_p.x = p_dyn_window.x + (_W.cols / 2);
				new_p.y = p_dyn_window.y + (_W.rows / 2);
				
				//Intersection or dead end
				if( all_point.size() > 2 || non_dead_end == false){
					addPoint2Explore(all_point/*, created*/);
					getNewBranch();					
				}
				else{
					_LP = all_point[0];
					_RP = all_point[1];
					moveForward();						
		
				}
			}
		}

		
		
		inline void LineFollower::init()
		{
			
			/*
			* Build the first window
			*/
			//Clean up old results ;
			LineFollower::clear();
			
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
		
		inline void LineFollower::getNewBranch()
		{
			
			if(_LRP_to_explore.size() > 0){
				//Access new LP RP
				_RP = _LRP_to_explore[0].first; 
				_LP = _LRP_to_explore[0].second;
				_last_drawing_point = _last_drawing_point_deque[0];
				//Remove them
				_LRP_to_explore.pop_front();
				_last_drawing_point_deque.pop_front();
				
				moveForward();
			}
			else{
				_LP.x = -1;
				_LP.y = -1;
			}

		}

		
		//TODO remove only the segment instead of the whole thing
		inline void LineFollower::removeLineSegment(cv::Mat& c)
		{
			//Make the whole mat black
			c.setTo(cv::Scalar(0));
		}

		inline void LineFollower::drawLine()
		{
			cv::line(_map_result, _last_drawing_point, cv::Point((_LP.x + _RP.x)/2, (_LP.y + _LP.y)/2), cv::Scalar(255));	
			_last_drawing_point = cv::Point((_LP.x + _RP.x)/2, (_LP.y + _LP.y)/2);
		}

		inline void LineFollower::upResize()
		{
			
			if(_W.size().width >= _map_in.size().width && _W.size().height >= _map_in.size().height){
				throw std::runtime_error("Dynamic window is the size (or more) of the image. It means that something went wrong in upResize");
			}
	// 		std::cout << "Upsize" << std::endl;
			cv::Size s;
			cv::Point2i p;
			_W.locateROI(s, p);
			
			int emergency_flag = true;
	// 		int rows = _W.rows;
	// 		int cols = _W.cols;
			
			int x = p.x;
			int y = p.y;
			int width =  _W.size().width;
			int height =  _W.size().height;
			
			if(x - 1 >= 0){
				x = x - 1;
				emergency_flag = false;
				if(x + width + 2 < _map_in.size().width-1 ){
					width = width + 2;
				}
				else if (x + width != _map_in.size().width-1){
					width++;
				}
			}
			else if (x + width != _map_in.size().width-1){
				emergency_flag = false;
				width++;
			}
			
			if(y - 1 >=0){
				y = y - 1;
				emergency_flag = false;
				if(y +height + 2 < _map_in.size().height-1){
					height = height + 2;
				}
				else if (y +height != _map_in.size().height-1){
					height++;
				}
			}
			else if (y +height != _map_in.size().height-1){
				emergency_flag = false;
				height++;
			}
			
	// 		std::cout << "new measurement" << x << " " << y << " " << width << " " << height << " for " << _map_in.size().height << std::endl;
			
			if(emergency_flag == true){
				throw std::runtime_error("Lost the line, impossible to resize the dynamic window to find it again.");
			}
			//Define new window
			//Rect got everything inversed. It needs a point with first dim as col and second as row
			_W = _map_in(cv::Rect( x , y  , width, height ));
			
			if(_W.size().width >= _map_in.size().width && _W.size().height >= _map_in.size().height){
				throw std::runtime_error("Dynamic window is the size of the image. It means that something went wrong in upResize");
			}
			
		}



		/*
		* Return a list of all LP and RP extracted rom the dynamic window
		* return false is nothing is an the dynamic window
		*/
		inline bool LineFollower::findNextLPRP(std::vector< cv::Point2i >& all_points)
		{
			
	// 		std::cout << "inside find new lp for the list"<< std::endl << _W << std::endl;
			//Memorise old position
			double dist_min = 0 ;
			
			while (dist_min <= _d){

				all_points.clear();
	// 			std::cout << _W<<std::endl;
				
				int i = 1, j = 0;
				int i_prev = 0, j_prev = 0;
				
				//Go to the first black point + 1
				int count_tmp=0;
				uchar val = _W.at<uchar>(j, i);
		// 		std::cout << "value for init : " << (int) val<<std::endl;
				while(val >= _value_of_white_min && count_tmp < 2*(_W.cols+_W.rows)+1 ){
					count_tmp++;
					i_prev = i;
					j_prev = j;
					if( j == 0 && i < _W.cols-1){
						i++;
					}
					else if (i == _W.cols-1 && j < _W.rows-1){
						
						j++;
					}
					else if (j == _W.rows-1 && i > 0 ){
						i--;
						
					}
					else if (j > 0 && i == 0 ){
						j--;
					}
					val = _W.at<uchar>(j, i);
		//  			std::cout << "value for init : " << (int) val<<std::endl;
				}
		// 		std::cout << "first black point : " << i << " "<< j << std::endl;
				if( count_tmp == 2*(_W.cols+_W.rows)+1){
					throw std::runtime_error("Dynamic window is full of white !!");
				}

				//Find all segment with end position
				count_tmp = 0;
				int prev = _W.at<uchar>(j, i);
				int actual = -1;
				while(count_tmp < 2*(_W.cols+_W.rows-2) ){
					
					count_tmp++;
		// 			std::cout << "COUNTING / : "<< count_tmp << std::endl;
		// 			std::cout << " i and j "<<i <<" "<<j << std::endl;
					//Go around the Dynamic window
					if( i < _W.cols-1 && j == 0){
						i++;
		// 				std::cout << " 1at " << i << " " << j;
						actual = _W.at<uchar>(j, i);
					}
					else if(j < _W.rows-1 && i == _W.cols-1){
						j++;
		// 				std::cout << " 2at " << i << " " << j;
						actual = _W.at<uchar>(j, i);
					}
					else if(i > 0 && j == _W.rows-1){
						i--;
		// 				std::cout << " 3at " << i << " " << j;
						actual = _W.at<uchar>(j, i);
					}
					else if(j > 0 && i == 0){
						j--;
		// 				std::cout << " 4at " << i << " " << j;
						actual = _W.at<uchar>(j, i);
					}
					
		// 			std::cout << std::endl;
		// 			int sum = actual - prev;
		// 			int a = actual;
		// 			int p = prev;

		// 			std::cout << actual << " " << prev <<std::endl;
					
					
					if( actual > _value_of_white_min && prev <= _value_of_white_min){
						cv::Size s;
						cv::Point2i p;
						_W.locateROI(s, p);
						
	// 					std::cout << "pushing point  : " << cv::Point2i(p.x + i, p.y + j) << " at : " << i << " and " << j << std::endl;
						
						all_points.push_back(cv::Point2i(p.x + i, p.y + j)); 
						//_RP.y = p.y + j;
						//_RP.x = p.x + i;
					}
					
					if( actual <=_value_of_white_min && prev > _value_of_white_min){
						cv::Size s;
						cv::Point2i p;
						_W.locateROI(s, p);
						
	// 					std::cout << "pushing end point  : " << cv::Point2i(p.x + i_prev, p.y + j_prev) << " at : " << i_prev << " and " << j_prev << std::endl;
						
						all_points.push_back(cv::Point2i(p.x + i_prev, p.y + j_prev));
						
		// 				_LP.y = p.y + j_prev;
		// 				_LP.x = p.x + i_prev;
					}
				
					prev = actual;
					i_prev = i;
					j_prev = j;
					
				}
				if( all_points.size() == 0){
					return false;
				}
				
				//If we should enough branches
				if(all_points.size() > 2){
					//Calculate distance in between branches
					dist_min = calculateDistance(all_points);
// 					std::cout << "DISTANCE MIN IS " << dist_min << std::endl << std::endl;
					//If the distance between branches is not good enough
					if(dist_min <= _d){
// 						std::cout << dist_min << " d " << _d << std::endl;
						upResize();
// 						std::cout << "RESIZED" << std::endl;
					}
				}
				//get out of the loop
				else{
					dist_min = _d+1;
				}
			}
// 			std::cout << "Out with " << all_points.size() << " ";
// 			std::cout << dist_min << " d " << _d << std::endl;
			return true;
			
		}
		
		
		inline void LineFollower::addPoint2Explore(const std::vector< cv::Point2i >& all_points)
		{
			if(all_points.size() >= 2){
				for(size_t i = 0 ; i < all_points.size() ; i=i+2){
					_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(all_points[i], all_points[i+1]));
					_last_drawing_point_deque.push_back(_last_drawing_point);
					
				}
			}
		}


		
		inline double LineFollower::calculateDistance(std::vector< cv::Point2i >& all_points)
		{
			//Calculating the distance in betzenn al point and return min_col
			double min = -1;
			double tmp;
			double tmp2;
			
			//init compare first and last ones
			if(all_points.size()>=2){
				tmp = all_points[0].x - all_points[all_points.size()-1].x;
				tmp2 = all_points[0].y - all_points[all_points.size()-1].y;
				
				if(tmp < 0){tmp = -tmp;}
				if(tmp2 < 0){tmp2 = -tmp2;}
				
				if(min == -1 || min > (tmp + tmp2)){
					min = tmp + tmp2;
				}
			}
			
			//Since they are classified, we need only one pass
			for (size_t i = 1 ; i < all_points.size()-1 ; i=i+2){
				tmp = all_points[i].x - all_points[i+1].x;
				tmp2 = all_points[i].y - all_points[i+1].y;
				
				if(tmp < 0){tmp = -tmp;}
				if(tmp2 < 0){tmp2 = -tmp2;}
				
				if(min == -1 || min > (tmp + tmp2)){
					min = tmp + tmp2;
				}
			}
			
			return min;
			
		}

		
		inline void LineFollower::moveForward()
		{
			drawLine();
			removeLineSegment(_W);
			
			// Find min and max points of the new points
			int max_row = 0;
			int min_row = 0;
			int max_col = 0;
			int min_col = 0;
			if( _LP.y > _RP.y){
				max_row = _LP.y;
				min_row = _RP.y;
			}
			else{
				max_row = _RP.y;
				min_row = _LP.y;
			}
			
			if(_LP.x > _RP.x){
				max_col = _LP.x;
				min_col = _RP.x;
			}
			else{
				max_col = _RP.x;
				min_col = _LP.x;
			}
			
	// 		std::cout << "defining new mat" << std::endl;
			int point_min_col = 0;
			int point_min_row = 0;
			int point_max_col = 0;
			int point_max_row = 0;
			
			if( min_col - _d >= 0 ){
				point_min_col = min_col - _d;
			}
			else{
				point_min_col = 0;
			}		
			if( min_row - _d >= 0){
				point_min_row = min_row - _d;
			}
			else{
				point_min_row = 0;
			}
			
			if(max_col + _d < _map_in.cols){
				point_max_col = max_col + _d;
			}
			else{
				point_max_col = _map_in.cols-1;
			}
			if(max_row + _d < _map_in.rows){
				point_max_row = max_row + _d;
			}
			else{
				point_max_row = _map_in.rows-1;
			}
			
			//Define new window
			//Rect got everything inversed. It needs a point with first dim as col and second as row
			_W = _map_in(cv::Rect( cv::Point2i(point_min_col , point_min_row ) , cv::Point2i(point_max_col, point_max_row )) );
			
			//Resize the window to match the line
			int type = typeOfIntersection(_W);
			//If we lost the line we upsize W
			while(type == 0){
				upResize();
				type = typeOfIntersection(_W);
			}
			
		}

		
		/*
		* This function return the number of line corssing in a square.
		* 1 means a (at init : dead else line)
		* 2 means a (at init : line else T crossing)
		* more means a N crossing
		* 0 means that we are not completely on at least one of the branch
		* -1 means that there is no line at all
		* -2 means that we can only see white
		*/
		
		//TODO slow function... and probably horribly not optimized
		
		inline int LineFollower::typeOfIntersection(cv::Mat& roi)
		{
			
			int count = 0;
			int count_noline = -1;
	// 		int count_black_after_white_line = 0;
			bool flag_first = false;
			int i = 0 ; int j = 0;
			int dist = 0;
			int flag_complete_lines = true;
			
			//Go to the first black point
			int count_tmp = 0;
			uchar val = roi.at<uchar>(i, j);
			while(val > 150 && count_tmp < 2*(roi.cols+roi.rows)+1 ){
				count_tmp++;
				if( j == 0 && i < roi.cols-1){
					i++;
				}
				else if (i == roi.cols-1 && j < roi.rows-1){
					j++;
				}
				else if (j == roi.rows-1 && i > 0 ){
					i--;
				}
				else if (j > 0 && i == 0 ){
					j--;
				}
	// 			std::cout << " i and j "<<i <<" "<<j << " at siwe " << roi.size() << std::endl;
				val = roi.at<uchar>(j, i);
			}
	// 		std::cout << "first black point : " << i << " "<< j << std::endl;
			if( count_tmp == 2*(roi.cols+roi.rows)+1){
				return -2;
			}
			
			else{
				count_tmp = 0;
				while(count_tmp < 2*(roi.cols+roi.rows-2) ){
					count_tmp++;
// 					std::cout << " i and j "<<i <<" "<<j << " value " << (int) roi.at<uchar>(j, i) << std::endl;
					if( j == 0 && i < roi.cols -1){
						i++;
						//Cross a white part
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == false){
							flag_first = true;
							count_noline++;
						}
						//Count distance until the get out of white line
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == true){
							dist++;
						}
						//End of it so we passed a white line
						else if(roi.at<uchar>(j, i) <= _value_of_white_min && flag_first == true){
							flag_first = false;
							if(dist < roi.cols){
	// 							std::cout << "here "<< j << " " << i <<std::endl;
								count++;
							}
							else{
								flag_complete_lines = false;
							}
							dist = 0;
						}
						
					}
					else if (i == roi.cols-1 && j < roi.rows-1){
						j++;
						//Cross a white part
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == false){
							flag_first = true;
							count_noline++;
						}
						//Count distance until the get out of white line
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == true){
							dist++;
						}
						//End of it so we passed a white line
						else if(roi.at<uchar>(j, i) <= _value_of_white_min && flag_first == true){
							flag_first = false;
							if(dist < roi.rows){
	// 							std::cout << "here "<< j << " " << i <<std::endl;
								count++;
							}
							else{
								flag_complete_lines = false;
							}
							dist = 0;
						}
						
					}
					
					else if (j == roi.rows-1 && i > 0 ){
						//Cross a white part
						i--;
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == false){
							flag_first = true;
							count_noline++;
						}
						//Count distance until the get out of white line
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == true){
							dist++;
						}
						//End of it so we passed a white line
						else if(roi.at<uchar>(j, i) <= _value_of_white_min && flag_first == true){
							flag_first = false;
							if(dist < roi.cols){
	// 							std::cout << "here "<< j << " " << i <<std::endl;
								count++;
							}
							else{
								flag_complete_lines = false;
							}
							dist = 0;
						}
						
					}
					else if (j > 0 && i == 0 ){
						j--;
						//Cross a white part
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == false){
							flag_first = true;
							count_noline++;
						}
						//Count distance until the get out of white line
						if(roi.at<uchar>(j, i) > _value_of_white_min && flag_first == true){
							dist++;
						}
						//End of it so we passed a white line
						else if(roi.at<uchar>(j, i) <= _value_of_white_min && flag_first == true){
							flag_first = false;
							if(dist < roi.rows){
	// 							std::cout << "here "<< j << " " << i <<std::endl;
								count++;
							}
							else{
								flag_complete_lines = false;
							}
							dist = 0;
						}
						
					}
				}
			}
			
			if(count_noline == -1){
				return count_noline;
			}
			else{
				if(flag_complete_lines == true){
					return count;
				}
				else{
					return 0;
				}
			}
		}

		inline void LineFollower::clear()
		{
			//reset Boost graph
			_map_result = cv::Scalar(0,0,0);
// 			_graph.clear(); 
			_LRP_to_explore.clear();
// 			_dad_vertex.clear();
			_last_drawing_point_deque.clear();
		}

		//To slow
// 		inline void LineFollower::printGraph()
// 		{
// 
// 			_graph.print();
// 		}
// 		
// 		inline void LineFollower::drawGraph(cv::Mat& m)
// 		{
// 			_graph.draw(m);
// 		}



		
	}
}

#endif
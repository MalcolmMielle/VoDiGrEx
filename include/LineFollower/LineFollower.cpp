#include "LineFollower.hpp"

void AASS::vodigrex::LineFollower::thin()
{
	try{
		cv::copyMakeBorder( _map_in, _map_in, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0 );
		//Build the first ROI
		init();
		//Init the first drawing point
		cv::Point2i p;
		cv::Size s;
		_W.locateROI(s, p);
		
		_last_drawing_point = cv::Point(p.x + (_W.rows/2), p.y + (_W.cols/2));
		
		std::vector<cv::Point2i> all_point;
		findNextLPRP(all_point);
		_LP = all_point[0];
		_RP = all_point[1];
		
		if(all_point.size() == 2){
			lineThinningAlgo();
		}
		else{
			addPoint2Explore(all_point);
			lineThinningAlgo();	
		}
	
	}
	catch(const std::exception &e){
		std::cout << "the unthinkable happened during voronoi landmark detection : " << e.what() << std::endl;
	}
}


inline void AASS::vodigrex::LineFollower::lineThinningAlgo()
{
	
	std::cout << "BASE line " << std::endl;
	
	while(_LP.x != -1 && _LP.y != -1){
		
// 		cv::imshow("during", _map_in);
// 		cv::waitKey(1);
		
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
// 			cv::waitKey(0);
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



inline void AASS::vodigrex::LineFollower::init()
{
	
	/*
	* Build the first window
	*/
	//Clean up old results ;
	AASS::vodigrex::LineFollower::clear();
	
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

			//Test new window
			_W = _map_in(cv::Rect( cv::Point(i - radius_min_width , j - radius_min_height), cv::Point( i + radius_max_width , j + radius_max_height ) ));
			type = typeOfIntersection(_W);
			
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
}

inline void AASS::vodigrex::LineFollower::getNewBranch()
{
	
	if(_LRP_to_explore.size() > 0){
		//Access new LP RP
		_RP = _LRP_to_explore[0].first; 
		_LP = _LRP_to_explore[0].second;
		_last_drawing_point = _last_drawing_point_deque[0];
		//Remove them
		_LRP_to_explore.pop_front();
		_last_drawing_point_deque.pop_front();
		
		bool ret = testNewBranchNotBlackandMoveForward();
		if(ret == false){
			getNewBranch();
		}
		
	}
	else{
		_LP.x = -1;
		_LP.y = -1;
	}

}


//TODO remove only the segment instead of the whole thing
inline void AASS::vodigrex::LineFollower::removeLineSegment(cv::Mat& c)
{
	//Make the whole mat black
	c.setTo(cv::Scalar(0));
}

inline void AASS::vodigrex::LineFollower::drawLine()
{
	cv::line(_map_result, _last_drawing_point, cv::Point((_LP.x + _RP.x)/2, (_LP.y + _RP.y)/2), cv::Scalar(255));	
	_last_drawing_point = cv::Point((_LP.x + _RP.x)/2, (_LP.y + _RP.y)/2);
}

inline void AASS::vodigrex::LineFollower::upResize(bool north, bool est, bool south, bool west)
{
	
	if(_W.size().width >= _map_in.size().width && _W.size().height >= _map_in.size().height){
		throw std::runtime_error("Dynamic window is the size (or more) of the image. It means that something went wrong in upResize");
	}
	cv::Size s;
	cv::Point2i p;
	_W.locateROI(s, p);
	
	int emergency_flag = true;
	
	int x = p.x;
	int y = p.y;
	int width =  _W.size().width;
	int height =  _W.size().height;
	
	if(north == true && x - 1 >= 0){
		x = x - 1;
		emergency_flag = false;
		if(south == true && x + width + 2 < _map_in.size().width-1 ){
			width = width + 2;
		}
		else if (x + width != _map_in.size().width-1){
			width++;
		}
	}
	else if(south == true && x + width != _map_in.size().width-1){
		emergency_flag = false;
		width++;		
	}
	if(est == true && y - 1 >=0){
		y = y - 1;
		emergency_flag = false;
		if(west == true && y +height + 2 < _map_in.size().height-1){
			height = height + 2;
		}
		else if(y +height != _map_in.size().height-1){
			height++;
		}
	}
	else if (west == true && y +height != _map_in.size().height-1){
		emergency_flag = false;
		height++;
	}
	
	if(emergency_flag == true){
		throw std::runtime_error("Nothing happened in the resize of the dynamic window");
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
inline bool AASS::vodigrex::LineFollower::findNextLPRP(std::vector< cv::Point2i >& all_points)
{
	
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
		}
		if( count_tmp == 2*(_W.cols+_W.rows)+1){
			throw std::runtime_error("Dynamic window is full of white !!");
		}

		//Find all segment with end position
		count_tmp = 0;
		int prev = _W.at<uchar>(j, i);
		int actual = -1;
		while(count_tmp < 2*(_W.cols+_W.rows-2) ){
			
			count_tmp++;
			//Go around the Dynamic window
			if( i < _W.cols-1 && j == 0){
				i++;
				actual = _W.at<uchar>(j, i);
			}
			else if(j < _W.rows-1 && i == _W.cols-1){
				j++;
				actual = _W.at<uchar>(j, i);
			}
			else if(i > 0 && j == _W.rows-1){
				i--;
				actual = _W.at<uchar>(j, i);
			}
			else if(j > 0 && i == 0){
				j--;
				actual = _W.at<uchar>(j, i);
			}
			
			
			if( actual > _value_of_white_min && prev <= _value_of_white_min){
				cv::Size s;
				cv::Point2i p;
				_W.locateROI(s, p);
				
				all_points.push_back(cv::Point2i(p.x + i, p.y + j)); 
			}
			
			if( actual <=_value_of_white_min && prev > _value_of_white_min){
				cv::Size s;
				cv::Point2i p;
				_W.locateROI(s, p);
				
				all_points.push_back(cv::Point2i(p.x + i_prev, p.y + j_prev));
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
			//If the distance between branches is not good enough
			if(dist_min <= _d){
				upResize(true, true, true, true);
			}
		}
		//get out of the loop
		else{
			dist_min = _d+1;
		}
	}
	return true;
	
}


inline void AASS::vodigrex::LineFollower::addPoint2Explore(const std::vector< cv::Point2i >& all_points)
{
	if(all_points.size() >= 2){
		for(size_t i = 0 ; i < all_points.size() ; i=i+2){
			_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(all_points[i], all_points[i+1]));
			_last_drawing_point_deque.push_back(_last_drawing_point);
			
		}
	}
}



inline double AASS::vodigrex::LineFollower::calculateDistance(std::vector< cv::Point2i >& all_points)
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


inline void AASS::vodigrex::LineFollower::moveForward()
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
		upResize(true, true, true, true);
		type = typeOfIntersection(_W);
	}
	
// 	std::cout << "At the end type : " << type << std::endl;
// 	cv::imshow("Dyn win", _W);
}



inline bool AASS::vodigrex::LineFollower::testNewBranchNotBlackandMoveForward()
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
	if(type == -1){
// 		std::cout << "PUTAIN" << std::endl;
// 		exit(0);
		return false;
	}
// 	std::cout << "this is the type" << type << std::endl;
// 	cv::waitKey(0);
	return true;
	
// 	std::cout << "At the end type : " << type << std::endl;
// 	cv::imshow("Dyn win", _W);
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

inline int AASS::vodigrex::LineFollower::typeOfIntersection(cv::Mat& roi)
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
		val = roi.at<uchar>(j, i);
	}
	if( count_tmp == 2*(roi.cols+roi.rows)+1){
		return -2;
	}
	
	else{
		count_tmp = 0;
		while(count_tmp < 2*(roi.cols+roi.rows-2) ){
			count_tmp++;
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

inline void AASS::vodigrex::LineFollower::clear()
{
	_map_result = cv::Scalar(0,0,0);
	_LRP_to_explore.clear();
	_last_drawing_point_deque.clear();
}
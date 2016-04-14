		
		template<typename VertexType, typename EdgeType>
void LineFollowerGraph<VertexType, EdgeType>::thin()
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
void LineFollowerGraph<VertexType, EdgeType>::lineThinningAlgo(Vertex& index_dad)
{
	
// 	std::cout << "LINE THINING GRAPH" << std::endl;

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
// 		cv::waitKey(30);
// #endif

		//Intersection or dead end
		if( all_point.size() > 2 || non_dead_end == false){
			
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
// 		cv::waitKey(0);
// #endif
			
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
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
					EdgeType sed;
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
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
					EdgeType sed;
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
void LineFollowerGraph<VertexType, EdgeType>::getNewBranch(Vertex& parent)
{
	if(_dad_vertex.size() > 0){
		parent = _dad_vertex.at(0);
		_dad_vertex.pop_front();
	}
	
// 	LineFollower::getNewBranch();
	
	if(this->_LRP_to_explore.size() > 0){
		//Access new LP RP
		this->_RP = _LRP_to_explore[0].first; 
		this->_LP = _LRP_to_explore[0].second;
		this->_last_drawing_point = _last_drawing_point_deque[0];
		//Remove them
		this->_LRP_to_explore.pop_front();
		this->_last_drawing_point_deque.pop_front();
		
		bool ret = LineFollower::testNewBranchNotBlackandMoveForward();
		if(ret == false){
			getNewBranch(parent);
		}
		
	}
	else{
		_LP.x = -1;
		_LP.y = -1;
	}
	
	
}


template<typename VertexType, typename EdgeType>
void LineFollowerGraph<VertexType, EdgeType>::addPoint2Explore(const std::vector< cv::Point2i >& all_points, const typename bettergraph::PseudoGraph< VertexType, EdgeType>::Vertex& loop)
{
// 	std::cout << "Adding " << all_points.size()/2 << "prints" << std::endl;
	//TODO probably don't need this
	if(all_points.size() >= 2){
		//index goes up
		//dad_index++;
		for(size_t i = 0 ; i < all_points.size() ; i=i+2){
// 			std::cout << "One push" << std::endl;
			_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(all_points[i], all_points[i+1]));
			_dad_vertex.push_back(loop);
			_last_drawing_point_deque.push_back(_last_drawing_point);
			
		}
	}
}



template<typename VertexType, typename EdgeType>
void LineFollowerGraph<VertexType, EdgeType>::clear()
{
	//reset Boost graph
	_graph.clear(); 
	_dad_vertex.clear();
	_line.clear();
	LineFollower::clear();
}

//To slow


template<typename VertexType, typename EdgeType>
bool LineFollowerGraph<VertexType, EdgeType>::loopDetection(cv::Point2i new_p, typename AASS::vodigrex::LineFollowerGraph<VertexType, EdgeType>::Vertex& dad_vertex)
{
	
// 	std::cout << "Total size " << _dad_vertex.size() << std::endl;
	for(size_t i = 0 ; i < _dad_vertex.size(); ++i){
// 		std::cout << "pb :S ? " << _dad_vertex.size() << std::endl;
		if(_graph[_dad_vertex[i]].getX() <= new_p.x + _marge &&
			_graph[_dad_vertex[i]].getX() >= new_p.x - _marge &&
			_graph[_dad_vertex[i]].getY() <= new_p.y + _marge &&
			_graph[_dad_vertex[i]].getY() >= new_p.y - _marge){
			dad_vertex = _dad_vertex[i];
		
// 			std::cout << "LOOP DETECTION YES" << std::endl;
			return true;
		}
		if( ( (_LRP_to_explore[i].first.x + _LRP_to_explore[i].second.x) /2 ) <= new_p.x + _marge &&
			( (_LRP_to_explore[i].first.x + _LRP_to_explore[i].second.x) /2 ) >= new_p.x - _marge &&
			( (_LRP_to_explore[i].first.y + _LRP_to_explore[i].second.y) /2 ) <= new_p.y + _marge &&
			( (_LRP_to_explore[i].first.y + _LRP_to_explore[i].second.y) /2 ) >= new_p.y - _marge){
			dad_vertex = _dad_vertex[i];
		
// 			std::cout << "LOOP DETECTION YES WEIRD" << std::endl;
			
			return true;
		}
	}
// 	std::cout << "NO DETECTION :|" << std::endl;
	return false;
}

template<typename VertexType, typename EdgeType>
void LineFollowerGraph<VertexType, EdgeType>::moveForward()
{
	LineFollower::moveForward();
	cv::Point2i p;
	cv::Size s;
	_W.locateROI(s, p);
	_line.push_back(std::pair<int, int>(p.x + (_W.size().width/2), p.y + (_W.size().height/2)));

}

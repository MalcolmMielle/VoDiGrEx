		
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
		
		//TODO : attention this is dirty
		
		cv::Mat m = _W.clone();
		//Is a dead end
		if(all_point.size() == 2){
//		std::cout << "Adding first vert" << std::endl;
			addVertex(dad);
		_all_time_dad_vertex.push_back(dad);
		_all_time_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(_LP, _RP));
			lineThinningAlgo(dad);
		}
		//Line
		else{
//		std::cout << "Adding first vert nopppe" << std::endl;
			addVertex(dad);
		_all_time_dad_vertex.push_back(dad);
		_all_time_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(_LP, _RP));
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
// 		cv::waitKey(1);
// #endif

		//Intersection or dead end
		if( all_point.size() > 2 || non_dead_end == false){
			Vertex new_dad;
			//I used just a certain number of move but it should use the number of move + the distance travelled by the edge for more robustness. To avoid useless self loops, the number of move needs to be above a certain threshold. Thanks to nature of the algorithm and the moveForward function, it _should_ be ok, by just considering that the bounding box needs to do at all least one jump forward.
			bool already_exist = loopDetection(new_p, new_dad);

			//New intersection
			if(already_exist == false){
				addVertex(dad_vertex, new_dad);
			}
			//Not a new intersection but still an intersection
			else{
				//Should be allowed to have self loop since it's a pseudo graph
				//I used just a certain number of move but it should use the number of move + the distance travelled by the edge for more robustness. To avoid useless self loops, the number of move needs to be above a certain threshold. Thanks to nature of the algorithm and the moveForward function, it _should_ be ok, by just considering that the bounding box needs to do at all least one jump forward.
				if(_line.size() > 1){
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
					EdgeType sed;
					sed.setLine(_line);
					_graph.addEdge(ed, new_dad, dad_vertex, sed);
				}
// 				std::cout << "Clear" << std::endl;
				_line.clear();
			}
			
			addPoint2Explore(all_point, new_dad);
			getNewBranch(dad_vertex);
			
// #ifdef DEBUG
// 		cv::Mat print;
// 		this->_map_in.copyTo(print);
// 		cv::Size s2;
// 		cv::Point2i p_dyn_window2;
// 		_W.locateROI(s2, p_dyn_window2);
// 		second_point = p_dyn_window2;
// 		second_point.x = second_point.x + _W.size().width;
// 		second_point.y = second_point.y + _W.size().height;
// 		cv::rectangle( print, p_dyn_window2, second_point, cv::Scalar( 255 ), 1, 4 );
// 		cv::imshow("tmp", print);
// 		
// 		bettergraph::PseudoGraph<VertexType, EdgeType> graph = this->getGraph();
// 		cv::Mat maa_3 = this->_map_in.clone();
// 		maa_3.setTo(cv::Scalar(0));
// 		AASS::vodigrex::draw<VertexType, EdgeType>(graph, maa_3);
// 		cv::imshow("tmp graph", maa_3);
		
// 		cv::waitKey(0);
// #endif
					
			
		}
		else{				

			//Making sure the line isn't actually a previsouly erased crossing.
			Vertex loop_vertex; 
			bool already_seen = loopDetection(new_p, loop_vertex);
			
			if(already_seen == true){
				//not zero because after switching branch we always move forward
				if(/*loop_vertex != dad_vertex*/ _line.size() > 1 ){
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
					EdgeType sed;
					sed.setLine(_line);
					_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
				}
// 				std::cout << "clear " << std::endl;
				_line.clear();
				dad_vertex = loop_vertex;
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
	
// 	std::cout << "get new branch siz eof line should be zero" << _line.size() << std::endl;
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
			_all_time_LRP_to_explore.push_back(std::pair<cv::Point2i, cv::Point2i>(all_points[i], all_points[i+1]));
			
			_dad_vertex.push_back(loop);
			_all_time_dad_vertex.push_back(loop);
			
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
	
	_all_time_dad_vertex.clear();
	_all_time_LRP_to_explore.clear();
}

//To slow


template<typename VertexType, typename EdgeType>
bool LineFollowerGraph<VertexType, EdgeType>::loopDetection(cv::Point2i new_p, typename AASS::vodigrex::LineFollowerGraph<VertexType, EdgeType>::Vertex& dad_vertex)
{
	
// 	std::cout << "Total size " << _all_time_dad_vertex.size() << std::endl;
	for(size_t i = 0 ; i < _all_time_dad_vertex.size(); ++i){
// 		std::cout << "pb :S ? " << i << " " << _dad_vertex.size() << " " << _graph[_all_time_dad_vertex[i]].getX() << " " << _graph[_all_time_dad_vertex[i]].getY() << " VS " << new_p << std::endl;
		if(_graph[_all_time_dad_vertex[i]].getX() <= new_p.x + _marge &&
			_graph[_all_time_dad_vertex[i]].getX() >= new_p.x - _marge &&
			_graph[_all_time_dad_vertex[i]].getY() <= new_p.y + _marge &&
			_graph[_all_time_dad_vertex[i]].getY() >= new_p.y - _marge){
			dad_vertex = _all_time_dad_vertex[i];
		
// 			std::cout << "LOOP DETECTION YES" << std::endl;
			return true;
		}
		
		//Not marge but inside bounding box _W
		if( ( (_all_time_LRP_to_explore[i].first.x + _all_time_LRP_to_explore[i].second.x) /2 ) <= new_p.x + (_W.cols / 2) &&
			( (_all_time_LRP_to_explore[i].first.x + _all_time_LRP_to_explore[i].second.x) /2 ) >= new_p.x - (_W.cols / 2) &&
			( (_all_time_LRP_to_explore[i].first.y + _all_time_LRP_to_explore[i].second.y) /2 ) <= new_p.y + (_W.rows / 2) &&
			( (_all_time_LRP_to_explore[i].first.y + _all_time_LRP_to_explore[i].second.y) /2 ) >= new_p.y - (_W.rows / 2)){
			dad_vertex = _all_time_dad_vertex[i];
		
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

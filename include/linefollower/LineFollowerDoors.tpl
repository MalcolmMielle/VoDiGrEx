
template<typename VertexType, typename EdgeType>
void LineFollowerDoors<VertexType, EdgeType>::lineThinningAlgo(Vertex& index_dad)
{
	
	
	std::cout << "LINE THINING DOOR" << std::endl;
	
	Vertex dad_vertex = index_dad;
	while(this->_LP.x != -1 && this->_LP.y != -1){

		std::vector<cv::Point2i> all_point;
		bool non_dead_end = this->findNextLPRP(all_point);
		
		cv::Size s;
		cv::Point2i p_dyn_window;
		this->_W.locateROI(s, p_dyn_window);
		
		cv::Point2i new_p;
		new_p.x = p_dyn_window.x + (this->_W.cols / 2);
		new_p.y = p_dyn_window.y + (this->_W.rows / 2);

		//Intersection or dead end
		if( all_point.size() > 2 || non_dead_end == false || wCrossDoor()){
			
			//USE : _all_crossings
			Vertex new_dad;
			bool already_exist = this->loopDetection(new_p, new_dad);
			
			

			//New intersection
			if(already_exist == false){
				this->addVertex(dad_vertex, new_dad);
				if(wCrossDoor()){
					this->_graph[new_dad].setType("keypoint");
				}
				else{
					this->_graph[new_dad].setType("normal");
				}
			}
			//Not a new intersection but still an intersection
			else{
				if(new_dad != dad_vertex){
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
					EdgeType sed;
					sed.setLine(this->_line);
					this->_line.clear();
					this->_graph.addEdge(ed, new_dad, dad_vertex, sed);
				}
			}
			
			this->addPoint2Explore(all_point, new_dad);
			this->getNewBranch(dad_vertex);
					
			
		}
		else{				

			//Making sure the line isn't actually a previsouly erased crossing.
			Vertex loop_vertex; 
			bool already_seen = this->loopDetection(new_p, loop_vertex);
			
			if(already_seen == true){
				if(loop_vertex != dad_vertex){
					typename bettergraph::PseudoGraph<VertexType, EdgeType>::Edge ed;
					EdgeType sed;
					sed.setLine(this->_line);
					this->_line.clear();
					this->_graph.addEdge(ed, loop_vertex, dad_vertex, sed);
					dad_vertex = loop_vertex;
				}
			}
			
			this->_LP = all_point[0];
			this->_RP = all_point[1];
			this->moveForward();		
		}
	}
}


template<typename VertexType, typename EdgeType>
void LineFollowerDoors<VertexType, EdgeType>::clear()
{
	LineFollowerGraph<VertexType, EdgeType>::clear();
	_doors.clear();
}


template<typename VertexType, typename EdgeType>
bool LineFollowerDoors<VertexType, EdgeType>::squareLineCollision (const cv::Point& p1, const cv::Point& p2, float minX, float minY, float maxX, float maxY) {  
	// Completely outside.
	if ((p1.x <= minX && p2.x <= minX) || (p1.y <= minY && p2.y <= minY) || (p1.x >= maxX && p2.x >= maxX) || (p1.y >= maxY && p2.y >= maxY))
		return false;

	//TODO straight line !
	if(p2.x - p1.x != 0 ){
		float m = (p2.y - p1.y) / (p2.x - p1.x);

// 			std::cout << "M " << m << std::endl;
		float y = m * (minX - p1.x) + p1.y;
		if (y > minY && y < maxY) return true;

		y = m * (maxX - p1.x) + p1.y;
		if (y > minY && y < maxY) return true;

		float x = (minY - p1.y) / m + p1.x;
		if (x > minX && x < maxX) return true;

		x = (maxY - p1.y) / m + p1.x;
		if (x > minX && x < maxX) return true;

		return false;
	}
	else{
		if(p1.y <= minY && p2.y >= maxY){
			return true;
		}
		else if(p1.y >= maxY && p2.y <= minY){
			return true;
		}
		else if(p1.y <= maxY && p1.y >= minY){
			if(p2.y > maxY){
				return true;
			}
		}
		else if(p2.y <= maxY && p2.y >= minY){
			if(p1.y < minY){
				return true;
			}
		}
		return false;
		
	}
}

template<typename VertexType, typename EdgeType>
bool LineFollowerDoors<VertexType, EdgeType>::wCrossDoor()
{
	bool res = false;
	
	cv::Point2i p;
	cv::Size s;
	this->_W.locateROI(s, p);
	int height = this->_W.rows;
	int width = this->_W.cols;
	
//  			std::cout << "trying the door detection with " << _doors.size() << std::endl;
	
	for(size_t i = 0 ; i < _doors.size() ; i = i+2){
// 				std::cout << " at : " << _doors[i].x << " at : " << _doors[i].y << std::endl;
// 				std::cout <<  p.x << " " << p.y << " " << p.x + width << " " <<p.y + height << std::endl;
		if(res == false){
			res = squareLineCollision(_doors[i], _doors[i + 1], p.x, p.y, p.x + width, p.y + height);
// 					std::cout << "it is " <<res << std::endl;
		}
		else{
			break;
		}
			
	}
	return res;

}

	
#include "ThinkerEVG.hpp"

void AASS::vodigrex::ThinkerEVG::think(const cv::Mat& map_in)
{
	reset();

	map_in.copyTo(_map_in);
	
// 	std::cout << "Channel " <<_map_in.channels() << std::endl;
	
	cv::Mat chan1;
	if(_map_in.channels() == 3){
		cv::cvtColor(_map_in, chan1, CV_RGB2GRAY);
	}
	else{
		map_in.copyTo(chan1);
	}
// 	std::cout << "Channel " <<chan1.channels() << std::endl;
	
// 	cv::imshow("TEST" , chan1);
// 	cv::waitKey(0);
	
// 	
// 	if(chan1.channels() != 1){
// 		std::cout << "Problem" << std::endl;
// 		exit(1);
// 	}
// 	cv::threshold(chan1, chan1, 50, 255, CV_THRESH_BINARY_INV);
// 	std::cout << "Matrix type" << std::endl;
// 	
// 	type2str(chan1.type());
	
// 	cv::imshow("TEST" , chan1);
// 	cv::waitKey(0);
	
// 	exit(0);
	//Use EVG THINK here
	
	grid_type curr_grid=read_file(chan1, _unknown_min, _unknown_max);
	
	
	if (curr_grid.empty() || curr_grid[0].empty()) {
		std::ostringstream str_test;
		str_test << "Read grid has no dimensions, line" << __LINE__ << " in file " << __FILE__;
		std::runtime_error(str_test.str() );
	}

	if (_robot_locx < 0 || _robot_locx >= int(curr_grid.size()) ||
	_robot_locy < 0 || _robot_locy >= int(curr_grid[0].size())) {
		_robot_locx=curr_grid.size()/2;
		_robot_locy=curr_grid[0].size()/2;
	}
	
	evg_thin thin(curr_grid, _distance_min, _distance_max, _pruning, _robot_close, _robot_locx, _robot_locy);
	
	_skel.clear();
	_skel=thin.generate_skeleton();
	
	draw();

}

grid_type AASS::vodigrex::ThinkerEVG::read_file(const cv::Mat& map,
		     int unknown_min,
		     int unknown_max) {

	column_type col(map.size().height,Unknown);
	grid_type newgrid(map.size().width,col);

	for (int i=0 ; i<map.size().width ; i++){
		for (int j=0 ; j<map.size().height ; j++) {
			int cell= (int) map.at<uchar>(j, i);
// 			std::cout << "CELL " << cell << std::endl;
			if (cell > unknown_max){
				newgrid[i][j]=Free;
			}
			else if (cell < unknown_min){
				newgrid[i][j]=Occupied;
			}
		}
	}

    return newgrid;
  }
  
  
bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge> AASS::vodigrex::ThinkerEVG::getGraph() const
{
	
	/* convert skeleton to GraphList */
	bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge> gl;
	
// 	class node {
//   public:
//   int x,y; //!< location in grid_coords
//   float radius; //!< distance to nearest obstacle (in number of cells)
//   float distance; //!< shortest depth in graph to graph root
//   int parent; 
// 	unsigned int num_children;  
// 	vector<unsigned int> children; //!< if # children > 1, graph branches
	
	std::vector< bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex > vvec;
	
	//Add all vertex
	for(size_t i = 0 ; i < _skel.size() ; ++i){
	
		bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex v;

		vodigrex::SimpleNode sn;
		sn.x = _skel[i].x;
		sn.y = _skel[i].y;
// 		std::string& type, const cv::Mat& m, cv::Point2i p, Vertex& vertex_out
// 		if(_skel[i].num_children == 4){
// 			s = "XCrossing";
// 		}
// 		else if(_skel[i].num_children == 3){
// 			s = "TCrossing";
// 		}
// 		else if(_skel[i].num_children == 3){
// 			s = "NCrossing";
// 		}
		
		gl.addVertex(v, sn);
		vvec.push_back(v);
	}
	
	//Add links
	for(size_t i = 0 ; i < _skel.size() ; ++i){
		for(size_t j = 0 ; j < _skel[i].children.size() ; ++j){
			bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Edge edg;
			gl.addEdge(edg, vvec[i], vvec[ _skel[i].children[j] ]);
		}
	}
	
	return gl;

}



void AASS::vodigrex::ThinkerEVG::draw()
{
	this->_map_result =cv::Mat(_map_in.size().height, _map_in.size().width, CV_32F, cv::Scalar(0,0,0));
	bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge> gl = getGraph();
	
	cv::Scalar color;
	if(this->_map_result.channels() == 1){
		color = 255;
	}
	else if(this->_map_result.channels() == 3){
		color[0] = 255;
		color[1] = 255;
		color[2] = 255; 
	}
	
	for(size_t i = 0 ; i < _skel.size() ; ++i){
		cv::Point2i point_skel;
		point_skel.x = _skel[i].x;
		point_skel.y = _skel[i].y;
		for(size_t j = 0 ; j < _skel[i].children.size() ; ++j){
			cv::Point2i point_skel_child;
			point_skel_child.x = _skel[ _skel[i].children[j] ].x;
			point_skel_child.y = _skel[ _skel[i].children[j] ].y;
			
			cv::line(this->_map_result, point_skel, point_skel_child, color, 5);
		}
	}

}



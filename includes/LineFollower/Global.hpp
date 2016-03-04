// #ifndef GLOBALVARIABLE_MAP
// #define GLOBALVARIABLE_MAP
// 
// #include <opencv2/opencv.hpp>
// // #include "Gateway.hpp"
// #include "boost/graph/adjacency_list.hpp"
// #include "boost/graph/topological_sort.hpp"
// #include "boost/graph/filtered_graph.hpp"
// #include "boost/function.hpp"
// 
// namespace AASS{
// 
// 	inline void printer(const std::vector<unsigned int>& p){
// 		
// 		std::vector<unsigned int>::const_iterator it;
// 		for(it = p.begin() ; it != p.end() ; it++){
// 			std::cout << *it << " " ;
// 		}
// 	}
// 
// 	inline void printer(const std::vector< std::vector<unsigned int> >& p){
// 		
// 		std::vector< std::vector<unsigned int> >::const_iterator it;
// 		for(it = p.begin() ; it != p.end() ; it++){
// 			printer(*it);
// 			std::cout << std::endl;
// 		}
// 		
// 	}
// 
// // 	/**
// // 	* @brief Levenshtein distance between two string
// // 	* 
// // 	* see: http://stackoverflow.com/questions/5849139/levenshtein-distance-inferring-the-edit-operations-from-the-matrix 
// // 	* 
// // 	* https://en.wikibooks.org/wiki/Algorithm_Implementation/Strings/Levenshtein_distance#C.2B.2B
// // 	* 
// // 	* @param[in] string_to_modify : input string
// // 	* @param[in] unchanged : model string
// // 	* @param[in] out : string describing the modification necesseary to go from input string to output string
// // 	* 
// // 	* @return edit-distance
// // 	*/
// // 
// // 	inline unsigned int levenshtein_distance(const std::string& string_to_modify, const std::string& unchanged, std::string& out) 
// // 	{
// // 		
// // 		out.clear();
// // 		
// // 	// 	std::cout << "COMPARING " << string_to_modify << " and " << unchanged << std::endl;
// // 		const std::size_t len1 = string_to_modify.size(), len2 = unchanged.size();
// // 		std::vector<unsigned int> col(len2+1), prevCol(len2+1);
// // 		
// // 		std::vector< std::vector < unsigned int > > mat;
// // 
// // 		for (unsigned int i = 0; i < prevCol.size(); i++){
// // 			prevCol[i] = i;
// // 		}
// // 		for (unsigned int i = 0; i < len1; i++) {
// // 			col[0] = i+1;
// // 			for (unsigned int j = 0; j < len2; j++){
// // 				col[j+1] = std::min( 
// // 							std::min ( prevCol[1 + j] + 1, col[j] + 1), 
// // 							prevCol[j] + (string_to_modify[i]==unchanged[j] ? //condition
// // 							0 : //If true 
// // 							1 //if false		
// // 							) 
// // 						);
// // 			}
// // 
// // 			col.swap(prevCol);
// // 			mat.push_back(col);
// // 		}
// // 		
// // 		mat.push_back(prevCol);
// // 		
// // 	// 	for(int i = 0 ; i < mat.size() ;i++){
// // 	// 	
// // 	// 		for(int j = 0 ; j < mat[i].size() ; j++){
// // 	// 			std::cout << mat[i][j] << " ";
// // 	// 		}
// // 	// 		std::cout << std::endl;
// // 	// 	}
// // 		
// // 
// // 		/*Get the order of modification :
// // 		* an horizontal move is an INSERTION of a letter from the 't string'
// // 		* an vertical move is a DELETION of a letter from the 's string'
// // 		* a diagonal move is either:
// // 		* 	a no-operation (both letters at respective positions are the same); the number doesn't change
// // 		*  a SUBSTITUTION (letters at respective positions are distinct); the number increase by one.
// // 		* 
// // 		*/
// // 		
// // 		int cell_x = len2;
// // 		int cell_y = len1;
// // 		
// // 		int cell;
// // 		
// // 		int cell_above;
// // 		int cell_left;
// // 		int cell_diag;
// // 		
// // 		while(cell_x != 0 || cell_y != 0){
// // 			
// // 			cell = mat.at(cell_y).at(cell_x);
// // 			if(cell_y - 1 >= 0){
// // 				cell_above = mat.at(cell_y - 1).at(cell_x);
// // 			}
// // 			else{
// // 				cell_above = - 1;
// // 			}
// // 			
// // 			if(cell_x - 1 >= 0){
// // 				cell_left = mat.at(cell_y).at(cell_x - 1);
// // 			}
// // 			else{
// // 				cell_left = - 1;
// // 			}
// // 			
// // 			if(cell_y - 1 >= 0 && cell_x - 1 >= 0){
// // 				cell_diag = mat.at(cell_y - 1).at(cell_x - 1);
// // 			}
// // 			else{
// // 				cell_diag = - 1;
// // 			}
// // 			
// // 	// 		std::cout << "all values " << cell << " above " << cell_above  << " diag " << cell_diag << " left " << cell_left << std::endl;
// // 		
// // 			if(cell_diag == -1){
// // 				if(cell_above == -1){
// // 					cell_x = cell_x - 1;
// // 					if(cell_left == cell || cell_left == cell-1){
// // 	// 					std::cout << "insertion" << std::endl;
// // 						out = out + "i";
// // 					}
// // 				}
// // 				else{
// // 	// 				std::cout << "deletion" << std::endl;
// // 					cell_y = cell_y - 1;
// // 					out = out + "d";
// // 				}
// // 				
// // 			}
// // 			else{
// // 				if(cell_diag <= cell_above && cell_diag <= cell_left){
// // 					if(cell_diag == cell - 1){
// // 	// 					std::cout << "substitution" << std::endl;
// // 						out = out + "s";
// // 					}
// // 					else{
// // 	// 					std::cout << "nothing" << std::endl;
// // 						out = out + "n";
// // 					}
// // 					
// // 					cell_x = cell_x - 1;
// // 					cell_y = cell_y - 1;
// // 					
// // 				}
// // 				else if(cell_left <= cell_above){
// // 					if(cell_left == cell || cell_left == cell-1){
// // 	// 					std::cout << "insertion" << std::endl;
// // 						out = out + "i";
// // 					}
// // 					cell_x = cell_x - 1;
// // 				}
// // 				else{
// // 	// 				std::cout << "deletion" << std::endl;
// // 					out = out + "d";
// // 					cell_y = cell_y - 1;
// // 				}
// // 			}
// // 		}
// // 		
// // 	// 	printer(mat);
// // 		std::reverse(out.begin(), out.end());
// // 		
// // 	// 	std::cout << "RESULT : " << out << std::endl;
// // 	// 	int a;
// // 	// 	std::cin >> a;
// // 		
// // 		return prevCol[len2];
// // 	}
// 
// 
// 
// 	namespace VoDiGrEx{
// 		
// 	// 	struct Place;
// 	// 	struct Intersection_Graph;
// 	// 	struct Gateway_struct;
// 		
// 			
// 		struct Intersection_Graph;
// 		
// 		
// 		struct EdgeElement{
// 			int number;
// 			EdgeElement() : number(0){};
// 		};
// 		
// 		/* 
// 		* Typedef for voronoi
// 		*/
// 		typedef boost::adjacency_list<
// 			boost::listS, boost::listS, boost::undirectedS, 
// 			topologicalmap::Intersection_Graph,
// 			EdgeElement, 
// 			boost::no_property > Graph_boost;
// 			
// 	// 	typedef typename boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
// 		typedef typename boost::graph_traits<Graph_boost>::vertex_iterator VertexIterator;
// 		typedef typename boost::graph_traits<Graph_boost>::vertex_descriptor Vertex;
// 		typedef typename boost::graph_traits<Graph_boost>::edge_descriptor Edge;
// 		typedef typename boost::graph_traits<Graph_boost>::out_edge_iterator EdgeIterator;
// 		
// 		///@brief Vertex type of GraphList
// 		struct Intersection_Graph{
// 			std::string type;
// 			cv::Mat mat;
// 			cv::Point2i point;
// 			int index;
// 			int force;
// 		};
// 		
// 	}
// 	
// 	
// 
// }
// 
// #endif
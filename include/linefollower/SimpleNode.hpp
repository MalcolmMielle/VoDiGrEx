#ifndef SIMPLENODE_LINEFOLLOWER_POINT_MAP_07032016
#define SIMPLENODE_LINEFOLLOWER_POINT_MAP_07032016

#include <vector>

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
		class SimpleNode{
			
		public :
			float x;
			float y;
			SimpleNode(){};
			SimpleNode(const SimpleNode& s){x = s.getX();y = s.getY();}
			float getX() const {return x;}
			float getY() const {return y;}
			void setX(float xx){ x = xx;}
			void setY(float yy){ y = yy;}
			void setPoint(float xx, float yy){ x = xx; y = yy;}
		};

		class SimpleEdge{
		public:
			std::vector< std::pair<int, int> > line;
			
			SimpleEdge(){};
			
			void addEdgePosition(int x, int y) {line.push_back(std::pair< int, int> (x, y));}
			void setLine(const std::vector< std::pair<int, int> >& l){line = l;}
			
			std::vector< std::pair<int, int> >& getLine(){return line;}
			const std::vector< std::pair<int, int> >& getLine() const {return line;}
		};
		
		inline std::ostream& operator<<(std::ostream& in, const SimpleNode &p){
			in << p.x << " " ; in << p.y; return in;
		}

		
		inline std::ostream& operator<<(std::ostream& in, const SimpleEdge &p){
// 			for(size_t i = 0 ; i < p.getLine().size() ; ++i){
// 				in << p.getLine()[i].first ; in << p.getLine()[i].second; 
// 			}
			return in;
		}

		
	}
}

#endif
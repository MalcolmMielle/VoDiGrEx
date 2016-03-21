#ifndef SIMPLENODENAMED_LINEFOLLOWER_POINT_MAP_21032016
#define SIMPLENODENAMED_LINEFOLLOWER_POINT_MAP_21032016

#include <vector>
#include "SimpleNode.hpp"

namespace AASS{
		
	namespace vodigrex{

		class SimpleNodeNamed : public SimpleNode{
		public:
			std::string type;
			void setType(const std::string& str){type = str;}
			std::string getType() const {return type;}
		};
	}
}

#endif
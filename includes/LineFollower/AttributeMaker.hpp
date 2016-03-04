#ifndef VODIGREX_ATTRIBUTEMAKER_ADDER
#define VODIGREX_ATTRIBUTEMAKER_ADDER

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <stdexcept>
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"
#include <boost/graph/copy.hpp>
#include <math.h>
	
namespace AASS{
	namespace VoDiGrEx{
		
		
		class LineFollower{};
		
		template<typename VertexType, typename EdgeType>
		class AttributeMaker{
		protected : 
			typedef boost::adjacency_list<
				boost::listS, boost::listS, boost::undirectedS, 
				VertexType,
				EdgeType, 
				boost::no_property > GraphType;
			typedef typename boost::graph_traits<GraphType>::vertex_iterator VertexIterator;
			typedef typename boost::graph_traits<GraphType>::vertex_descriptor Vertex;
			typedef typename boost::graph_traits<GraphType>::edge_descriptor Edge;
			typedef typename boost::graph_traits<GraphType>::out_edge_iterator EdgeIterator;
			
		public :
			AttributeMaker(){};
			virtual VertexType make(const LineFollower& lin) = 0;
			virtual EdgeType make(const LineFollower& lin) = 0;
			
		};
	
	}
	
}

#endif
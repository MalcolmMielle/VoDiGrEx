#ifndef THINKER_EVG_MAPMAKER
#define THINKER_EVG_MAPMAKER

// #include "Thinker.hpp"

// #include "SketchMap.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "Evg_thin/evg-thin.hh"
#include "Evg_thin/utils.hh"

#include "bettergraph/PseudoGraph.hpp"
#include "linefollower/SimpleNode.hpp"
#include "Util.hpp"

#include "voronoidiagram/Thinker.hpp"

namespace AASS{
	
	namespace vodigrex{
	
		
		class ThinkerEVG : public AASS::vodigrex::Thinker{
		protected:
			cv::Mat _voronoi;
			skeleton_type _skel;
			int _unknown_min;
			int _unknown_max;
			bool _pruning;
			bool _robot_close;
			float _distance_min;
			float _distance_max;
			int _robot_locx;
			int _robot_locy;

			
		public:
			ThinkerEVG() : Thinker(), 
				_unknown_min(220), 
				_unknown_max(240), 
				_pruning(true), 
				_robot_close(false), 
				_distance_min(0.0), 
				_distance_max(FLT_MAX), 
				_robot_locx(-1), 
				_robot_locy(-1) {
			};
			
			virtual void think(const cv::Mat& map_in);
			bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge> getGraph() const;
			void reset(){
				Thinker::reset();
				_voronoi = cv::Scalar(0,0,0);
				_skel.clear();
			}
			
			void setPruning(bool p){_pruning = p;}
			void setUnknown(int min, int max){_unknown_min = min, _unknown_max = max;}
			void setRobotLoc(float x, float y){_robot_locx = x; _robot_locy = y;}
			
		private:
			grid_type read_file(const cv::Mat& map, int unknown_min, int unknown_max);
			void draw();
			
			
		};
	}
}



#endif
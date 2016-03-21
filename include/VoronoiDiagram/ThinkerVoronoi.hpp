#ifndef THINKER_VORNOI_MAPMAKER
#define THINKER_VORNOI_MAPMAKER

#include "Thinker.hpp"

//TODO : remove this !
#include "opencv2/imgproc/imgproc.hpp"

namespace AASS{
	namespace vodigrex{
	
		
		class ThinkerVoronoi : public Thinker{
		protected:
			cv::Mat _voronoi;
			cv::Mat _label;

			
		public:
			ThinkerVoronoi() : Thinker() {
			};
			
			virtual void think(const cv::Mat& map_in);	
			virtual void reset(){
				Thinker::reset();
				_voronoi = cv::Scalar(0,0,0);
				_label = cv::Scalar(0,0,0);
			}
			
			virtual void modePartialNormalized(){ setMode(0);}
			virtual void modeSobelLabel(){ setMode(1);}
			virtual void modeLaplaceLabel(){ setMode(2);}
			virtual void modeCannyVoro(){ setMode(3);}
			virtual void modeLaplaceVoro(){ setMode(4);}
			virtual void modeLocalMaxima(){ setMode(5);}
			virtual void modeLocalMaximaBest(){ setMode(6);}
			virtual void modeLocalMaximaCombo(){setMode(8);}
			virtual void modeDelaunay(){ setMode(7);}
			virtual void modeSkeleton(){ setMode(8);}
			
			virtual cv::Mat& getVoronoi(){return _voronoi;}
			virtual void voronoi(const cv::Mat& mapin);
			
		private:
			virtual void voronoiSobelLabel();
			virtual void voronoiCannyVoro();
			virtual void voronoiLaplaceVoro();
			virtual void voronoiLaplaceLabel();
		// 	virtual void delaunay(const std::vector<cv::Point2f>& obst);
			
			virtual void localMaxima();
			virtual void localMaximaBest();
			virtual void localMaximaCombo();
			virtual void partialNormalize(cv::Mat& mat_in);
			
			virtual void skeleton();
			
		};

	}
}


#endif
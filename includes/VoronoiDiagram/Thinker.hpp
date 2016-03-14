#ifndef THINKER_MAPMAKER
#define THINKER_MAPMAKER

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace AASS{
	
	namespace vodigrex{

		class Thinker{
		protected:
			cv::Mat _map_in;
			cv::Mat _map_result;
			int _mode;
			int _threshold_local_max;
			float _sharpner;
			int _dilate;
			int _downSample;
			int _level;
			
		public:
			Thinker() : _mode(4), _threshold_local_max(0.5), _sharpner(0), _dilate(0), _downSample(1), _level(30){};
			

			virtual void think(const cv::Mat& map_in)=0;
			virtual int getMode() const {return _mode;}
			virtual void setMode(int m){_mode = m;}
			virtual void setThresholdLocalMax(int t){_threshold_local_max = t;}
			virtual int getThresholdLocalMax(){return _threshold_local_max;}
			virtual float getSharpner(){return _sharpner;}
			virtual void setSharpner(float f){_sharpner = f;}
			virtual int getDilate(){return _dilate;}
			virtual void setDilate(int d){_dilate = d;}
			virtual int getDownSample(){return _downSample;}
			virtual void setDownSample(int d){_downSample = d;}
			virtual void setLevel(int l){_level = l;}
			
			virtual void reset(){
				_map_in = cv::Scalar(0,0,0);
				_map_result = cv::Scalar(0,0,0);
			};
			virtual void dilateErode(cv::Mat& m);
			
			virtual const cv::Mat& getResult() const {return _map_result;}
			virtual const cv::Mat& getMapIn() const {return _map_in;}
			
		};

		inline void Thinker::dilateErode(cv::Mat& m)
		{
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
			
			for(int i = 0; i<_dilate ; i++){
			
				cv::erode(m, m, kernel);
			}
			
		}
	}
}
#endif
#include "ThinkerVoronoi.hpp"

void AASS::vodigrex::ThinkerVoronoi::think(const cv::Mat& map_in)
{  
	reset();

	map_in.copyTo(_map_in);
// 	cv::imshow("That voroinit", _map_in);
// 	cv::imshow("That voroinit in", map_in);
// 	cv::waitKey(0);
	
	std::cout << "simple asign" << std::endl;
	if(_mode == 0){
		voronoi(_map_in);
		partialNormalize(_voronoi);
	}
// 	this->_map_result = _voronoi;
	if(_mode == 1){
		voronoi(_map_in);
		voronoiSobelLabel();
	}
	if(_mode == 2){
		voronoi(_map_in);
		voronoiLaplaceLabel();
	}
	//Nope and nope those function are uterly useless because Voronoi distance is not sharp enough
	if(_mode == 3){
		voronoi(_map_in);
		voronoiCannyVoro();
	}
	if(_mode == 4){
		voronoi(_map_in);
		voronoiLaplaceVoro();
	}
	if(_mode == 5){
		voronoi(_map_in);
		localMaxima();
	}
	if(_mode == 6){
		voronoi(_map_in);
		localMaximaBest();
	}
// 	if(_mode == 7){
// 		std::cout << "going for delaunay" << std::endl;
// 		delaunay();
// 	}
	if(_mode == 8){
		voronoi(_map_in);
		localMaximaCombo();
	}
	if(_mode == 9){
		skeleton();
	}
	
	dilateErode(this->_map_result);
}

void AASS::vodigrex::ThinkerVoronoi::voronoi(const cv::Mat& mapin)
{
	
	std::cout << "channels : " << mapin.channels() << std::endl;
// 	cv::imshow("That voro", _map_in);
// 	cv::waitKey(0);
	
	if(mapin.channels() == 3){
		cv::cvtColor(mapin, _voronoi, CV_RGB2GRAY);
	}
	else{
		mapin.copyTo(_voronoi);
	}
	
// 	cv::imshow("Src", _map_in);
// 	cv::waitKey(0);
	
	cv::threshold(_voronoi, _voronoi, 50, 255, CV_THRESH_BINARY_INV);
	
	std::cout << "distance transform" << std::endl;
	//Laplace use a pixel distance instead of the Euclidean one
	if(this->_mode == 4){
// 		Pixel distance give way to good result for sketch map but is PERFECT for building maps.
// 		cv::distanceTransform(_voronoi, _voronoi, _label, CV_DIST_C, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_CCOMP);
		cv::distanceTransform(_voronoi, _voronoi, _label, CV_DIST_L2, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_CCOMP);
	}
	//Euclidean distance
	else{
		cv::distanceTransform(_voronoi, _voronoi, _label, CV_DIST_L2, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_CCOMP);
	}
	
// 	cv::normalize(_voronoi, _voronoi, 0, 1., cv::NORM_MINMAX);
// 	cv::imshow("That voro", _voronoi);
// 	cv::waitKey(0);

	//0 - 200 seems pretty good as an interval
	cv::normalize(_label, _label, 0, 200, cv::NORM_MINMAX);
	
	//std::cout << _label << "Tdammmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm"<<std::endl;
	
	//Make it more sharp.
	cv::Mat tmp;
	cv::GaussianBlur(_voronoi, tmp, cv::Size(0, 0), 3);
	cv::addWeighted(_voronoi, 1.5, tmp, _sharpner, 0, _voronoi);
	
	if(this->_mode != 4){
		cv::normalize(_voronoi, _voronoi, 0, 1., cv::NORM_MINMAX);
	}
	
// 	cv::imshow("voronoi", _voronoi);
// 	cv::waitKey(0);
	
}

/****************************************USING LABEL*********************************/

/*Function that return Voronoi Lines using a Sobel filter
 */
void AASS::vodigrex::ThinkerVoronoi::voronoiSobelLabel()
{
	
	
	cv::Mat sobelX;
	cv::Mat sobelY;
	//int ddepth = CV_16S;
	int scale = 1;
	int delta = 0;
	/// Gradient X
	_label.convertTo(_label,CV_32F);
	
	cv::Sobel(_label, sobelX, -1, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
/// Gradient Y
	cv::Sobel(_label, sobelY, -1, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::addWeighted( sobelX, -0.5, sobelY, -0.5, 0, this->_map_result );
	
	/*std::string name5("Sobel");
	cv::namedWindow(name5, 1);
	cv::imshow(name5, this->_map_result);*/
	
}

/*Function that return Voronoi Lines using a Laplace filter
 */
void AASS::vodigrex::ThinkerVoronoi::voronoiLaplaceLabel()
{
	
	_label.convertTo(_label,CV_32F);
	cv::Laplacian(_label, this->_map_result, -1);
	
}




/*****************************USING DIRECTLY VORONOI IMAGE************************/

void AASS::vodigrex::ThinkerVoronoi::voronoiCannyVoro()
{
	cv::normalize(_voronoi, _voronoi, 0, 255, cv::NORM_MINMAX);
	_voronoi.convertTo(_voronoi, CV_8U);
	
	cv::Canny(_voronoi, this->_map_result, _threshold_local_max, 3*_threshold_local_max, 3 );
	
}


void AASS::vodigrex::ThinkerVoronoi::voronoiLaplaceVoro()
{
	//Here erode 2 time is super cool !
	_voronoi.convertTo(_voronoi,CV_32F);
	
	for(int i = 0; i < this-> _downSample ; i++){
		cv::pyrDown(_voronoi, _voronoi, cv::Size(_voronoi.cols/2, _voronoi.rows/2 ) );
	}
	
	
	/*
	 * Those two method are the same
	 * 
	 * METHOD 1
	 */
	cv::Laplacian(_voronoi, this->_map_result, -1);
	
// 	cv::Mat cop;
// 	this->_map_result.copyTo(cop);
// 	cv::normalize(cop, cop, 0, 255, cv::NORM_MINMAX);
	
// 	cv::Mat invSrc =  cv::Scalar::all(255) - cop;
// 	cv::normalize(invSrc, invSrc, 0, 1., cv::NORM_MINMAX);
	
// 	cv::imshow("Laplacian", invSrc);
// 	cv::waitKey(0);

	/*
	* METHOD 2
	*/
// 	int kernel_size = 3;
// 	cv::Mat kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F );
// 	kernel.at<float>(1,1) = -8;
// 	std::cout << "Kernel : " << kernel << std::endl;
// 	cv::filter2D(_voronoi, this->_map_result, -1 , kernel, cv::Point( -1, -1 ), 0, cv::BORDER_DEFAULT );
// 	std::cout << "channels " <<_voronoi.channels() << " " << this->_map_result.channels()<<std::endl;

// 	cv::imshow("laplace", this->_map_result);
	/*
	 * Method is :
	 * We use a kernel as such if we use Lapace :
	 * [0 1 0
	 * 	1 -4 1
	 * 	0 1 0]
	 * Every local maxima is thus, strictly less than 0
	 * This give pretty good voronoi lines and is super duper fast
	 */
	
	/*
	 * Using 70 - 30 strategy for voronoi lines (we let 70% of the points)
	 * InputArray src, double* minVal, double* maxVal=0, Point* minLoc=0, Point* maxLoc=0, InputArray mask=noArray()
	 */
	
	//Fixing the level depending on the minima of the image.
	double min;
	double max;
	cv::Point2i p_minloc;
	cv::Point2i p_maxloc;
	cv::minMaxLoc(this->_map_result, &min, &max, &p_minloc, &p_maxloc);
	std::cout << "partial calculation is " << (this->_level * (-min) ) / 100 << std::endl;
	double level_tmp = 1 / ( (this->_level * (-min) ) / 100);
	std::cout << "level is "<< level_tmp << " with min " << min <<  std::endl;
	
	for (int i = 0; i < this->_map_result.rows; ++i) {
		for (int j = 0; j < this->_map_result.cols; ++j) {
			//remove positive values
			if(this->_map_result.at<float>(i, j)>0){
				this->_map_result.at<float>(i, j) = 0;
			}
			else{
				//Magic bit
				//this remove lots of value close to zero but not zero. like 5.96046e-08 but keep 0.00803226 and that's what we want.
				
				/*
				 * This only keep every value LESS than the value of the percentage of min we want. For exmaple, _level = 30, we remove every value above 30% of minima and keep the remaining 70%.
				 * This is how it work : 
				 * * level_tmp is 1/(percentage of minima we want)
				 * * pixel value * level_tmp > 1 if pixel value is more than 1/(percentage of minima we want)
				 * * pixel value * level_tmp < 1 if pexel value is less than 1/(percentage of minima we want)
				 * * int level is the new value of pixel value * level_tmp. Thus level is 0 is pixel value * level_tmp is less than 1 and level is more than 0 if pixel value * level_tmp is more than one.
				 * * pixel_value is replace by level.
				 * 
				 * And then the full imaged is treshold keeping all value above 0.
				 * 
				 * Propotionality tables
				 * 
				 * value pix   perc
				 * 30max       30
				 * -min(max)   100
				 * 
				 * 30max       1
				 * val         more than 30max > 1 ; less than 20max < 1
				 */
				
				int level =  level_tmp * ( - this->_map_result.at<float>(i, j) );
				this->_map_result.at<float>(i, j) = level;
			}
		}
	}
	cv::threshold(this->_map_result, this->_map_result, 0, 255, CV_THRESH_BINARY);
	this->_map_result.convertTo(this->_map_result, CV_8U);
	
// 	cv::imshow("EVD", this->_map_result);
// 	cv::waitKey(0);
	
// 	std::cout << "FINAL  " << this-> _map_result;
		
}


/*********************** DELAUNAY *****************************************/



/*Function to calculate delaunay triangle. Intersection are voronoi points. Problem : too many points
 */
// inline void ThinkerVoronoi::delaunay(const std::vector<cv::Point2f>& obst)
// {
// 	
// 	//TODO Need to convert into grayscale ??
// 	std::cout << "Dub div a" << std::endl;
// 	exit(0);
// 	int rows = _map_in.size().width;
// 	std::cout << "Dub div aa " << std::endl;
// 	int cols = _map_in.size().height;
// 	std::cout << "Dub div aaa" << std::endl;
// 	cv::Rect rect(0, 0, rows, cols);
// 	std::cout << "Dub div" << std::endl;
// 	cv::Subdiv2D subdiv(rect);
// 	std::cout << "Dub div2" << std::endl;
// 	this->_map_result = cv::Mat(rect.size(), CV_8UC3);
// 	std::cout << "Dub div3" << std::endl;
// 	this->_map_result = cv::Scalar::all(0);
// 	std::cout << "Dub div 4" << std::endl;
// 	//Insert all points inside the map
// 	for(size_t i=0; i<obst.size();i++){
// 		if(obst[i].x <rows && obst[i].x >0 && obst[i].y <cols && obst[i].y>0){
// 			subdiv.insert(obst[i]);
// 		}
// 	}
// 	std::cout << "Dub div after" << std::endl;
// 	std::vector<std::vector<cv::Point2f> > facets;
//     std::vector<cv::Point2f> centers;
//     subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);
// 	
// 	std::vector<cv::Point> ifacet;
//     std::vector<std::vector<cv::Point> > ifacets(1);
// 	
// 	std::vector<int> count_points;
// 	std::vector<cv::Point> points;
// 	
// 	//Take the points of each Vornoi facet and print them
// 	for( size_t i = 0; i < facets.size(); i++ )
//     {
//         ifacet.resize(facets[i].size());
//         for( size_t j = 0; j < facets[i].size(); j++ ){
//             ifacet[j] = facets[i][j];
// 			
// 			//Check if we already saw the point before. If yes increment the count. If not add it to the list
// 			//Problem, it's never more than 3... since we have a lots of points and not line there is a lot of intersection
// 			/*******************************************************************/
// 			int flag=0;
// 			if(ifacet[j].x <rows && ifacet[j].x >0 && ifacet[j].y <cols && ifacet[j].y>0){
// 				//std::cout << "Good point"<<std::endl;
// 				for(size_t z=0;z<points.size();z++){
// 					if(ifacet[j].x == points[z].x && ifacet[j].y == points[z].y){
// 						flag=flag+1;
// 						count_points[z]=count_points[z]+1;
// 					}
// 				}
// 				if(flag == 0){
// 					points.push_back(ifacet[j]);
// 					count_points.push_back(1);
// 				}
// 			}
// 			
// 			/*******************************************************************/
// 			
// 		}
// 		//Draw intersection points
// 		for(size_t z=0; z < ifacet.size();z++){
// 			cv::circle(this->_map_result,ifacet[z], 3, cv::Scalar(255, 0, 255), -1);
// 		}
// 		//Fill voronoi zones
//         /*cv::Scalar color;
//         color[0] = rand() & 255;
//         color[1] = rand() & 255;
//         color[2] = rand() & 255;
//         cv::fillConvexPoly(this->_map_result, ifacet, color, 8, 0);
// 		*/
//         ifacets[0] = ifacet;
//         //cv::polylines(this->_map_result, ifacets, true, cv::Scalar(255, 255, 255), 1);
//         cv::circle(this->_map_result, centers[i], 3, cv::Scalar(255, 255, 255), -1);
//     }
//     
//     //Print junction points.
//     //std::cout << "List of points size" << points.size() << std::endl;
//     for (size_t i =0; i< points.size();i++){
// 		//std::cout << "x : "<<points[i].x <<" y : "<<points[i].y << "seen : "<<count_points[i]<< std::endl;
// 		if(count_points[i] >= 3){
// 			cv::circle(this->_map_result, points[i], 3, cv::Scalar(255, 0, 255), -1);
// 		}
// 	}
//    
//    // cv::imshow( win, this->_map_result );
// 	
// }






void AASS::vodigrex::ThinkerVoronoi::localMaxima()
{

	// accept only char type matrices
    CV_Assert(_voronoi.depth() != sizeof(uchar));
	float val_min, val_max;
	
	float max = 1;
	float min =0;
	
	this->_map_result =cv::Mat(_voronoi.size().height, _voronoi.size().width, CV_32F, cv::Scalar(0,0,0));
	
	cv::MatIterator_<float> it = _voronoi.begin<float>();
	cv::MatIterator_<float> new_mat= this->_map_result.begin<float>();
	
	
	//Horizontal maxima
	for( int i = 0; i< (this->_map_result.size().width) ;i++){
		for(int j= 1;j< this->_map_result.size().height-1;j++){
			it++;
			val_min = (float)(*(it-1));
			val_max = (float)(*(it+1));
			if(val_min <= (*it) && val_max<=(*it) && (*it)>_threshold_local_max){
				//It's a local maxima
				(*new_mat) = max;
			}
			else{
				(*new_mat) = min;
			}
				//std::cout << "the max min it :"<<val_max<<" "<<val_min<<" "<<(float)(*it)<<std::endl;
				//(*new_mat) = 1;
				//std::cout << (float) (*new_mat) << ", ";
			new_mat++;
			}
		//std::cout <<std::endl;
	}
	
	//Vertical maxima
	it = _voronoi.begin<float>();
	new_mat= this->_map_result.begin<float>();
	
	for( int i = 1; i< (this->_map_result.size().width-1) ;i++){
		for(int j= 1;j< this->_map_result.size().height;j++){
			it++;
			val_min = (float)(*(it-this->_map_result.size().width));
			val_max = (float)(*(it+this->_map_result.size().width));
			if(val_min <= (*it) && val_max<=(*it) && (*new_mat) == max){
				//It's a local maxima
				(*new_mat) = max;
			}
			else{
				(*new_mat) = min;
			}
			new_mat++;
		}
		//std::cout <<std::endl;
	}
	//std::cout << std::endl << std::endl;
	
}



void AASS::vodigrex::ThinkerVoronoi::localMaximaBest()
{
	// accept only char type matrices
    CV_Assert(_voronoi.depth() != sizeof(uchar));
	float val_min, val_max, val_max_h, val_min_h;
	
	float max = 1;
	float min =0;
	
	
	this->_map_result =cv::Mat(_voronoi.size().height, _voronoi.size().width, CV_32F, cv::Scalar(0,0,0));
	
	cv::MatIterator_<float> it = _voronoi.begin<float>();
	cv::MatIterator_<float> new_mat= this->_map_result.begin<float>();
	it = it+_voronoi.size().width;
	new_mat = new_mat+this->_map_result.size().width;
	//Horizontal maxima
	for( int i = 0; i< (this->_map_result.size().width) ;i++){
		for(int j= 1;j< this->_map_result.size().height-1;j++){
			it++;
			val_min = (float)(*(it-1));
			val_max = (float)(*(it+1));			
			val_min_h = (float)(*(it-this->_map_result.size().width));
			val_max_h = (float)(*(it+this->_map_result.size().width));
			if( ( (val_min < (*it) && val_max<(*it) ) || ( ( val_min_h < (*it) ) && val_max_h <(*it) ) ) && (*it)>_threshold_local_max){
				//It's a local maxima
				(*new_mat) = max;
			}
			//else if( val_min < (*it) && val_max<(*it) && val_min_h < (*it) && val_max_h <(*it) && (*it)>_threshold_local_max){
// 				(*new_mat) = max;
// 			}
			else{
				(*new_mat) = min;
			}
			//TODO should be at the begining.
			new_mat++;
		}
		//std::cout <<std::endl;
	}
	
}

void AASS::vodigrex::ThinkerVoronoi::partialNormalize(cv::Mat& mat_in)
{
	cv::Mat ROI;
	this->_map_result =cv::Mat(_voronoi.size().height, _voronoi.size().width, CV_32F, cv::Scalar(0,0,0));
	
	int step_width = mat_in.size().width /5;
	int step_height = mat_in.size().height /5;
	for (int i=0; i< mat_in.size().width;i = i + step_width){
		for (int j=0; j<mat_in.size().height;j= j + step_height){
			ROI = mat_in(cv::Rect(i, j, step_width, step_height));
			cv::normalize(ROI, ROI, 0, 1., cv::NORM_MINMAX);
			
			/***********************************************************
			* Clauclate maxima per sections*
			* **********************************************************/
			float val_min, val_max;
	
			float max = 1;
			float min =0;
			
			cv::Mat Final_ROI;
			Final_ROI = this->_map_result(cv::Rect(i, j, step_width, step_height));
		
			cv::MatIterator_<float> it = ROI.begin<float>();
			cv::MatIterator_<float> new_mat= Final_ROI.begin<float>();
			
			//Horizontal maxima			
			for( int i = 0; i< (Final_ROI.size().width) ;i++){
				for(int j= 1;j< Final_ROI.size().height-1;j++){
					it++;
					val_min = (float)(*(it-1));
					val_max = (float)(*(it+1));
					if(val_min <= (*it) && val_max<=(*it) && (*it)>_threshold_local_max){
						//It's a local maxima
						(*new_mat) = max;
					}
					else{
						(*new_mat) = min;
					}
					new_mat++;
				}
			}
			
			//Vertical maximas
			it = ROI.begin<float>();
			it = it+ROI.size().width;
			new_mat= Final_ROI.begin<float>();
			new_mat = new_mat + +Final_ROI.size().width;
			//for( int i = 1; i< (Final_ROI.size().width-1) ;i++){
			for(int i = 0; i< (Final_ROI.size().width) ;i++){
				for(int j= 1;j< Final_ROI.size().height-1;j++){
					it++;
					val_min = (float)(*(it-step_width));
					//This part is good
					val_max = (float)(*(it+step_width));

					if(val_min <= (*it) && val_max<=(*it) && (*new_mat) == max){
						//It's a local maxima
						(*new_mat) = max;
					}
					else{
						(*new_mat) = min;
					}
					new_mat++;
				}
			}
	
	/*****************************************************************
	 * **************************************************************/
			
			
		}
	}

	
	/***************************************************/
	
}


void AASS::vodigrex::ThinkerVoronoi::localMaximaCombo()
{

	this->_map_result =cv::Mat(_voronoi.size().height, _voronoi.size().width, CV_32F, cv::Scalar(0,0,0));
		// accept only char type matrices
    CV_Assert(_voronoi.depth() != sizeof(uchar));
	float val_min, val_max, val_max_h, val_min_h;
	
	float max = 1;
	float min =0;
	
	cv::Mat local_max = cv::Mat(_voronoi.size().height, _voronoi.size().width, CV_32F, cv::Scalar(0,0,0));
	cv::Mat local_max_best = cv::Mat(_voronoi.size().height, _voronoi.size().width, CV_32F, cv::Scalar(0,0,0));
	
	cv::MatIterator_<float> it = _voronoi.begin<float>();
	
	cv::MatIterator_<float> new_mat = local_max.begin<float>();
	cv::MatIterator_<float> new_mat_best = local_max_best.begin<float>();
	
	//Horizontal maximnew_mata
	for( int i = 0; i< (local_max.size().width) ;i++){
		for(int j= 1;j< local_max.size().height-1;j++){
			it++;
			val_min = (float)(*(it-1));
			val_max = (float)(*(it+1));			
			val_min_h = (float)(*(it-local_max_best.size().width));
			val_max_h = (float)(*(it+local_max_best.size().width));
			//Best first part
			if( ( (val_min < (*it) && val_max<(*it) ) || ( val_min_h < (*it) && val_max_h <(*it) ) ) && (*it)>_threshold_local_max){
				//It's a local maxima
				(*new_mat_best) = max;
			}
			else{
				(*new_mat_best) = min;
			}
			new_mat_best++;
			
// 			Maxima First part
			if(val_min <= (*it) && val_max<=(*it) && (*it)>_threshold_local_max){
				//It's a local maxima
				(*new_mat) = max;
			}
			else{
				(*new_mat) = min;
			}
			new_mat++;
			}
		//std::cout <<std::endl;
	}
	
	//Vertical maxima and fuse !
	it = _voronoi.begin<float>();
	new_mat= local_max.begin<float>();
	new_mat_best = local_max_best.begin<float>();
	cv::MatIterator_<float> mat_fused = this->_map_result.begin<float>();
	
	for( int i = 1; i< (local_max.size().width-1) ;i++){
		for(int j= 1;j< local_max.size().height;j++){
			it++;
			val_min = (float)(*(it-local_max.size().width));
			val_max = (float)(*(it+local_max.size().width));
			if(val_min <= (*it) && val_max<=(*it) && (*new_mat) == max){
				//It's a local maxima
				(*mat_fused) = max;
			}
			else if((*new_mat_best) == max){
				(*mat_fused) = max;
			}
			else{
				(*mat_fused) = min;
			}
			mat_fused++;
			new_mat++;
			new_mat_best++;
		}
		//std::cout <<std::endl;
	}
	//std::cout << std::endl << std::endl;
	//Need to fuse themm now
}



/* SKELETON :
 * 
 * from
 * http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
 * 
 * http://cgm.cs.mcgill.ca/~godfried/teaching/projects97/azar/skeleton.html
 * 
 * Problem is some pattern are erased
 **/

void AASS::vodigrex::ThinkerVoronoi::skeleton()
{
	cv::cvtColor(this->_map_in, _voronoi, CV_RGB2GRAY);
	cv::threshold(_voronoi, _voronoi, 10, 255, CV_THRESH_BINARY_INV);
	
	cv::Mat eroded;
	cv::Mat temp;
	_map_result = _voronoi.clone();
	_map_result.setTo(cv::Scalar(0));

	std::cout << " Before the loop " <<std::endl;
	
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	bool done;
	do
	{
		cv::erode(_voronoi, eroded, element);
		cv::dilate(eroded, temp, element);
		cv::subtract(_voronoi, temp, temp);
		cv::bitwise_or(_map_result, temp, _map_result);
		eroded.copyTo(_voronoi);

		done = (cv::countNonZero(_voronoi) == 0);
	} while (!done);

}




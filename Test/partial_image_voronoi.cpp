#include "ThinkerVoronoi.hpp"
#include "DynamicGraphExtractor.hpp"
// 
int main(){
	
	cv::Mat map = cv::imread("../Test/map.png", CV_LOAD_IMAGE_GRAYSCALE);
	AASS::vodigrex::ThinkerVoronoi thinker_map;
	thinker_map.setLevel(1);
	thinker_map.think(map);
	cv::Mat result_t_map;
	thinker_map.getResult().copyTo(result_t_map);
	cv::imshow("Voronoi", result_t_map);
	cv::waitKey(0);
	
	AASS::vodigrex::ThinkerVoronoi thinker_map2;
	thinker_map2.setLevel(30);
	thinker_map2.think(map);
	cv::Mat result_t_map2;
	thinker_map2.getResult().copyTo(result_t_map2);
	cv::imshow("Voronoi2", result_t_map2);
	cv::waitKey(0);
	
	
	cv::Mat line = cv::imread("../Test/ObstacleMap111.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat line_base;
	line.copyTo(line_base);
	cv::imshow("base", line);
	AASS::vodigrex::ThinkerVoronoi thinker;
	thinker.setLevel(30);
	thinker.think(line);
	cv::Mat result_t;
	thinker.getResult().copyTo(result_t);
	cv::imshow("Voronoi", result_t);
	cv::Mat roi = line(cv::Rect(0,0,line.size().width/1.5,line.size().height/1.5));
	cv::imshow("ROI", roi);
	thinker.think(roi);
	cv::Mat result_roi = thinker.getResult();
	cv::imshow("Result roi", result_roi);
	cv::Mat roi_final = result_roi(cv::Rect(0,0,result_roi.size().width-1,result_roi.size().height-1));
	cv::imshow("Result roi final", roi_final);
	
// 	cv::waitKey(0);
	
	cv::Mat line_modified = cv::imread("../Test/ObstacleMap1_modifed.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::imshow("base_modified", line_modified);
// 	cv::waitKey(0);
	AASS::vodigrex::ThinkerVoronoi thinker_modified;
	thinker_modified.setLevel(30);
	thinker_modified.think(line_modified);
	cv::Mat result_modified_t; 
	thinker_modified.getResult().copyTo(result_modified_t);
	cv::imshow("Voronoi_modified", result_modified_t);
// 	cv::waitKey(0);
/*	
	cv::Mat test = cv::Mat::zeros(line.size().width, line.size().height);
	
	test = test + */

	cv::Mat diff;
// 	= result - result_modified;
// 	cv::imshow("Voronoi again", result);
	
	cv::Mat result =  cv::Scalar::all(255) - result_t;
// 	cv::waitKey(0);
	cv::Mat result_modified =  cv::Scalar::all(255) - result_modified_t;
// 	cv::waitKey(0);
// 	cv::imshow("bug1", result);
// 	cv::imshow("bug2", result_modified);
// 	cv::waitKey(0);
	diff = result - result_modified;
// 	cv::waitKey(0);
	cv::Mat diff2 = result_modified - result;
// 	cv::waitKey(0);
	cv::Mat all_diff = diff + diff2;
// 	cv::waitKey(0);
	
	cv::imshow("Diff", diff);
	cv::imshow("Diff2", diff2);
	cv::imshow("AllDiff", all_diff);
	
	cv::waitKey(0);
	
	
	cv::Mat line_base_inv =  cv::Scalar::all(255) - line_base;
	cv::Mat line_modified_inv =  cv::Scalar::all(255) - line_modified;
	
	
	cv::Mat diff_map = line_base_inv - line_modified_inv;
	cv::Mat diff2_map = line_modified_inv - line_base_inv;
	cv::Mat all_diff_map = diff_map + diff2_map;
	
	thinker_modified.think(all_diff_map);
	cv::Mat all_voro; thinker_modified.getResult().copyTo(all_voro);
	
	cv::imshow("Diff_map", line_base_inv);
	cv::imshow("Diff_map_inv", line_modified_inv);
	cv::imshow("Diff2_map", diff2_map);
	cv::imshow("AllDiff_map", all_diff_map);
	cv::imshow("AllVORO_map", all_voro);
	
	cv::waitKey(0);
	
	AASS::vodigrex::DynamicGraphExtractor<> gp;
	gp.inputMap(line);
	gp.getNonModifiedEdges();
	
}
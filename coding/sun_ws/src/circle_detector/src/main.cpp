// #include <detector.h>
#include <PPS.h>
#include <MMCS.h>
#include <time.h>

int main(int argc, char** argv){

    cv::Mat src = cv::imread("../1.jpg");
    
    // ================================= PPS & MMCS ========================================
    time_t PPS_begin = clock();
    PPS model(src);
    std::vector<Eigen::Vector2f> feature_points = model.findFeaturePoints();
    time_t PPS_end = clock();


    time_t MMCS_begin = clock();
    MMCS searcher(feature_points, Vector2f(model.src.cols/2,model.src.rows/2));
    searcher.discard_noise(7,11);
    searcher.gradient_descend(0.01, 30);
    time_t MMCS_end= clock();


    if(model.src.at<uchar>(searcher.center[1], searcher.center[0]) > 125){
        cv::circle(model.src, cv::Point(searcher.center[0], searcher.center[1]), 3, cv::Scalar(0), 3);
    }else{
        cv::circle(model.src, cv::Point(searcher.center[0], searcher.center[1]), 3, cv::Scalar(255), 3);
    }
    cv::imshow("MMCS_center_img", model.src);

    cout << "PPS costs " << (-PPS_begin+PPS_end)*1000.0/CLOCKS_PER_SEC << " ms" << endl;
    cout << "MMCS costs " << (-MMCS_begin+MMCS_end)*1000.0/CLOCKS_PER_SEC << " ms" << endl;


    // =================================== HOUGH ========================================
    src = cv::imread("../1.jpg");
    time_t H_begin = clock();
    float scaler = (float)500.0/src.cols;
    cv::resize(src, src, cv::Size(), scaler, scaler);
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(src, src, cv::Size(5,5), 5);
    //cv::threshold(src, src, 150, 255, cv::THRESH_BINARY);
	vector<cv::Vec3f> cir;
	cv::HoughCircles(src, cir, CV_HOUGH_GRADIENT, 1, 100, 180, 30, 0, 0);
	time_t H_end = clock();

    cout << "Hough costs " << (H_end - H_begin)*1000.0/CLOCKS_PER_SEC << " ms" << endl;
    if(cir.size() > 0){
        cout << "Hough circle: " << cir[0][0] << "\t" << cir[0][1] << endl;
    }

    cv::RNG rng(123456);
	for (size_t i = 0; i < cir.size(); i++) {
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::circle(src, cv::Point(cir[i][0], cir[i][1]), cir[i][2], color, 1, 8);
	}
	cv::imshow("Hough_img", src);


    // // ======================================== PPS target searching ======================================
    // PPS model(src);
    // model.findFeaturePoints();


    
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
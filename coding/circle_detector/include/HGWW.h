#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <cstdlib>

#include <eigen3/Eigen/Dense>
#define infinity 10000
#define sample_number 20
using namespace std;


class HGWW{
public:
    // x0,  y0, a, b, gamma
    float ellipse_param[5];
    // point set
    vector<Eigen::Vector2f> PointSet;
    // sample point set
    vector<Eigen::Vector2f> SamplePointSet;
    // ellipse model g(x|theta) params
    vector<Eigen::Vector2f> mean;
    vector<Eigen::Matrix2f> Sigma;
    vector<float> weights;

    HGWW(/* args */);
    ~HGWW();
    bool is_inside_ellipse(cv::Point point);
    Eigen::Matrix2f get_rotation_matrix();
    float get_distance_from_point2ellipse(cv::Point point);
    float d_theta(cv::Point point, float T, float d);
    int indicator_func(float T, float d);
    void resample_points_from_ellipse();
    float GaussianKernel(Eigen::Vector2f x, Eigen::Vector2f u, Eigen::Matrix2f Sigma);
    float f_x_DataModel(Eigen::Vector2f x, float h);
    void update_ellipse_model_params(float h);
    float compute_cost_function(float h);

    void Distance_Density_Based_Fitting(float T ,float h_max=4, float h_min=3);
};

HGWW::HGWW(/* args */)
{
}

HGWW::~HGWW()
{
}


// judge if the given point is inside of the param-based ellispe
bool HGWW::is_inside_ellipse(cv::Point point){
    Eigen::Vector2f point_vec(point.x - this->ellipse_param[0], 
                              point.y - this->ellipse_param[1]);
    point_vec = this->get_rotation_matrix().inverse()*point_vec;
    point_vec[0] = point_vec[0]/this->ellipse_param[2];
    point_vec[1] = point_vec[1]/this->ellipse_param[3];
    if(point_vec.norm() >= 1){
        return false;
    }
    else{
        return true;
    }
}

// since the rotation matrix depends on the fifth param in ellipse_param
// once you update the ellipse data, you need use a updated rotation matrix
Eigen::Matrix2f HGWW::get_rotation_matrix(){
    Eigen::Matrix2f R;
    R << cos(this->ellipse_param[4]), -sin(this->ellipse_param[4]),
         sin(this->ellipse_param[4]), cos(this->ellipse_param[4]);
    return R;
}

// compute an approximate value of the distance from a given point to ellipse
float HGWW::get_distance_from_point2ellipse(cv::Point point){
    float a = this->ellipse_param[2], b = this->ellipse_param[3];
    if(this->is_inside_ellipse(point)){
        return min(a,b)*(1-sqrt(pow(point.x/a, 2)+pow(point.y/b, 2)));
    }else{
        return sqrt(pow(point.x,2)+pow(point.x,2))*(1-1/(sqrt(pow(point.x/a, 2)+pow(point.y/b, 2))));
    }
}

// compute d_theta, where T is the Inlier threshold
float HGWW::d_theta(cv::Point point, float T, float d){
    if(d*d < T*T){
        return d*d;
    }else{
        return infinity;
    }
}

// indicator function to judge if d^2 < T^2
int HGWW::indicator_func(float T, float d){
    if(d*d < T*T){
        return 1;
    }else{
        return 0;
    }
}

// resample points from ellipse
void HGWW::resample_points_from_ellipse(){
    srand((unsigned)time(NULL));
    for(int i = 0; i < sample_number; i++){
        float t = (rand()/(float)RAND_MAX)*2*CV_PI;
        this->SamplePointSet.push_back(
            this->get_rotation_matrix()*Eigen::Vector2f(
                this->ellipse_param[2]*cos(t),
                this->ellipse_param[3]*sin(t)
            )+Eigen::Vector2f(
                this->ellipse_param[0],
                this->ellipse_param[1]
            )
        );
    }
    return;
}

// compute GaussianKernel
float HGWW::GaussianKernel(Eigen::Vector2f x, Eigen::Vector2f u, Eigen::Matrix2f Sigma){
    return (1/(2*CV_PI*sqrt(Sigma.norm())))*exp(
        -0.5*(x-u).transpose()*Sigma.inverse()*(x-u)
    );
}
    
// compute data model
float HGWW::f_x_DataModel(Eigen::Vector2f x, float h){
    Eigen::Matrix2f Sigma = Eigen::MatrixXd::Identity(2,2)*h*h;
    int n = this->PointSet.size();
    float f_x = 0;
    for(int i = 0; i < n; i++){
        f_x += this->GaussianKernel(x, this->PointSet[i], Sigma);
    }
    return f_x/n;
}

// update_ellipse_model_params
void HGWW::update_ellipse_model_params(float h){
    float sum_h = 0;
    for(int i = 0; i < sample_number; i++){
        Eigen::Vector2f u1 = this->SamplePointSet[i];
        Eigen::Vector2f u2;
        float h = (u1-u2).norm();
        if(i+1 >= sample_number){
            u2 = this->SamplePointSet[0];    
        }else{
            u2 = this->SamplePointSet[i+1];
        }
        this->mean.push_back(0.5*(u1+u2));
        Eigen::Vector2f n2 = (u1-u2)/h;
        Eigen::Matrix2f Q, A;
        Q << -n2[1], n2[0],
              n2[0], n2[1];
        A << h*h, 0,
             0, pow(h, 2);
        this->Sigma.push_back(Q.transpose()*A*Q);
        this->weights.push_back(h);
        sum_h += h;
    }
    for(int i = 0; i < sample_number; i++){
        this->weights[i] /= sum_h;
    }
    return;
}

// Algorithm 1
void HGWW::Distance_Density_Based_Fitting(float T, float h_max, float h_min){
    float h = h_max;
    float E = infinity;
    while(h >= h_min){

    }

    return;
}

// compute cost function
float HGWW::compute_cost_function(float h){
    float A = 0,B = 0,C = 0;
    Eigen::Vector2f zero_vec(0,0);
    int n = this->PointSet.size();
    if(n >= sample_number){
        for(int i = 0; i < n; i++){
            for(int j = 0; j < n; j++){
                A += (1/(n*n))*this->GaussianKernel(zero_vec, this->PointSet[i]-this->PointSet[j], 2*h*h*Eigen::MatrixXd::Identity(2,2));
                if(j < sample_number){
                    B += (1/n)*this->weights[j]*this->GaussianKernel(
                        zero_vec,
                        this->PointSet[i]-this->mean[j],
                        h*h*Eigen::MatrixXd::Identity(2,2)+this->Sigma[j]
                    );
                }
                if(j < sample_number && i < sample_number){
                    C += this->weights[j]*this->weights[i]*this->GaussianKernel(
                        zero_vec,
                        this->mean[j]-this->mean[i],
                        this->Sigma[j]+this->Sigma[i]
                    );
                }
            }
        }
    }else{
        for(int i = 0; i < sample_number; i++){
            for(int j = 0; j < sample_number; j++){
                if(i < n && j < n){
                    A += (1/(n*n))*this->GaussianKernel(zero_vec, this->PointSet[i]-this->PointSet[j], 2*h*h*Eigen::MatrixXd::Identity(2,2));
                }
                if(i < n){
                    B += (1/n)*this->weights[j]*this->GaussianKernel(
                        zero_vec,
                        this->PointSet[i]-this->mean[j],
                        h*h*Eigen::MatrixXd::Identity(2,2)+this->Sigma[j]
                    );
                }
                C += this->weights[j]*this->weights[i]*this->GaussianKernel(
                    zero_vec,
                    this->mean[j]-this->mean[i],
                    this->Sigma[j]+this->Sigma[i]
                );
            }
        }
    }
    return A+B+C;
}
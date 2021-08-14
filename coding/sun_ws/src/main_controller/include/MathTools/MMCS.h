#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>


using namespace std;
using namespace Eigen;

// 
/* 
@brief Mapping-based MSE optimization Center Search Method
@param vector<Vector2f> feature_points, obtained from PPS
@param Vector2f prior_center
@param float scale = 1000.0
 */
class MMCS{
public:
    bool is_debugging = true;

    Vector2f center;
    std::vector<Vector2f> feature_points;

    VectorXf v,x,y;
    int N;
    float Ev;
    float Ex0x, Ey0y;
    float scale;

    void update_params();
    void discard_noise(float v_min, float v_max);
    void SaveVdata(string file = "../V_sequence.txt");
    float cost_func();
    Vector2f gradient();

    void gradient_descend(float learning_rate = 0.01, int max_iter = 1000);
    

    MMCS(vector<Vector2f> feature_points, Vector2f prior_center,float scale = 1000.0);
    ~MMCS();
};

MMCS::MMCS(vector<Vector2f> feature_points, Vector2f prior_center, float scale){
    this->center = prior_center;
    this->scale = scale;
    this->feature_points = feature_points;
}

MMCS::~MMCS(){
}

void MMCS::SaveVdata(string file){
    ofstream outf(file.c_str(), ios_base::out);
    for(int i = 0; i < this->N; i++){
        outf << this->v[i] << "\t";
    }
    outf << endl;
    return;
}
// update v, Ev, Ex0x, Ey0y, when obtaining a new center
void MMCS::update_params(){
    this->v = ((this->x.array()-this->center[0])*(this->x.array()-this->center[0])
              +(this->y.array()-this->center[1])*(this->y.array()-this->center[1]))/this->scale;
    this->Ev = this->v.mean();
    this->Ex0x = (this->center[0] - this->x.array()).mean();
    this->Ey0y = (this->center[1] - this->y.array()).mean();
}


// discard noise underlying the given range of v
// you'd better choose the range by MATLAB plot
void MMCS::discard_noise(float v_min, float v_max){
    std::vector<bool> discard_flag;
    int max_index = 0;
    for(int i = 0; i < this->feature_points.size(); i++){
        float tmp_x = this->feature_points[i][0] - this->center[0];
        float tmp_y = this->feature_points[i][1] - this->center[1];
        float tmp_v = (tmp_x*tmp_x+tmp_y*tmp_y)/this->scale;
        if((tmp_v > v_min && tmp_v < v_max) || (v_min == v_max)){
            discard_flag.push_back(true);
            max_index ++;
        }else{
            discard_flag.push_back(false);
        }
    }

    this->v = VectorXf::Zero(max_index);
    this->x = VectorXf::Zero(max_index);
    this->y = VectorXf::Zero(max_index);
    for(int i = 0, index = 0; i < this->feature_points.size(); i++){
        if(discard_flag[i]){
            this->v[index] = ((this->feature_points[i][0] - this->center[0])*(this->feature_points[i][0] - this->center[0])
                             +(this->feature_points[i][1] - this->center[1])*(this->feature_points[i][1] - this->center[1]))/this->scale;
            this->x[index] = this->feature_points[i][0];
            this->y[index] = this->feature_points[i][1];
            index++;
        }
    }
    this->N = this->v.size();
    this->update_params();
    if(is_debugging){this->SaveVdata();}
    return;
}


float MMCS::cost_func(){
    return (this->v.array() - this->Ev).square().sum()/this->N;
}

// grad_x = sum((v-Ev).*((x0-x)-Ex0x));
// grad_y = sum((v-Ev).*((y0-y)-Ey0y));
Vector2f MMCS::gradient(){
    float grad_x = ((this->v.array()-this->Ev)*(this->center[0]-this->x.array()-this->Ex0x)).sum();
    float grad_y = ((this->v.array()-this->Ev)*(this->center[1]-this->y.array()-this->Ey0y)).sum();
    return Vector2f(grad_x*4/this->N, grad_y*4/this->N);
}

void MMCS::gradient_descend(float learning_rate, int max_iter){
    for(int iter = 0; iter < max_iter; iter++){
        if(is_debugging){
            float MSE = this->cost_func();
            cout << "iteration:" << iter+1 << "\t MSE:" << MSE << endl;
        }
        this->center -= this->gradient()*learning_rate;
        this->update_params();
    }
    cout << "Optimized center:" << this->center[0] <<"\t"<< this->center[1] << endl;
    cout << "N:" << this->N << endl;
}
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <ctime>

using namespace std;
using namespace Eigen;

#define infinity MAXFLOAT


/* adaptive PID parameters learning machine, powered by PSO 
@param Vector3f param_min
@param Vector3f param_max
*/
class PSO{
public:
    vector<Vector3f> local_best_params;
    Vector3f global_best_params;
    vector<float> local_best_t;
    float global_best_t = infinity;

    vector<Vector3f> velocity;

    int particle_num;
    // PSO parameters: w c1 c2
    // where w is the weight of staying still, c1 is about local best interfere, c2 is about global best interfere
    float PSO_params[3] = {0.3, 0.252, 0.442};

    vector<Vector3f> run(vector<float> run_t, vector<Vector3f> run_params, float scale = 1.0);
    
    PSO(Vector3f param_min, Vector3f param_max, int particle_num = 10);
    ~PSO();
};

// initialize the particles with a given spawning area
PSO::PSO(Vector3f param_min, Vector3f param_max, int particle_num){
    this->particle_num = particle_num;
    Vector3f mean = (param_max+param_min)/2.0;
    Vector3f range = (param_max-param_min)/2.0;
    srand((unsigned)time(NULL));
    for(int i = 0; i < particle_num; i++){
        Vector3f particle = (Vector3f::Random(3).array()*range.array())+mean.array();
        this->local_best_params.push_back(particle);
        this->velocity.push_back(Vector3f::Zero(3));
        this->local_best_t.push_back(1000);
    }
    return;
}

PSO::~PSO(){
}

/* update new parameters according to given run time, you can set a scale used to modify the updated parameters' range 
@param vector<float> run_t
@param vector<Vector3f> run_params
*/
vector<Vector3f> PSO::run(vector<float> run_t, vector<Vector3f> run_params, float scale){
    vector<Vector3f> new_params;
    // update best run time
    // compute new PID params
    for(int i = 0; i < this->particle_num; i++){
        if(run_t[i] < this->local_best_t[i]){
            this->local_best_t[i] = run_t[i];
            this->local_best_params[i] = run_params[i];
            if(run_t[i] < this->global_best_t){
                this->global_best_t = run_t[i];
                this->global_best_params = run_params[i];
            }
        }
        this->velocity[i] = this->PSO_params[0]*this->velocity[i]+
                            this->PSO_params[1]*(this->local_best_params[i]-run_params[i])+
                            this->PSO_params[2]*(this->global_best_params-run_params[i]);
        new_params.push_back(scale*this->velocity[i]+run_params[i]);
    }
    return new_params;
};


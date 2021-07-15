
#define USE_NLOPT
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iterator>
#include <mutex>
#include <thread>
#include <string>
#include <iostream>
#include <stdio.h>
#include <setjmp.h>
#include <limbo/bayes_opt/boptimizer.hpp>
#include <limbo/limbo.hpp>

#include<fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <nlopt.h>

#include <vector>
#include <utility>
//#include <gnuplot_i.hpp>
#include <unistd.h>





// you can also include <limbo/limbo.hpp> but it will slow down the compilation
#include <limbo/bayes_opt/boptimizer.hpp>

double alpha_value;
int sample_value;
int iteration_value;


void my_ctor (void) __attribute__ ((constructor));
void
my_ctor (void)
{
    printf ("hello before main()\n");
    std::ifstream file;
    std::string riga, valore_param;
    int inizio,fine,i;
    int line;
    std::string namefile = "/home/loris/ply_and_stl/param.txt";


    line =51;
    file.open(namefile, std::ios::in);
      if(!file){
    std::cout <<"\n\n The file cannot be opened";
  } 
     while(!file.eof() && i<line){
    std::getline(file,riga);
    i++;
    }
    std::getline(file,riga);
    inizio = riga.find("{");
    fine = riga.find("}");
    valore_param = riga.substr(inizio+1,fine-inizio-1); 
    file.close();  
    //valoreparam = readline(name_file,44,"{","}");
    sample_value = std::stoi(valore_param); 
    printf ("samples: %d\n",std::stoi(valore_param));


    line =52;
    i=0;
    file.open(namefile, std::ios::in);
      if(!file){
    std::cout <<"\n\n The file cannot be opened";
  } 
     while(!file.eof() && i<line){
    std::getline(file,riga);
    i++;
    }
    std::getline(file,riga);
    inizio = riga.find("{");
    fine = riga.find("}");
    valore_param = riga.substr(inizio+1,fine-inizio-1); 
    file.close();  
    //valoreparam = readline(name_file,44,"{","}");
    iteration_value = std::stoi(valore_param); 

    printf ("iterations: %d\n",iteration_value);    

    line =53;
    i=0;
    file.open(namefile, std::ios::in);
      if(!file){
    std::cout <<"\n\n The file cannot be opened";
  } 
     while(!file.eof() && i<line){
    std::getline(file,riga);
    i++;
    }
    std::getline(file,riga);
    inizio = riga.find("{");
    fine = riga.find("}");
    valore_param = riga.substr(inizio+1,fine-inizio-1); 
    file.close();  
    //valoreparam = readline(name_file,44,"{","}");
    alpha_value = std::stod(valore_param); 

    printf ("alpha: %4.4f\n",alpha_value); 
}

using namespace limbo;

struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

// depending on which internal optimizer we use, we need to import different parameters
    struct opt_nloptnograd : public defaults::opt_nloptnograd {
    };


    // enable / disable the writing of the result files
    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(int, stats_enabled, true);
    };

    // no noise
    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 1e-10);
    };

    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
    };

    // we use 10 random samples to initialize the algorithm
    struct init_randomsampling {
        BO_DYN_PARAM(int, samples);
    };

    // we stop after 40 iterations
    struct stop_maxiterations {
        BO_DYN_PARAM(int, iterations);
    };

    // we use the default parameters for acqui_ucb
    struct acqui_ucb : public defaults::acqui_ucb {

        BO_DYN_PARAM(double,alpha);

    };

};

struct Eval {
    // number of input dimension (x.size())
    BO_PARAM(size_t, dim_in, 1);
    // number of dimensions of the result (res.size())
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        double y = -((5 * x(0) - 2.5) * (5 * x(0) - 2.5)) + 5;
        // we return a 1-dimensional vector
        return tools::make_vector(y);
    }
};



BO_DECLARE_DYN_PARAM(double, Params::acqui_ucb,alpha = .2);
BO_DECLARE_DYN_PARAM(int, Params::init_randomsampling, samples = sample_value);
BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations = iteration_value);


int main()
{   
    // we use the default acquisition function / model / stat / etc.

    // using Kernel_t = kernel::MaternFiveHalves<Params>;
    // using Mean_t = mean::Data<Params>;  
    // using gp_t = model::GP<  Params,Kernel_t,Mean_t,
    bayes_opt::BOptimizer<Params> boptimizer;
    // run the evaluation
    boptimizer.optimize(Eval());
    // the best sample found
    std::cout << "Best sample: " << boptimizer.best_sample()(0) << " - Best observation: " << boptimizer.best_observation()(0) << std::endl;
    return 0;
}

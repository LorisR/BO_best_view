#define USE_NLOPT
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <string>
#include <iostream>
#include <stdio.h>
#include <setjmp.h>
#include <limbo/bayes_opt/boptimizer.hpp>
#include <limbo/limbo.hpp>
#include <try_bayesian/bayesian_msg.h>
#include <try_bayesian/to_bayesian.h>
#include<fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <nlopt.h>

#include <vector>
#include <utility>
//#include <gnuplot_i.hpp>
#include <unistd.h>
#include <ros/ros.h>


boost::shared_ptr<try_bayesian::to_bayesian const> sharedPtr;
try_bayesian::to_bayesian msg_from_halcon;
try_bayesian::bayesian_msg msg_to_normalizer;
float pi = M_PI;
float rho;
float phi;
float ragg;
float score;
int ii = 0;

int Kragg; 
int Kscore;
int K_penalita;
int K_premio; 
double y_penalita =0;
double y_premio =0;
double y_ragg = 0;
double y_score = 0;
int sample_value, iteration_value;
double alpha_value;


// set del numero di prove e numero di ottimizazioni

float soglia_min_penalita;
float soglia_min_premio;
//
//
std::string fine_bo;
std::ofstream save_file;
std::string tempo,name_save_file,tipo_funzione;
std::string file_format = ".txt";

//leggo i parametri bo
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
    printf("\n ho letto il file\n");
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
std::string readline(std::string namefile,int line,std::string delimiter1,std::string delimiter2){
  std::ifstream file;
  std::string riga, valore_param;
  int i=0;
  int inizio, fine;
  if(!file){
    std::cout <<"\n\n The file cannot be opened";
  }  
  file.open(namefile, std::ios::in);
  while(!file.eof() && i<line){
    std::getline(file,riga);
    i++;
  }
  std::getline(file,riga);
  inizio = riga.find(delimiter1);
  fine = riga.find(delimiter2);
  valore_param = riga.substr(inizio+1,fine-inizio-1);      

  return valore_param;  
}

std::string GetCurrentTimeForFileName(){
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}

using namespace limbo;


struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

// depending on which internal optimizer we use, we need to import different parameters
#ifdef USE_NLOPT
    struct opt_nloptnograd : public defaults::opt_nloptnograd {
    };
#elif defined(USE_LIBCMAES)
    struct opt_cmaes : public defaults::opt_cmaes {
    };
#else
    struct opt_gridsearch : public defaults::opt_gridsearch {
    };
#endif

    // enable / disable the writing of the result files
    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(int, stats_enabled, false);
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

struct Eval{


    int i;
    // m = ros::NodeHandle("/");
    // number of input dimension (x.size())
    BO_PARAM(size_t, dim_in, 2);
    // number of dimensions of the result (res.size())
    BO_PARAM(size_t, dim_out, 1);

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {   

        ros::NodeHandle n;
        ros::NodeHandle m;
        msg_to_normalizer.phi = x(0);
        msg_to_normalizer.theta = x(1);
        msg_to_normalizer.rho= 200;
        
        // pubblico la richiesta della bayesian optimization
        ros::Publisher pub = m.advertise<try_bayesian::bayesian_msg>("bayesian", 1000);
        ros::Duration(3.0).sleep();
        std:: cout << "\n ho pubblicato adesso \n";
        pub.publish(msg_to_normalizer);


        // aspetto la risposta dal nodo halcon
        sharedPtr = ros::topic::waitForMessage<try_bayesian::to_bayesian>("/halcon_to_bayesian",n);
            if (sharedPtr == NULL){
        ROS_INFO("No point clound messages received");
        
            }else{
                //std::cout << "received\n" << *sharedPtr;
                msg_from_halcon=*(sharedPtr);
            }

        


        ragg = msg_from_halcon.check;
        score = msg_from_halcon.score;
        
        if(ragg == 1.0){
            ragg = 0;
        }

        std::cout << "\n\n-------------------------------\n\n"<<"iteration no:"<<ii<<"\n";
        std::cout << "la raggiungibilita e lo score valgono:"<< ragg <<"\t" << score <<"\n";
        if (tipo_funzione == "False"){
            y_ragg = Kragg*ragg;
            y_score = Kscore*(score-1);
            std::cout << "Kragg:\t"<< Kragg << "\nKscore:\t" << Kscore<<"\n";
            // y1 y2 = ros wait for message
        }else{
            std::cout << "Kragg:\t"<< Kragg << "\nKscore:\t" << Kscore<<"\n";
            std::cout << "Kpenalita:\t"<< K_penalita << "\nKpremio:\t" << K_premio<<"\n";
            // y1 y2 = ros wait for message
            y_ragg = Kragg*ragg;
            y_score = Kscore*(score-1);
            y_penalita = -std::pow(K_penalita*(score-std::max(score,soglia_min_penalita)),2);
            y_premio = K_premio*(score-std::max(score,soglia_min_premio));

            std::cout<<"value reachability function;\t"<<  y_ragg <<"\nvalue score function:\t"<< y_score << "\n";
            std::cout<<"value penalty score function;\t"<<  y_penalita <<"\nvalue reward score function:\t"<<y_premio << "\n";
           
            
        }
        double y = y_ragg + y_score + y_penalita + y_premio;
        std::cout << "Value overall cost function:\t" << y << "\n";
        save_file << x(0) <<"\t"<< x(1) <<"\t";
        save_file << ragg <<"\t" << score <<"\t";
        save_file << y_ragg <<"\t" << y_score <<"\t";
        save_file << y_penalita <<"\t" << y_premio <<"\n";
        // we return a 1-dimensional vector
        ii =ii+1;
        return tools::make_vector(y);
        
    }
};

BO_DECLARE_DYN_PARAM(double, Params::acqui_ucb,alpha = alpha_value);
BO_DECLARE_DYN_PARAM(int, Params::init_randomsampling, samples = sample_value);
BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations = iteration_value);

int main(int argc, char **argv)
{  ros::init(argc ,argv, "Bayesian_node");
   std::string valore_param;
    valore_param = readline("/home/loris/ply_and_stl/param.txt",38,"{","}");
    std::cout << "Il nome del file vale: "<<  valore_param;
    tempo = GetCurrentTimeForFileName();
    name_save_file = valore_param+tempo+file_format;
    std::cout << "\n" <<name_save_file;
    save_file.open(name_save_file,std::ios::app);
    valore_param = readline("/home/loris/ply_and_stl/param.txt",39,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param;
    Kragg = std::stof(valore_param); 
    valore_param = readline("/home/loris/ply_and_stl/param.txt",40,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param;
    Kscore = std::stof(valore_param); 
    valore_param = readline("/home/loris/ply_and_stl/param.txt",41,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param;
    tipo_funzione = valore_param; 
    valore_param = readline("/home/loris/ply_and_stl/param.txt",42,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param;
    soglia_min_penalita = std::stof(valore_param); 
    valore_param = readline("/home/loris/ply_and_stl/param.txt",43,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param;
    soglia_min_premio = std::stof(valore_param); 
    valore_param = readline("/home/loris/ply_and_stl/param.txt",44,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param;
    K_penalita = std::stof(valore_param);
    valore_param = readline("/home/loris/ply_and_stl/param.txt",45,"{","}");
    std::cout << "\nho letto:\t"<<  valore_param <<"\n";
    K_premio = std::stof(valore_param);
    save_file << std::setprecision(3) << std::fixed; 
    save_file << "Numero prove:\t"<<sample_value<<"Numero ottimizzazioni\t"<<iteration_value<<"alpha = "<< Params::acqui_ucb::alpha()<<"\n";
    save_file << "x(0)\tx(1)\treachability\tscore\ty_ragg\ty_score\ty_penalita\ty_premio\n";

    // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    // we use the default acquisition function / model / stat / etc.

    bayes_opt::BOptimizer<Params> boptimizer;
    // run the evaluation
    // Eval evalfunct;

    // evalfunct.i = 0;
    

    boptimizer.optimize(Eval());
    std::cout << "Best sample: " << boptimizer.best_sample()(0) << " - Best observation: " << boptimizer.best_observation()(0) << std::endl;
    save_file.close(); 
    std:: cout << "\n digit a letter to close the terminal";
    std:: cin >> tempo;

    return 0;
}   

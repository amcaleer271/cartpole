#include <cartpole/state.hpp>
#include <cartpole/logging.hpp>
#include <cartpole/logging.hpp>
#include <fstream>

namespace cartpole{

    Logger::Logger(){
        output_file.open ("log.csv");
        output_file << "Position X,Angle Theta,Control u,Time t\n";
    }
    void Logger::store(const State& state, const double u, const double t){
        output_file << state.x <<","<< (state.theta * 180 / 3.14159) << "," << u << "," << t << "\n";
    }
    
    //Destructor
    Logger::~Logger(){
        output_file.close();
    }
};



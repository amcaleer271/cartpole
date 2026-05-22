#include <cartpole/state.hpp>
#include <cartpole/logging.hpp>
#include <fstream>

namespace cartpole
{

    // Contstructor
    Logger::Logger()
    {
        output_file.open("log.csv");                                // Filename of output file, located in build directory
        output_file << "Position X,Angle Theta,Control u,Time t\n"; // First row of data, must match headers in plotting
    }
    void Logger::store(const State &state, const double u, const double t)
    {
        // Log the current state, note that angle is logge in degrees
        output_file << state.x << "," << (state.theta * 180 / 3.14159) << "," << u << "," << t << "\n";
    }

    // Destructor
    Logger::~Logger()
    {
        output_file.close();
    }
};

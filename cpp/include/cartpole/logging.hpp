
//This header function defines the interface of the logger tool

#pragma once
#include <iostream>
#include <fstream>
#include <cartpole/state.hpp>

namespace cartpole
{

    class Logger
    {
    private:
        std::ofstream output_file;

    public:
        Logger();
        void store(const State &state, const double u, const double t);
        ~Logger();
    };

}

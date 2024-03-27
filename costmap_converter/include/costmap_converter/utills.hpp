#include <iostream>
#include <math.h>
#include <cstdlib>
namespace costmap_converter
{
    
    double inverseCDF(double u)
    {
        double x;
        
        if(0 <= u < 1/2)
        {
            x = sqrt(2*u); 
        }
        else if(1/2 <= u <= 1)
        {
            x = 2 - sqrt(2*(1-u));
        }
        else
        {
            x = 0;
        }
        //x = x;
        return x;
    }
    double sampling()
    {
        double u = ((double)rand()/(double)RAND_MAX);
        double sample = inverseCDF(u);
        return sample; 
    }
};
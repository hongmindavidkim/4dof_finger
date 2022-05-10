#include "actuator_transformation.h"
#include "math.h"
#include <cstdint>

uint32_t* ActuatorTransformation(float mcp, float pip, float dip, float mcr) { // M1=mcp, M2=pip, M3=dip, M4=mcr 
    uint32_t* transform = new uint32_t[4];
    // MCR dependencies
    transform[0] = rad2pulse(PULLEY_RATIO_1*mcr);
    transform[1] = rad2pulse(PULLEY_RATIO_2*mcr);
    transform[2] = rad2pulse(PULLEY_RATIO_3*mcr);
    // MCR joint input
    transform[3] = rad2pulse(mcr);
    // MCP joint input
    transform[0] += radconv(-1*mcp);
    // Distal dependencies
    transform[1] += radconv(PULLEY_DEPENDENCY_21*-1*mcp);
    transform[2] += radconv(PULLEY_DEPENDENCY_31*-1*mcp);
    // PIP joint input
    transform[1] += radconv(pip);
    // Distal dependencies
    transform[2] += radconv(PULLEY_DEPENDENCY_32*pip);
    // DIP joint input
    transform[2] += radconv(-1*dip);
    // Round to nearest integer
    transform[0] = (uint32_t)round(transform[0]);
    transform[1] = (uint32_t)round(transform[1]);
    transform[2] = (uint32_t)round(transform[2]);
    transform[3] = (uint32_t)round(transform[3]);
    
    //static uint32_t transform[4] = {(uint32_t)round(d_transform[0]), (uint32_t)round(d_transform[1]), (uint32_t)round(d_transform[2]), (uint32_t)round(d_transform[3])};
    return transform;
}
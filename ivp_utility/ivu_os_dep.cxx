// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <cstring>
#include <ivu_types.hxx>

// WIN32 ---------------------------------------------------------------------

static int IVP_RAND_SEED = 1;

// returns 0 .. 1.0
IVP_FLOAT ivp_rand()
{
    const uint next_seed = (uint)IVP_RAND_SEED * 75U;
    std::memcpy(&IVP_RAND_SEED, &next_seed, sizeof(IVP_RAND_SEED));
    IVP_FLOAT res = (IVP_RAND_SEED & 0xffff) / (float)0x10000;
    return res;
}

void ivp_srand(int seed)
{
    if (seed == 0)
        seed = 1;
    IVP_RAND_SEED = seed;
}

int ivp_srand_read()
{
    return IVP_RAND_SEED;
}

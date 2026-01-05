// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#pragma implementation "ivp_performancecounter.hxx"
#endif
#include <ivp_performancecounter.hxx>

const double kScaleFactorForReducingPrecisionLoss = 1e6;
const double kInvertedScaleFactorForReducingPrecisionLoss = 1.0 / kScaleFactorForReducingPrecisionLoss;

#if defined(WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

double GetInvertedPerformanceFrequency()
{
    LARGE_INTEGER frequency;
    // Since XP+ always succeeds.
    (void)QueryPerformanceFrequency(&frequency);
    return 1.0 / (frequency.QuadPart > 0 ? frequency.QuadPart : 1);
}

long long GetPerformanceCounter()
{
    LARGE_INTEGER profile_counter;
    // Since XP+ always succeeds.
    (void)QueryPerformanceCounter(&profile_counter);
    return profile_counter.QuadPart;
}

#endif

void IVP_PerformanceCounter_Simple::reset_and_print_performance_counters(IVP_Time current_time)
{
    const IVP_DOUBLE universe_samples_count = count_PSIs;
    if (universe_samples_count == 0.0)
        return;

    IVP_DOUBLE collision = counter[IVP_PE_PSI_UNIVERSE] + counter[IVP_PE_PSI_SHORT_MINDISTS] +
                           counter[IVP_PE_PSI_CRITICAL_MINDISTS] + counter[IVP_PE_PSI_HULL] + counter[IVP_PE_AT_INIT];
    IVP_DOUBLE dynamics = counter[IVP_PE_PSI_CONTROLLERS] + counter[IVP_PE_PSI_INTEGRATORS];
    IVP_DOUBLE sum = collision + dynamics;

    const IVP_DOUBLE factor = kInvertedScaleFactorForReducingPrecisionLoss / universe_samples_count;

    ivp_message("[performance] TOTAL %2.1f%% %2.2fs\nCOLLISION %2.2fs\nDYNAMIC %2.2fs\n\n"
                "UNIVERSE: %2.2fs\nCONTROLLERS: %2.2fs\nINTEGRATORS: %2.2fs\n"
                "HULL: %2.2fs\nSHORT MINDISTS: %2.2fs\nCRITICAL MINDISTS: %2.2fs\nSIMULATE %2.2fs\n",
                sum * factor * 100.0, sum * factor, collision * factor, dynamics * factor,
                counter[IVP_PE_PSI_UNIVERSE] * factor, counter[IVP_PE_PSI_CONTROLLERS] * factor,
                counter[IVP_PE_PSI_INTEGRATORS] * factor, counter[IVP_PE_PSI_HULL] * factor,
                counter[IVP_PE_PSI_SHORT_MINDISTS] * factor, counter[IVP_PE_PSI_CRITICAL_MINDISTS] * factor,
                counter[IVP_PE_AT_INIT] * factor);

#ifdef WIN32
    ref_counter64 = GetPerformanceCounter();
#endif

    counting = IVP_PE_PSI_START;
    memset(counter, 0, sizeof(counter));
    count_PSIs = 0;
    time_of_last_reset = current_time;
}

void IVP_PerformanceCounter_Simple::environment_is_going_to_be_deleted(IVP_Environment *)
{
    P_DELETE_THIS(this);
}

IVP_PerformanceCounter_Simple::~IVP_PerformanceCounter_Simple() {}

#if defined(WIN32)
IVP_PerformanceCounter_Simple::IVP_PerformanceCounter_Simple()
    : inv_counter_freq(GetInvertedPerformanceFrequency()), ref_counter64(GetPerformanceCounter()), count_PSIs(0),
      counting(IVP_PE_PSI_START), time_of_last_reset()
{
    memset(counter, 0, sizeof(counter));
}

void IVP_PerformanceCounter_Simple::pcount(IVP_PERFORMANCE_ELEMENT el)
{
    if (el == IVP_PE_PSI_UNIVERSE)
        count_PSIs++;

    const long long now = GetPerformanceCounter();
    const long long diff0 = now - ref_counter64;
    ref_counter64 = now;

    counter[counting] += kScaleFactorForReducingPrecisionLoss * double(diff0) * inv_counter_freq;
    counting = el;
}

void IVP_PerformanceCounter_Simple::start_pcount()
{
    counting = IVP_PE_PSI_START;
}

void IVP_PerformanceCounter_Simple::stop_pcount() {}

#elif defined(PSXII)

/*
 *	Include file for the performance counters. Link with libpc.a.
 *	Note that the performance counters will not be implemented
 *	in the retail version of the hardware so it sould not be
 *	compiled into the final release app.
 *
 *	Refer to the libpc.txt documentation for the defines to set
 *	up the performance counter. Below are some commonly used
 *	settings.
 */
#include <libpc.h>
/*
 * Count CPU cycles in Counter0 and DCache Misses
 * in Counter1
 */
#define PROFILE_CPU_DCACHE                             \
    (SCE_PC_CTE |                                      \
                                                       \
     SCE_PC0_CPU_CYCLE |                               \
     SCE_PC_U0 | SCE_PC_S0 | SCE_PC_K0 | SCE_PC_EXL0 | \
                                                       \
     SCE_PC1_DCACHE_MISS |                             \
     SCE_PC_U1 | SCE_PC_S1 | SCE_PC_K1 | SCE_PC_EXL1)

/*
 * Count ICache misses in Counter0 and CPU cycles
 * in Counter1.
 */
#define PROFILE_ICACHE_CPU                             \
    (SCE_PC_CTE |                                      \
                                                       \
     SCE_PC1_CPU_CYCLE |                               \
     SCE_PC_U1 | SCE_PC_S1 | SCE_PC_K1 | SCE_PC_EXL1 | \
                                                       \
     SCE_PC0_ICACHE_MISS |                             \
     SCE_PC_U0 | SCE_PC_S0 | SCE_PC_K0 | SCE_PC_EXL0)

/*
 * Count Address bus busy(0) and Data bus busy(1)
 */
#define PROFILE_ADDRBUS_DATABUS                        \
    (SCE_PC_CTE |                                      \
                                                       \
     SCE_PC0_ADDR_BUS_BUSY |                           \
     SCE_PC_U0 | SCE_PC_S0 | SCE_PC_K0 | SCE_PC_EXL0 | \
                                                       \
     SCE_PC1_DATA_BUS_BUSY |                           \
     SCE_PC_U1 | SCE_PC_S1 | SCE_PC_K1 | SCE_PC_EXL1)

/*
 *	Refer to the Sony libpc documentation for the flags.
 */
void IVP_PerformanceCounter_Simple::start_pcount()
{
    int flags = PROFILE_CPU_DCACHE;
    count_PSIs++;
    counting = IVP_PE_PSI_START;
    scePcStart(flags, 0, 0);
}

void IVP_PerformanceCounter_Simple::stop_pcount()
{
    scePcStart(SCE_PC0_NO_EVENT | SCE_PC1_NO_EVENT, 0, 0);
}

void IVP_PerformanceCounter_Simple::pcount(IVP_PERFORMANCE_ELEMENT el)
{
    /*
     *	This could be += or = depending on how pcount() is called
     */
    int c0 = scePcGetCounter0();
    int diff0 = c0 - ref_counter[0];
    ref_counter[0] = c0;

    int c1 = scePcGetCounter1();
    int diff1 = c1 - ref_counter[1];
    ref_counter[1] = c1;

    counter[counting] += diff0;
    counting = el;
}

#else
IVP_PerformanceCounter_Simple::IVP_PerformanceCounter_Simple()
    : ref_counter64(0), count_PSIs(0), counting(IVP_PE_PSI_START), time_of_last_reset()
{
    memset(counter, 0, sizeof(counter));
}

void IVP_PerformanceCounter_Simple::pcount(IVP_PERFORMANCE_ELEMENT) {}
void IVP_PerformanceCounter_Simple::start_pcount() {}
void IVP_PerformanceCounter_Simple::stop_pcount() {}
#endif

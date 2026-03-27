// Adapted from blah2-arm/src/capture/rspduo/RspDuo.h
// Changes: stripped Source/IqData inheritance, added g_tuner/g_capture_buf/g_waiting_rf_change,
//          single-tuner mode only, stream_b stub

#pragma once

#include "config.h"
#include "sdrplay_api.h"

#include <atomic>
#include <complex>
#include <vector>

// SDR globals (verbatim from RspDuo.cpp)
extern sdrplay_api_DeviceT        *chosenDevice;
extern sdrplay_api_DeviceT         devs[1023];
extern sdrplay_api_DeviceParamsT  *deviceParams;
extern sdrplay_api_ErrT            err;
extern sdrplay_api_CallbackFnsT    cbFns;
extern sdrplay_api_RxChannelParamsT *chParams;

// Tuner selection — sdrplay_api_Tuner_A (SMA1, reference) or _B (SMA2)
extern sdrplay_api_TunerSelectT    g_tuner;

// Capture state
extern std::vector<std::complex<float>> g_capture_buf;
extern std::atomic<bool>                g_capture_done;
extern std::atomic<bool>                g_waiting_rf_change;

// SDR lifecycle (verbatim from RspDuo.cpp)
void open_api();
void get_device();
void set_device_parameters(double fc_hz);
void initialise_device();
void uninitialise_device();

// Retune mid-stream (new — blah2 never retunes)
void retune(double fc_hz);

// Static C callback wrappers — pattern verbatim from RspDuo.h
// Callbacks are plain C function pointers; these forward to our free functions.
static void _stream_a_callback(short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params, unsigned int numSamples,
    unsigned int reset, void *cbContext);

static void _stream_b_callback(short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params, unsigned int numSamples,
    unsigned int reset, void *cbContext)
{
    // Not used in single-tuner mode — stub required by SDK
    (void)xi; (void)xq; (void)params; (void)numSamples; (void)reset; (void)cbContext;
}

static void _event_callback(sdrplay_api_EventT eventId,
    sdrplay_api_TunerSelectT tuner, sdrplay_api_EventParamsT *params,
    void *cbContext);

// Forward declarations for callback implementations (defined in sdr.cpp)
void stream_a_callback_impl(short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params, unsigned int numSamples);
void event_callback_impl(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tuner,
    sdrplay_api_EventParamsT *params);

// Inline wrapper definitions (same pattern as RspDuo.h)
static void _stream_a_callback(short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params, unsigned int numSamples,
    unsigned int reset, void *cbContext)
{
    (void)reset; (void)cbContext;
    stream_a_callback_impl(xi, xq, params, numSamples);
}

static void _event_callback(sdrplay_api_EventT eventId,
    sdrplay_api_TunerSelectT tuner, sdrplay_api_EventParamsT *params,
    void *cbContext)
{
    (void)cbContext;
    event_callback_impl(eventId, tuner, params);
}

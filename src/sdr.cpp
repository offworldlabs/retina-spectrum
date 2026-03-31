// Adapted from blah2-arm/src/capture/rspduo/RspDuo.cpp
// Changes from blah2:
//   get_device()          — Tuner_Both/Dual_Tuner → g_tuner/Single_Tuner
//   set_device_parameters() — BW_8_000, IF_Zero, decimation off, notches off, AGC 50Hz hardcoded
//   stream_a_callback_impl  — gutted dual-channel interleave, rfChanged-aware single accumulator
//   retune()               — new: sdrplay_api_Update + sets g_waiting_rf_change (no fixed sleep)
// Verbatim from blah2: open_api(), event_callback_impl(), initialise_device(), uninitialise_device()

#include "sdr.h"

#ifndef MOCK_ONLY

#include <chrono>
#include <iostream>
#include <thread>

// SDR globals (verbatim from RspDuo.cpp)
sdrplay_api_DeviceT        *chosenDevice = nullptr;
sdrplay_api_DeviceT         devs[1023];
sdrplay_api_DeviceParamsT  *deviceParams = nullptr;
sdrplay_api_ErrT            err;
sdrplay_api_CallbackFnsT    cbFns;
sdrplay_api_RxChannelParamsT *chParams   = nullptr;

// Tuner selection — default Tuner A (SMA1, reference antenna)
sdrplay_api_TunerSelectT    g_tuner = sdrplay_api_Tuner_A;

// Capture state
std::vector<std::complex<float>> g_capture_buf;
std::atomic<bool>                g_capture_done{false};
std::atomic<bool>                g_waiting_rf_change{false};

// ── open_api — verbatim from blah2 RspDuo.cpp ────────────────────────────────
void open_api()
{
    float ver = 0.0;
    if ((err = sdrplay_api_Open()) != sdrplay_api_Success)
    {
        std::cerr << "Error: API open failed " << sdrplay_api_GetErrorString(err) << std::endl;
        exit(1);
    }
    if ((err = sdrplay_api_ApiVersion(&ver)) != sdrplay_api_Success)
    {
        std::cerr << "Error: Set API version failed " << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_Close();
        exit(1);
    }
    if (ver != SDRPLAY_API_VERSION)
    {
        std::cerr << "Error: API versions do not match, local=" << SDRPLAY_API_VERSION
                  << " API=" << ver << std::endl;
        sdrplay_api_Close();
        exit(1);
    }
}

// ── get_device — one change from blah2: single tuner mode ────────────────────
void get_device()
{
    unsigned int ndev = 0;
    unsigned int chosenIdx = 0;

    // verbatim from blah2
    if ((err = sdrplay_api_LockDeviceApi()) != sdrplay_api_Success)
    {
        std::cerr << "Error: Lock API during device selection failed "
                  << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_Close();
        exit(1);
    }
    if ((err = sdrplay_api_GetDevices(devs, &ndev,
        sizeof(devs) / sizeof(sdrplay_api_DeviceT))) != sdrplay_api_Success)
    {
        std::cerr << "Error: sdrplay_api_GetDevices failed "
                  << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }
    std::cerr << "[sdr] MaxDevs=" << sizeof(devs) / sizeof(sdrplay_api_DeviceT)
              << " NumDevs=" << ndev << std::endl;
    if (ndev == 0)
    {
        std::cerr << "Error: No devices found" << std::endl;
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }

    // pick first RSPduo (verbatim from blah2)
    unsigned int i;
    for (i = 0; i < ndev; i++)
    {
        if (devs[i].hwVer == SDRPLAY_RSPduo_ID)
        {
            chosenIdx = i;
            break;
        }
    }
    if (i == ndev)
    {
        std::cerr << "Error: Could not find RSPduo device" << std::endl;
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }

    chosenDevice = &devs[chosenIdx];

    // CHANGED from blah2: single tuner (g_tuner = A or B) instead of Tuner_Both/Dual_Tuner
    chosenDevice->tuner      = g_tuner;
    chosenDevice->rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;

    std::cerr << "[sdr] SerNo=" << devs[chosenIdx].SerNo
              << " hwVer=" << (int)devs[chosenIdx].hwVer
              << " tuner=" << (g_tuner == sdrplay_api_Tuner_A ? "A" : "B") << std::endl;

    // verbatim from blah2
    if ((err = sdrplay_api_SelectDevice(chosenDevice)) != sdrplay_api_Success)
    {
        std::cerr << "Error: Select device failed " << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_UnlockDeviceApi();
        sdrplay_api_Close();
        exit(1);
    }
    if ((err = sdrplay_api_UnlockDeviceApi()) != sdrplay_api_Success)
    {
        std::cerr << "Error: Unlock device API failed " << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_Close();
        exit(1);
    }
}

// ── set_device_parameters — changed: BW_8_000, IF_Zero, notches off, AGC 50Hz ─
void set_device_parameters(double fc_hz)
{
    // verbatim from blah2
    if ((err = sdrplay_api_GetDeviceParams(chosenDevice->dev, &deviceParams)) != sdrplay_api_Success)
    {
        std::cerr << "Error: sdrplay_api_GetDeviceParams failed "
                  << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_Close();
        exit(1);
    }
    if (deviceParams == nullptr)
    {
        std::cerr << "Error: Device parameters pointer is null" << std::endl;
        sdrplay_api_Close();
        exit(1);
    }

    // verbatim from blah2 (isochronous USB mode)
    deviceParams->devParams->mode = sdrplay_api_ISOCH;

    chParams = deviceParams->rxChannelA;
    if (chParams == nullptr)
    {
        std::cerr << "Error: Channel parameters pointer is null" << std::endl;
        sdrplay_api_Close();
        exit(1);
    }

    chParams->tunerParams.rfFreq.rfHz = fc_hz;

    // CHANGED: 8 MHz bandwidth, zero-IF, no decimation
    chParams->tunerParams.bwType                     = sdrplay_api_BW_8_000;
    chParams->tunerParams.ifType                     = sdrplay_api_IF_Zero;
    chParams->ctrlParams.decimation.enable           = 0;
    chParams->ctrlParams.decimation.decimationFactor = 1;

    // CHANGED: AGC 50Hz hardcoded (blah2 has this configurable)
    chParams->ctrlParams.agc.enable        = sdrplay_api_AGC_50HZ;
    chParams->ctrlParams.agc.setPoint_dBfs = AGC_SETPOINT;

    // CHANGED: notches OFF — blah2 enables these to block FM/DAB for radar,
    //          we want to see FM and DAB bands
    chParams->rspDuoTunerParams.rfNotchEnable    = 0;
    chParams->rspDuoTunerParams.rfDabNotchEnable = 0;

    // Only StreamA used; StreamB stub required by SDK
    cbFns.StreamACbFn = _stream_a_callback;
    cbFns.StreamBCbFn = _stream_b_callback;
    cbFns.EventCbFn   = _event_callback;
}

// ── stream_a_callback_impl — gutted from blah2, rfChanged-aware accumulator ───
// API spec (section 2.10.2): params->rfChanged fires when sdrplay_api_Update_Tuner_Frf
// takes effect in the stream. This is the correct signal for retune completion.
// NOTE: reset parameter (section 3.21) only fires on sdrplay_api_Init, NOT on Update.
void stream_a_callback_impl(short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params, unsigned int numSamples)
{
    // Wait for retune to take effect — discard samples until rfChanged fires
    if (g_waiting_rf_change)
    {
        if (!params->rfChanged) return;     // retune not yet in stream — discard
        g_capture_buf.clear();
        g_waiting_rf_change = false;        // new frequency now valid in stream
        std::cerr << "[sdr] rfChanged received — capturing at new frequency" << std::endl;
        return;                             // skip this callback's samples — may be transitional
    }

    if (g_capture_done) return;

    for (unsigned int i = 0; i < numSamples; i++)
        g_capture_buf.push_back({(float)xi[i], (float)xq[i]});

    if ((int)g_capture_buf.size() >= N_FFT * N_AVG)
        g_capture_done = true;
}

// ── event_callback_impl — verbatim from blah2 RspDuo.cpp ─────────────────────
void event_callback_impl(sdrplay_api_EventT eventId,
    sdrplay_api_TunerSelectT tuner, sdrplay_api_EventParamsT *params)
{
    switch (eventId)
    {
    case sdrplay_api_GainChange:
        std::cerr << "[sdr] GainChange gRdB=" << params->gainParams.gRdB
                  << " lnaGRdB=" << params->gainParams.lnaGRdB << std::endl;
        break;
    case sdrplay_api_PowerOverloadChange:
        std::cerr << "[sdr] PowerOverloadChange" << std::endl;
        // verbatim from blah2: acknowledge the overload message
        sdrplay_api_Update(chosenDevice->dev, tuner,
            sdrplay_api_Update_Ctrl_OverloadMsgAck, sdrplay_api_Update_Ext1_None);
        break;
    case sdrplay_api_DeviceRemoved:
        std::cerr << "[sdr] Device removed!" << std::endl;
        break;
    default:
        std::cerr << "[sdr] Unknown event " << eventId << std::endl;
        break;
    }
}

// ── initialise_device — verbatim from blah2 ──────────────────────────────────
void initialise_device()
{
    if ((err = sdrplay_api_Init(chosenDevice->dev, &cbFns, nullptr)) != sdrplay_api_Success)
    {
        std::cerr << "Error: sdrplay_api_Init failed "
                  << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_Close();
        exit(1);
    }
}

// ── uninitialise_device — verbatim from blah2 ────────────────────────────────
void uninitialise_device()
{
    if ((err = sdrplay_api_Uninit(chosenDevice->dev)) != sdrplay_api_Success)
    {
        std::cerr << "Error: sdrplay_api_Uninit failed "
                  << sdrplay_api_GetErrorString(err) << std::endl;
    }
    sdrplay_api_ReleaseDevice(chosenDevice);
    sdrplay_api_Close();
}

// ── retune — new, not in blah2 (never retunes mid-stream) ────────────────────
void retune(double fc_hz)
{
    chParams->tunerParams.rfFreq.rfHz = fc_hz;
    g_capture_done     = false;
    g_waiting_rf_change = true;  // discard callbacks until params->rfChanged fires
    sdrplay_api_Update(chosenDevice->dev, g_tuner,
        sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
    // no sleep — sweep thread spins on g_capture_done with a timeout
}

#else  // MOCK_ONLY — no SDRplay API available

#include <atomic>
#include <complex>
#include <vector>

sdrplay_api_TunerSelectT         g_tuner           = sdrplay_api_Tuner_A;
std::vector<std::complex<float>> g_capture_buf;
std::atomic<bool>                g_capture_done{false};
std::atomic<bool>                g_waiting_rf_change{false};

void open_api()                    {}
void get_device()                  {}
void set_device_parameters(double) {}
void initialise_device()           {}
void uninitialise_device()         {}
void retune(double)                {}

#endif

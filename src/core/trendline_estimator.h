/*++

    Copyright (c) Microsoft Corporation.
    Licensed under the MIT License.

Abstract:

    Trendline Slope estimator for delay-based congestion detection.
    Based on the trendline filter used in GCC (Google Congestion Control).

--*/

#pragma once

#include "quic_platform.h"

#define TRENDLINE_WINDOW_SIZE 20              // Sliding window size
#define TRENDLINE_SMOOTHING_COEF 0.9          // Exponential smoothing coefficient
#define TRENDLINE_UPDATE_INTERVAL_US 50000    // 50ms update interval (similar to WebRTC)
#define TRENDLINE_MIN_SAMPLES_PER_UPDATE 2    // Minimum samples before updating slope

typedef enum TRENDLINE_OVERUSE_STATE {
    TRENDLINE_UNDERUSING = -1,
    TRENDLINE_NORMAL = 0,
    TRENDLINE_OVERUSING = 1
} TRENDLINE_OVERUSE_STATE;

typedef struct TRENDLINE_PACKET_TIMING {
    double ArrivalTimeMs;      // Arrival time (relative to first packet)
    double SmoothedDelayMs;    // Smoothed delay
    double RawDelayMs;         // Raw delay
} TRENDLINE_PACKET_TIMING;

typedef struct TRENDLINE_ESTIMATOR {
    //
    // Sliding window storing packet timing info
    //
    TRENDLINE_PACKET_TIMING DelayHist[TRENDLINE_WINDOW_SIZE];
    uint32_t DelayHistHead;    // Ring buffer head
    uint32_t DelayHistCount;   // Current sample count

    //
    // Delay accumulation
    //
    double AccumulatedDelayMs;
    double SmoothedDelayMs;
    int64_t FirstArrivalTimeUs;

    //
    // Trendline slope output
    //
    double TrendlineSlope;     // Current slope
    double PrevTrendlineSlope; // Previous slope

    //
    // Congestion detection
    //
    double Threshold;          // Adaptive threshold
    int64_t LastUpdateTimeUs;
    int32_t NumDeltas;
    double ModifiedTrend;      // Scaled slope used for overuse detection
    double TimeOverUsingMs;    // Time spent above the overuse threshold
    int32_t OveruseCounter;    // Consecutive over-threshold updates
    TRENDLINE_OVERUSE_STATE OveruseState;

    //
    // Interval-based update control (similar to WebRTC packet grouping)
    //
    int64_t LastSlopeUpdateTimeUs;    // Last time slope was recalculated
    int32_t SamplesInCurrentInterval; // Samples collected in current interval
    double IntervalAccumulatedDelay;  // Accumulated delay in current interval

    //
    // Batch-level tracking (fix for ACK batching bias)
    //
    uint64_t PrevBatchSendTimeUs;     // Previous batch's last send time
    uint64_t PrevBatchRecvTimeUs;     // Previous batch's ACK arrival time

} TRENDLINE_ESTIMATOR;

//
// Initialize the trendline estimator
//
void
TrendlineEstimatorInitialize(
    _Out_ TRENDLINE_ESTIMATOR* Estimator
    );

//
// Update - call on each ACK (legacy per-packet method)
// SendDeltaUs: sender packet interval
// RecvDeltaUs: receiver packet interval
// ArrivalTimeUs: arrival timestamp
//
void
TrendlineEstimatorUpdate(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator,
    _In_ int64_t SendDeltaUs,
    _In_ int64_t RecvDeltaUs,
    _In_ int64_t ArrivalTimeUs
    );

//
// Update at batch level - call once per ACK event (recommended)
// BatchSendTimeUs: last packet's send time in the batch
// BatchRecvTimeUs: ACK arrival time
// BatchBytes: total bytes in the batch (optional, for future use)
//
void
TrendlineEstimatorUpdateBatch(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator,
    _In_ uint64_t BatchSendTimeUs,
    _In_ uint64_t BatchRecvTimeUs,
    _In_ uint32_t BatchBytes
    );

//
// Get current slope
//
double
TrendlineEstimatorGetSlope(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    );

//
// Get current overuse detector state
//
TRENDLINE_OVERUSE_STATE
TrendlineEstimatorGetOveruseState(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    );

BOOLEAN
TrendlineEstimatorIsOverusing(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    );

const char*
TrendlineEstimatorGetOveruseStateName(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    );

//
// Reset the estimator
//
void
TrendlineEstimatorReset(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator
    );

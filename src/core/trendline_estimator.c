/*++

    Copyright (c) Microsoft Corporation.
    Licensed under the MIT License.

Abstract:

    Trendline Slope estimator implementation.
    Uses linear least-squares regression to compute delay trend.

--*/

#include "precomp.h"
#include "trendline_estimator.h"
#include <math.h>

#ifdef QUIC_CLOG
#include "trendline_estimator.c.clog.h"
#endif

#define TRENDLINE_THRESHOLD_GAIN 4.0
#define TRENDLINE_MIN_NUM_DELTAS 60
#define TRENDLINE_OVERUSING_TIME_THRESHOLD_MS 10.0
#define TRENDLINE_OVERUSE_COUNTER_THRESHOLD 1
#define K_UP   0.0087
#define K_DOWN 0.039

void
TrendlineEstimatorInitialize(
    _Out_ TRENDLINE_ESTIMATOR* Estimator
    )
{
    CxPlatZeroMemory(Estimator, sizeof(*Estimator));
    Estimator->FirstArrivalTimeUs = -1;
    Estimator->Threshold = 12.5;
    Estimator->LastUpdateTimeUs = -1;
    Estimator->LastSlopeUpdateTimeUs = -1;
    Estimator->SamplesInCurrentInterval = 0;
    Estimator->IntervalAccumulatedDelay = 0.0;
    Estimator->OveruseState = TRENDLINE_NORMAL;
}

//
// Linear least-squares slope calculation
//
static double
LinearFitSlope(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    )
{
    if (Estimator->DelayHistCount < 2) {
        return 0.0;
    }

    // Calculate centroid
    double sumX = 0.0, sumY = 0.0;
    for (uint32_t i = 0; i < Estimator->DelayHistCount; i++) {
        uint32_t idx = (Estimator->DelayHistHead + i) % TRENDLINE_WINDOW_SIZE;
        sumX += Estimator->DelayHist[idx].ArrivalTimeMs;
        sumY += Estimator->DelayHist[idx].SmoothedDelayMs;
    }
    double xAvg = sumX / Estimator->DelayHistCount;
    double yAvg = sumY / Estimator->DelayHistCount;

    // Calculate slope k = sum((x-x_avg)(y-y_avg)) / sum((x-x_avg)^2)
    double numerator = 0.0, denominator = 0.0;
    for (uint32_t i = 0; i < Estimator->DelayHistCount; i++) {
        uint32_t idx = (Estimator->DelayHistHead + i) % TRENDLINE_WINDOW_SIZE;
        double x = Estimator->DelayHist[idx].ArrivalTimeMs;
        double y = Estimator->DelayHist[idx].SmoothedDelayMs;
        numerator += (x - xAvg) * (y - yAvg);
        denominator += (x - xAvg) * (x - xAvg);
    }

    if (denominator == 0.0) {
        return 0.0;
    }
    return numerator / denominator;
}

static void
TrendlineEstimatorDetectOveruse(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator,
    _In_ double TimeDeltaMs
    )
{
    if (Estimator->DelayHistCount < TRENDLINE_WINDOW_SIZE) {
        Estimator->ModifiedTrend = 0.0;
        Estimator->OveruseState = TRENDLINE_NORMAL;
        return;
    }

    uint32_t NumDeltas =
        Estimator->NumDeltas < TRENDLINE_MIN_NUM_DELTAS ?
            (uint32_t)Estimator->NumDeltas : TRENDLINE_MIN_NUM_DELTAS;

    Estimator->ModifiedTrend =
        (double)NumDeltas * Estimator->TrendlineSlope * TRENDLINE_THRESHOLD_GAIN;

    if (Estimator->ModifiedTrend > Estimator->Threshold) {
        Estimator->TimeOverUsingMs += TimeDeltaMs;
        Estimator->OveruseCounter++;

        if (Estimator->TimeOverUsingMs > TRENDLINE_OVERUSING_TIME_THRESHOLD_MS &&
            Estimator->OveruseCounter > TRENDLINE_OVERUSE_COUNTER_THRESHOLD &&
            Estimator->TrendlineSlope >= Estimator->PrevTrendlineSlope) {
            Estimator->OveruseState = TRENDLINE_OVERUSING;
        }
    } else if (Estimator->ModifiedTrend < -Estimator->Threshold) {
        Estimator->TimeOverUsingMs = 0.0;
        Estimator->OveruseCounter = 0;
        Estimator->OveruseState = TRENDLINE_UNDERUSING;
    } else {
        Estimator->TimeOverUsingMs = 0.0;
        Estimator->OveruseCounter = 0;
        Estimator->OveruseState = TRENDLINE_NORMAL;
    }
}

void
TrendlineEstimatorUpdate(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator,
    _In_ int64_t SendDeltaUs,
    _In_ int64_t RecvDeltaUs,
    _In_ int64_t ArrivalTimeUs
    )
{
    // Calculate one-way delay variation (recv_delta - send_delta)
    double deltaMs = (double)(RecvDeltaUs - SendDeltaUs) / 1000.0;

    Estimator->NumDeltas++;
    if (Estimator->NumDeltas > 1000) {
        Estimator->NumDeltas = 1000;
    }

    if (Estimator->FirstArrivalTimeUs < 0) {
        Estimator->FirstArrivalTimeUs = ArrivalTimeUs;
    }

    // Initialize last slope update time
    if (Estimator->LastSlopeUpdateTimeUs < 0) {
        Estimator->LastSlopeUpdateTimeUs = ArrivalTimeUs;
    }

    // Accumulate delay for current interval
    Estimator->IntervalAccumulatedDelay += deltaMs;
    Estimator->SamplesInCurrentInterval++;

    // Check if we should update (50ms interval passed and have enough samples)
    int64_t timeSinceLastUpdate = ArrivalTimeUs - Estimator->LastSlopeUpdateTimeUs;
    BOOLEAN shouldUpdate = (timeSinceLastUpdate >= TRENDLINE_UPDATE_INTERVAL_US) &&
                           (Estimator->SamplesInCurrentInterval >= TRENDLINE_MIN_SAMPLES_PER_UPDATE);

    if (shouldUpdate) {
        // Calculate average delay for this interval
        double avgDeltaMs = Estimator->IntervalAccumulatedDelay / Estimator->SamplesInCurrentInterval;

        // Apply exponential smoothing on accumulated delay
        Estimator->AccumulatedDelayMs += avgDeltaMs;
        Estimator->SmoothedDelayMs =
            TRENDLINE_SMOOTHING_COEF * Estimator->SmoothedDelayMs +
            (1.0 - TRENDLINE_SMOOTHING_COEF) * Estimator->AccumulatedDelayMs;

        // Add to sliding window
        double arrivalTimeMs = (double)(ArrivalTimeUs - Estimator->FirstArrivalTimeUs) / 1000.0;

        uint32_t insertIdx;
        if (Estimator->DelayHistCount < TRENDLINE_WINDOW_SIZE) {
            insertIdx = Estimator->DelayHistCount;
            Estimator->DelayHistCount++;
        } else {
            // Window full, overwrite oldest
            insertIdx = Estimator->DelayHistHead;
            Estimator->DelayHistHead = (Estimator->DelayHistHead + 1) % TRENDLINE_WINDOW_SIZE;
        }

        Estimator->DelayHist[insertIdx].ArrivalTimeMs = arrivalTimeMs;
        Estimator->DelayHist[insertIdx].SmoothedDelayMs = Estimator->SmoothedDelayMs;
        Estimator->DelayHist[insertIdx].RawDelayMs = Estimator->AccumulatedDelayMs;

        // Calculate slope when window is full
        Estimator->PrevTrendlineSlope = Estimator->TrendlineSlope;
        if (Estimator->DelayHistCount >= TRENDLINE_WINDOW_SIZE) {
            Estimator->TrendlineSlope = LinearFitSlope(Estimator);
        }
        TrendlineEstimatorDetectOveruse(
            Estimator,
            (double)timeSinceLastUpdate / 1000.0);

        // Reset interval counters
        Estimator->LastSlopeUpdateTimeUs = ArrivalTimeUs;
        Estimator->SamplesInCurrentInterval = 0;
        Estimator->IntervalAccumulatedDelay = 0.0;
    }

    Estimator->LastUpdateTimeUs = ArrivalTimeUs;
}

//
// Batch-level update - call once per ACK event
// This fixes the ACK batching bias where multiple packets in the same ACK
// get the same RecvTime, causing incorrect delta calculations.
//
void
TrendlineEstimatorUpdateBatch(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator,
    _In_ uint64_t BatchSendTimeUs,
    _In_ uint64_t BatchRecvTimeUs,
    _In_ uint32_t BatchBytes
    )
{
    UNREFERENCED_PARAMETER(BatchBytes);

    // First batch - just record, don't calculate
    if (Estimator->PrevBatchSendTimeUs == 0) {
        Estimator->PrevBatchSendTimeUs = BatchSendTimeUs;
        Estimator->PrevBatchRecvTimeUs = BatchRecvTimeUs;
        Estimator->FirstArrivalTimeUs = (int64_t)BatchRecvTimeUs;
        return;
    }

    // Calculate inter-batch time deltas
    int64_t SendDeltaUs = (int64_t)(BatchSendTimeUs - Estimator->PrevBatchSendTimeUs);
    int64_t RecvDeltaUs = (int64_t)(BatchRecvTimeUs - Estimator->PrevBatchRecvTimeUs);

    // Filter out invalid deltas (negative or zero)
    if (SendDeltaUs <= 0 || RecvDeltaUs <= 0) {
        Estimator->PrevBatchSendTimeUs = BatchSendTimeUs;
        Estimator->PrevBatchRecvTimeUs = BatchRecvTimeUs;
        return;
    }

    Estimator->NumDeltas++;
    if (Estimator->NumDeltas > 1000) {
        Estimator->NumDeltas = 1000;
    }

    // Calculate one-way delay variation
    double deltaMs = (double)(RecvDeltaUs - SendDeltaUs) / 1000.0;

    // Accumulate delay
    Estimator->AccumulatedDelayMs += deltaMs;

    // EMA smoothing
    Estimator->SmoothedDelayMs =
        TRENDLINE_SMOOTHING_COEF * Estimator->SmoothedDelayMs +
        (1.0 - TRENDLINE_SMOOTHING_COEF) * Estimator->AccumulatedDelayMs;

    // Add to sliding window
    double arrivalTimeMs = (double)(BatchRecvTimeUs - Estimator->FirstArrivalTimeUs) / 1000.0;

    uint32_t insertIdx;
    if (Estimator->DelayHistCount < TRENDLINE_WINDOW_SIZE) {
        insertIdx = Estimator->DelayHistCount;
        Estimator->DelayHistCount++;
    } else {
        insertIdx = Estimator->DelayHistHead;
        Estimator->DelayHistHead = (Estimator->DelayHistHead + 1) % TRENDLINE_WINDOW_SIZE;
    }

    Estimator->DelayHist[insertIdx].ArrivalTimeMs = arrivalTimeMs;
    Estimator->DelayHist[insertIdx].SmoothedDelayMs = Estimator->SmoothedDelayMs;
    Estimator->DelayHist[insertIdx].RawDelayMs = Estimator->AccumulatedDelayMs;

    // Calculate slope when window has enough samples
    Estimator->PrevTrendlineSlope = Estimator->TrendlineSlope;
    if (Estimator->DelayHistCount >= TRENDLINE_WINDOW_SIZE) {
        Estimator->TrendlineSlope = LinearFitSlope(Estimator);
    }
    TrendlineEstimatorDetectOveruse(
        Estimator,
        (double)RecvDeltaUs / 1000.0);

    // Update previous batch info
    Estimator->PrevBatchSendTimeUs = BatchSendTimeUs;
    Estimator->PrevBatchRecvTimeUs = BatchRecvTimeUs;
    Estimator->LastUpdateTimeUs = (int64_t)BatchRecvTimeUs;
}

double
TrendlineEstimatorGetSlope(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    )
{
    return Estimator->TrendlineSlope;
}

TRENDLINE_OVERUSE_STATE
TrendlineEstimatorGetOveruseState(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    )
{
    return Estimator->OveruseState;
}

BOOLEAN
TrendlineEstimatorIsOverusing(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    )
{
    return Estimator->OveruseState == TRENDLINE_OVERUSING;
}

const char*
TrendlineEstimatorGetOveruseStateName(
    _In_ const TRENDLINE_ESTIMATOR* Estimator
    )
{
    switch (Estimator->OveruseState) {
    case TRENDLINE_OVERUSING:
        return "OVERUSING";
    case TRENDLINE_UNDERUSING:
        return "UNDERUSING";
    case TRENDLINE_NORMAL:
    default:
        return "NORMAL";
    }
}

void
TrendlineEstimatorReset(
    _Inout_ TRENDLINE_ESTIMATOR* Estimator
    )
{
    TrendlineEstimatorInitialize(Estimator);
}

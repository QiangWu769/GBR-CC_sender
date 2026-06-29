/*++

    Copyright (c) Microsoft Corporation.
    Licensed under the MIT License.

--*/

#pragma once

#include "sliding_window_extremum.h"
#include "trendline_estimator.h"

#define kBbrDefaultFilterCapacity 3

typedef struct BBR_BANDWIDTH_FILTER {

    //
    // TRUE if bandwidth is limited by the application
    //
    BOOLEAN AppLimited : 1;

    //
    // Target packet number to quit the AppLimited state
    //
    uint64_t AppLimitedExitTarget;

    //
    // Max filter for tracking the maximum recent delivery_rate sample, for estimating max bandwidth
    //
    QUIC_SLIDING_WINDOW_EXTREMUM WindowedMaxFilter;

    QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY WindowedMaxFilterEntries[kBbrDefaultFilterCapacity];

} BBR_BANDWIDTH_FILTER;

typedef struct QUIC_CONGESTION_CONTROL_BBR {

    //
    // Whether the bottleneck bandwidth has been detected
    //
    BOOLEAN BtlbwFound : 1;

    //
    // TRUE when exiting quiescence
    //
    BOOLEAN ExitingQuiescence : 1;

    //
    // If TRUE, EndOfRecovery is valid
    //
    BOOLEAN EndOfRecoveryValid : 1;

    //
    // If TRUE, EndOfRoundTrip is valid
    //
    BOOLEAN EndOfRoundTripValid : 1;

    //
    // If TRUE, AckAggregationStartTime is valid
    //
    BOOLEAN AckAggregationStartTimeValid : 1;

    //
    // If TRUE, ProbeRttRound is valid
    //
    BOOLEAN ProbeRttRoundValid : 1;

    //
    // If TRUE, ProbeRttEndTime is valid
    //
    BOOLEAN ProbeRttEndTimeValid : 1;

    //
    // If TRUE, current RTT sample is expired
    //
    BOOLEAN RttSampleExpired: 1;

    //
    // If TRUE, there has been at least one MinRtt sample
    //
    BOOLEAN MinRttTimestampValid: 1;

    //
    // The size of the initial congestion window in packets
    //
    uint32_t InitialCongestionWindowPackets;

    uint32_t CongestionWindow; // bytes

    uint32_t InitialCongestionWindow; // bytes

    uint32_t RecoveryWindow; // bytes

    //
    // The number of bytes considered to be still in the network.
    //
    // The client of this module should send packets until BytesInFlight becomes
    // larger than CongestionWindow (see QuicCongestionControlCanSend). This
    // means BytesInFlight can become larger than CongestionWindow by up to one
    // packet's worth of bytes, plus exemptions (see Exemptions variable).
    //
    uint32_t BytesInFlight;
    uint32_t BytesInFlightMax;

    //
    // A count of packets which can be sent ignoring CongestionWindow.
    // The count is decremented as the packets are sent. BytesInFlight is still
    // incremented for these packets. This is used to send probe packets for
    // loss recovery.
    //
    uint8_t Exemptions;

    //
    // Count of packet-timed round trips
    //
    uint64_t RoundTripCounter;

    //
    // The dynamic gain factor used to scale the estimated BDP to produce a
    // congestion window (cwnd)
    //
    uint32_t CwndGain;

    //
    // The dynamic gain factor used to scale bottleneck bandwidth to produce the
    // pacing rate
    //
    uint32_t PacingGain;

    //
    // The dynamic send quantum specifies the maximum size of these transmission
    // aggregates
    //
    uint64_t SendQuantum;

    //
    // Counter of continuous round trips in STARTUP
    //
    uint8_t SlowStartupRoundCounter;

    //
    // Current cycle index in kPacingGain
    //
    uint32_t PacingCycleIndex;

    //
    // Starting time of ack aggregation
    //
    uint64_t AckAggregationStartTime;

    //
    // Number of bytes acked during this aggregation
    //
    uint64_t AggregatedAckBytes;

    //
    // Current state of recovery
    //
    uint32_t RecoveryState;

    //
    // Current state of BBR state machine
    //
    uint32_t BbrState;

    //
    // The time at which the last pacing gain cycle was started
    //
    uint64_t CycleStart;

    //
    // Receiving acknowledgment of a packet after EndoOfRoundTrip will
    // indicate the current round trip is ended
    //
    uint64_t EndOfRoundTrip;

    //
    // Receiving acknowledgment of a packet after EndoOfRecovery will cause
    // BBR to exit the recovery mode
    //
    uint64_t EndOfRecovery;

    //
    // The bandwidth of last during STARTUP state
    //
    uint64_t LastEstimatedStartupBandwidth;

    //
    // Indicates whether to exit ProbeRtt if there're at least one RTT round with the
    // minimum cwnd
    //
    uint64_t ProbeRttRound;

    //
    // Indicates the eariest time to exit ProbeRTT state
    //
    uint64_t ProbeRttEndTime;

    //
    // The max filter tracking the recent maximum degree of aggregation in the path
    //
    QUIC_SLIDING_WINDOW_EXTREMUM MaxAckHeightFilter;
    QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY MaxAckHeightFilterEntries[kBbrDefaultFilterCapacity];

    uint64_t MinRtt; // microseconds

    //
    // Time when MinRtt was sampled. Only valid if MinRttTimestampValid is set.
    //
    uint64_t MinRttTimestamp; // microseconds

    //
    // BBR estimates maximum bandwidth by the maximum recent bandwidth
    //
    BBR_BANDWIDTH_FILTER BandwidthFilter;

    //
    // Recent delivery rate tracking for logging
    //
    uint64_t RecentSendRate;    // Most recent send rate in bps
    uint64_t RecentAckRate;     // Most recent ack rate in bps
    uint64_t RecentDeliveryRate; // Most recent delivery rate in bps

    //
    // Periodic logging fields
    //
    uint64_t LastPeriodicLogTime;   // Last time periodic log was generated (microseconds)
    uint64_t LastLoggedSendBytes;   // Send bytes at last log
    uint64_t LastLoggedRecvBytes;   // Receive bytes at last log
    uint64_t LastLoggedSentPackets; // Sent packets at last log
    uint64_t LastLoggedLostPackets; // Lost packets at last log

    //
    // Send and Ack delay tracking for delivery rate calculation
    //
    uint64_t RecentSendDelay;       // Send delay used in recent delivery rate calculation (microseconds)
    uint64_t RecentAckDelay;        // Ack delay used in recent delivery rate calculation (microseconds)

    //
    // Cellular outer-loop gating. The delay-based overuse detector only
    // switches out of the GBR inner loop after the cellular ratio has stayed
    // above the target threshold for a short guard interval.
    //
    BOOLEAN CellularRatioAboveThresholdStartTimeValid;
    uint64_t CellularRatioAboveThresholdStartTime;
    BOOLEAN CellularOveruseStartTimeValid;
    uint64_t CellularOveruseStartTime;
    BOOLEAN CellularOuterLoopLastBackoffTimeValid;
    uint64_t CellularOuterLoopLastBackoffTime;

    //
    // Trendline slope estimator for delay-based congestion detection
    //
    TRENDLINE_ESTIMATOR TrendlineEstimator;

    //
    // Previous packet send/receive times for calculating deltas
    //
    uint64_t PrevSendTimeUs;
    uint64_t PrevRecvTimeUs;

} QUIC_CONGESTION_CONTROL_BBR;

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlInitialize(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ const QUIC_SETTINGS_INTERNAL* Settings
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlPeriodicLog(
    _In_ QUIC_CONGESTION_CONTROL* Cc
    );

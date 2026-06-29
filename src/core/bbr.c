/*++

    Copyright (c) Microsoft Corporation.
    Licensed under the MIT License.

Abstract:

    Bottleneck Bandwidth and RTT (BBR) congestion control.

--*/

#include "precomp.h"
#include "trendline_estimator.h"
#include <math.h>
#include <time.h>
#ifdef QUIC_CLOG
#include "bbr.c.clog.h"
#endif
#ifdef QUIC_ENHANCED_PACKET_LOGGING
#include "bbr_packet_level_logging.h"
#endif

//
// Cellular ratio input (weak symbols provided by perf cellular module)
//
extern volatile double g_CellularRatioValue __attribute__((weak));
extern volatile double g_CellularRawRatioValue __attribute__((weak));
extern volatile uint64_t g_CellularRatioUpdateSeq __attribute__((weak));
extern volatile int g_CellularRatioAvailable __attribute__((weak));
extern volatile int g_CellularControlEnabled __attribute__((weak));
extern volatile int g_TestModeEnabled __attribute__((weak));

static inline int
IsCellularRatioAvailable(void) {
    if (&g_CellularRatioAvailable == NULL) {
        return 0;
    }
    return g_CellularRatioAvailable != 0;
}

static inline double
GetCellularRatio(void) {
    return IsCellularRatioAvailable() ? g_CellularRatioValue : 1.0;
}

static inline double
GetCellularRawRatio(void) {
    if (&g_CellularRawRatioValue == NULL) {
        return GetCellularRatio();
    }
    return IsCellularRatioAvailable() ? g_CellularRawRatioValue : 1.0;
}

static inline uint64_t
GetCellularRatioUpdateSeq(void) {
    if (&g_CellularRatioUpdateSeq == NULL) {
        return 0;
    }
    return IsCellularRatioAvailable() ? g_CellularRatioUpdateSeq : 0;
}

static inline int
IsCellularControlEnabled(void) {
    if (&g_CellularControlEnabled == NULL) {
        return 0;
    }
    return g_CellularControlEnabled != 0;
}

#define CELLULAR_OUTER_LOOP_RATIO_THRESHOLD 0.2
#define CELLULAR_OUTER_LOOP_RATIO_HOLD_US 100000
#define CELLULAR_OUTER_LOOP_OVERUSE_HOLD_US 100000
#define CELLULAR_OUTER_LOOP_BACKOFF_COOLDOWN_US 200000
#define CELLULAR_OUTER_LOOP_BACKOFF_FACTOR 0.95
#define CELLULAR_OUTER_LOOP_MAX_REDUCTION_PER_BACKOFF 0.95
#define CELLULAR_OUTER_LOOP_BACKOFF_MARGIN_BYTES_PER_SEC 625.0
#define CELLULAR_OUTER_LOOP_MIN_REDUCTION_INTERVAL_US 10000
#define CELLULAR_OUTER_LOOP_MAX_REDUCTION_INTERVAL_US 200000
#define CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC 100000.0

static BOOLEAN
IsCellularControlActive(
    void
    )
{
    return IsCellularRatioAvailable() && IsCellularControlEnabled();
}

static BOOLEAN
IsCellularOuterLoopRatioGateOpen(
    _In_ const QUIC_CONGESTION_CONTROL_BBR* Bbr
    )
{
    return Bbr->CellularRatioAboveThresholdStartTimeValid &&
           CxPlatTimeDiff64(
               Bbr->CellularRatioAboveThresholdStartTime,
               CxPlatTimeUs64()) >= CELLULAR_OUTER_LOOP_RATIO_HOLD_US;
}

static void
UpdateCellularOuterLoopRatioGate(
    _In_ QUIC_CONGESTION_CONTROL_BBR* Bbr,
    _In_ uint64_t TimeNow
    )
{
    if (!IsCellularControlActive() ||
        GetCellularRatio() <= CELLULAR_OUTER_LOOP_RATIO_THRESHOLD) {
        Bbr->CellularRatioAboveThresholdStartTimeValid = FALSE;
        Bbr->CellularRatioAboveThresholdStartTime = 0;
        return;
    }

    if (!Bbr->CellularRatioAboveThresholdStartTimeValid) {
        Bbr->CellularRatioAboveThresholdStartTimeValid = TRUE;
        Bbr->CellularRatioAboveThresholdStartTime = TimeNow;
    }
}

static BOOLEAN
IsCellularOuterLoopOveruseGateOpen(
    _In_ const QUIC_CONGESTION_CONTROL_BBR* Bbr
    )
{
    return Bbr->CellularOveruseStartTimeValid &&
           CxPlatTimeDiff64(
               Bbr->CellularOveruseStartTime,
               CxPlatTimeUs64()) >= CELLULAR_OUTER_LOOP_OVERUSE_HOLD_US;
}

static BOOLEAN
IsCellularOuterLoopBackoffCooldownActive(
    _In_ const QUIC_CONGESTION_CONTROL_BBR* Bbr
    )
{
    return Bbr->CellularOuterLoopLastBackoffTimeValid &&
           CxPlatTimeDiff64(
               Bbr->CellularOuterLoopLastBackoffTime,
               CxPlatTimeUs64()) < CELLULAR_OUTER_LOOP_BACKOFF_COOLDOWN_US;
}

static void
UpdateCellularOuterLoopOveruseGate(
    _In_ QUIC_CONGESTION_CONTROL_BBR* Bbr,
    _In_ uint64_t TimeNow
    )
{
    if (!IsCellularControlActive() ||
        !TrendlineEstimatorIsOverusing(&Bbr->TrendlineEstimator)) {
        Bbr->CellularOveruseStartTimeValid = FALSE;
        Bbr->CellularOveruseStartTime = 0;
        return;
    }

    if (!Bbr->CellularOveruseStartTimeValid) {
        Bbr->CellularOveruseStartTimeValid = TRUE;
        Bbr->CellularOveruseStartTime = TimeNow;
    }
}

static BOOLEAN
IsCellularOuterLoopActive(
    _In_ const QUIC_CONGESTION_CONTROL_BBR* Bbr
    )
{
    BOOLEAN OuterLoopCondition =
        IsCellularControlActive() &&
        GetCellularRatio() > CELLULAR_OUTER_LOOP_RATIO_THRESHOLD &&
        IsCellularOuterLoopRatioGateOpen(Bbr) &&
        TrendlineEstimatorIsOverusing(&Bbr->TrendlineEstimator) &&
        IsCellularOuterLoopOveruseGateOpen(Bbr) &&
        !IsCellularOuterLoopBackoffCooldownActive(Bbr);
    UNREFERENCED_PARAMETER(OuterLoopCondition);
    return FALSE;
}

static BOOLEAN
IsCellularInnerLoopActive(
    _In_ const QUIC_CONGESTION_CONTROL_BBR* Bbr
    )
{
    return IsCellularControlActive() && !IsCellularOuterLoopActive(Bbr);
}

static inline int
IsTestModeEnabled(void) {
    if (&g_TestModeEnabled == NULL) {
        return 0;
    }
    return g_TestModeEnabled != 0;
}

//==============================================================================
// Test Mode: Step-wise Rate Ramp with variable time per level
// 0-60s:  0-3 Mbps,  60-80s: 3-6 Mbps,  80-100s: 6-10 Mbps
//==============================================================================

#define TEST_STEP0_DURATION_US  60000000  // 60 seconds for first step
#define TEST_STEP_DURATION_US   20000000  // 20 seconds for other steps
#define TEST_LOG_PATH           "/home/qwu26/msquic_cellular/artifacts/bbr_logs/test_log.txt"

// Rate steps in bytes/sec: 3, 6, 10 Mbps
static const uint64_t g_TestRateSteps[] = {
    3 * 1000 * 1000 / 8,   // Step 0: 0-3 Mbps -> 375000 bytes/sec
    6 * 1000 * 1000 / 8,   // Step 1: 3-6 Mbps -> 750000 bytes/sec
    10 * 1000 * 1000 / 8   // Step 2: 6-10 Mbps -> 1250000 bytes/sec
};
#define TEST_NUM_STEPS 3

static uint64_t g_TestStartTimeUs = 0;
static int g_TestInitialized = 0;
static int g_TestLogInitialized = 0;

static uint64_t
TestGetCurrentRateBytes(void) {
    if (!g_TestInitialized) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        g_TestStartTimeUs = (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
        g_TestInitialized = 1;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t nowUs = (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
    uint64_t elapsedUs = nowUs - g_TestStartTimeUs;

    int step;
    uint64_t stepElapsedUs;
    uint64_t stepDuration;

    // First step is 60s, others are 20s each
    if (elapsedUs < TEST_STEP0_DURATION_US) {
        step = 0;
        stepElapsedUs = elapsedUs;
        stepDuration = TEST_STEP0_DURATION_US;
    } else {
        uint64_t afterStep0 = elapsedUs - TEST_STEP0_DURATION_US;
        step = 1 + (int)(afterStep0 / TEST_STEP_DURATION_US);
        if (step >= TEST_NUM_STEPS) {
            step = TEST_NUM_STEPS - 1;
        }
        stepElapsedUs = afterStep0 % TEST_STEP_DURATION_US;
        stepDuration = TEST_STEP_DURATION_US;
    }

    // Get rate range for this step
    uint64_t rateStart = (step == 0) ? 0 : g_TestRateSteps[step - 1];
    uint64_t rateEnd = g_TestRateSteps[step];

    // Linear interpolation within step
    return rateStart + (stepElapsedUs * (rateEnd - rateStart)) / stepDuration;
}

static double
TestGetElapsedSec(void) {
    if (!g_TestInitialized) return 0.0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t nowUs = (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
    return (double)(nowUs - g_TestStartTimeUs) / 1000000.0;
}

//==============================================================================
// Ratio-Based Congestion Control
//==============================================================================

#define RATIO_BW_UNIT 8
#define RATIO_MICROSECS_IN_SEC 1000000

static double g_RatioPacingRate = 0.0;  // bytes/sec
static uint64_t g_LastRateUpdateTimeUs = 0;
static uint64_t g_LastProcessedRatioUpdateSeq = 0;
static uint64_t g_RatioBaseRttUs = 0;

//
// Gain映射函数（两段Sigmoid映射）
// ratio 范围: 0 ~ 2
// gain 范围: 0.99 ~ 1.01
//
// 两段式设计 (threshold=0.2):
//   r <= threshold: gain = 0.99 ~ 1.0 (减速区)
//   r > threshold:  gain = 1.0 ~ 1.01 (加速区)
//
static inline double
Sigmoid(double x) {
    return 1.0 / (1.0 + exp(-x));
}

static inline double
GetRatioGain(double ratio) {
    // 参数配置
    const double k = 25.0;            // sigmoid 陡峭程度
    const double threshold = 0.2;     // 分界点
    const double center_low = 0.15;   // 减速区 sigmoid 中心点
    const double center_high = 0.25;  // 加速区 sigmoid 中心点

    // 预计算归一化常数
    double s_low_0  = Sigmoid(k * (0.0 - center_low));
    double s_low_th = Sigmoid(k * (threshold - center_low));
    double s_high_th = Sigmoid(k * (threshold - center_high));
    double s_high_2  = Sigmoid(k * (2.0 - center_high));

    double gain;

    if (ratio <= threshold) {
        // 减速区: gain 从 0.99 (r=0) 平滑过渡到 1.0 (r=threshold)
        double s_r = Sigmoid(k * (ratio - center_low));
        double normalized = (s_r - s_low_0) / (s_low_th - s_low_0);
        gain = 0.99 + 0.01 * normalized;
    } else {
        // 加速区: gain 从 1.0 (r=threshold) 平滑过渡到 1.01 (r=2)
        double s_r = Sigmoid(k * (ratio - center_high));
        double normalized = (s_r - s_high_th) / (s_high_2 - s_high_th);
        gain = 1.0 + 0.01 * normalized;
    }

    return gain;
}

//
// 获取当前 Cellular Gain
//
static inline double
GetCellularGain(void) {
    double ratio = GetCellularRatio();
    return GetRatioGain(ratio);
}

//
// 更新 Ratio-based pacing rate
//
static inline void
UpdateRatioPacingRate(double ratio, uint64_t BandwidthEst) {
    if (g_RatioPacingRate <= 0.0) {
        // 初始化：使用当前带宽估计
        g_RatioPacingRate = (double)BandwidthEst / RATIO_BW_UNIT;
        if (g_RatioPacingRate < CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC) {
            g_RatioPacingRate = CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC;  // 最小 100 KB/s
        }
    }

    // 获取时间差
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t nowUs = (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;

    if (g_LastRateUpdateTimeUs == 0) {
        g_LastRateUpdateTimeUs = nowUs;
        return;
    }

    double timeDeltaSec = (double)(nowUs - g_LastRateUpdateTimeUs) / 1000000.0;
    g_LastRateUpdateTimeUs = nowUs;

    if (timeDeltaSec <= 0.0 || timeDeltaSec > 1.0) {
        timeDeltaSec = 0.01;
    }

    // 使用 gain 更新速率
    double gain = GetRatioGain(ratio);
    g_RatioPacingRate *= gain;

    // 限制范围
    if (g_RatioPacingRate < CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC) {
        g_RatioPacingRate = CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC;
    }
    if (g_RatioPacingRate > 100000000.0) {
        g_RatioPacingRate = 100000000.0;
    }
}

static inline int
ConsumeCellularRatioUpdate(uint64_t RatioUpdateSeq) {
    if (&g_CellularRatioUpdateSeq == NULL) {
        return 1;
    }
    if (RatioUpdateSeq == 0 || RatioUpdateSeq == g_LastProcessedRatioUpdateSeq) {
        return 0;
    }
    g_LastProcessedRatioUpdateSeq = RatioUpdateSeq;
    return 1;
}

//
// 获取当前 pacing rate
//
static inline uint64_t
GetRatioPacingRate(void) {
    return (uint64_t)g_RatioPacingRate;
}

static inline uint64_t
GetCellularOuterLoopReductionInterval(uint64_t MinRtt) {
    uint64_t Interval = MinRtt;
    if (Interval == 0 || Interval == UINT64_MAX) {
        Interval = CELLULAR_OUTER_LOOP_MAX_REDUCTION_INTERVAL_US;
    }
    if (Interval < CELLULAR_OUTER_LOOP_MIN_REDUCTION_INTERVAL_US) {
        Interval = CELLULAR_OUTER_LOOP_MIN_REDUCTION_INTERVAL_US;
    }
    if (Interval > CELLULAR_OUTER_LOOP_MAX_REDUCTION_INTERVAL_US) {
        Interval = CELLULAR_OUTER_LOOP_MAX_REDUCTION_INTERVAL_US;
    }
    return Interval;
}

static BOOLEAN
ApplyCellularOuterLoopBackoff(
    _In_ QUIC_CONGESTION_CONTROL_BBR* Bbr,
    _In_ uint64_t TimeNow,
    _In_ uint64_t EstimatedThroughputBps,
    _In_ uint64_t MinRtt
    )
{
    if (!IsCellularOuterLoopActive(Bbr) || EstimatedThroughputBps == 0) {
        return FALSE;
    }

    uint64_t ReductionInterval =
        GetCellularOuterLoopReductionInterval(MinRtt);
    if (Bbr->CellularOuterLoopLastBackoffTimeValid &&
        CxPlatTimeDiff64(
            Bbr->CellularOuterLoopLastBackoffTime,
            TimeNow) < ReductionInterval) {
        return FALSE;
    }

    Bbr->CellularOuterLoopLastBackoffTimeValid = TRUE;
    Bbr->CellularOuterLoopLastBackoffTime = TimeNow;

    double TargetRate =
        ((double)EstimatedThroughputBps * CELLULAR_OUTER_LOOP_BACKOFF_FACTOR /
         RATIO_BW_UNIT) -
        CELLULAR_OUTER_LOOP_BACKOFF_MARGIN_BYTES_PER_SEC;
    if (g_RatioPacingRate > 0.0) {
        double MaxReductionRate =
            g_RatioPacingRate * CELLULAR_OUTER_LOOP_MAX_REDUCTION_PER_BACKOFF;
        if (TargetRate < MaxReductionRate) {
            TargetRate = MaxReductionRate;
        }
    }
    if (TargetRate < CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC) {
        TargetRate = CELLULAR_MIN_PACING_RATE_BYTES_PER_SEC;
    }

    if (g_RatioPacingRate <= 0.0 || TargetRate < g_RatioPacingRate) {
        g_RatioPacingRate = TargetRate;
        return TRUE;
    }

    return FALSE;
}

static inline uint64_t
GetRatioBaseRtt(uint64_t MinRtt) {
    if (MinRtt == 0 || MinRtt == UINT64_MAX) {
        return g_RatioBaseRttUs;
    }
    if (g_RatioBaseRttUs == 0 || MinRtt < g_RatioBaseRttUs) {
        g_RatioBaseRttUs = MinRtt;
    }
    return g_RatioBaseRttUs;
}

//
// 计算 RatioCwnd = pacing_rate × base_RTT × 2
//
static inline uint32_t
GetRatioCwnd(uint64_t MinRtt) {
    uint64_t BaseRtt = GetRatioBaseRtt(MinRtt);
    if (g_RatioPacingRate <= 0.0 || BaseRtt == 0) {
        return 0;
    }

    // 使用不随 BBR MinRtt 过期膨胀的 base RTT，避免队列 RTT 反过来放大 cwnd。
    const uint64_t MaxRttForCwnd = 500000;
    uint64_t EffectiveRtt = BaseRtt < MaxRttForCwnd ? BaseRtt : MaxRttForCwnd;

    uint64_t cwnd = (uint64_t)(g_RatioPacingRate * EffectiveRtt / RATIO_MICROSECS_IN_SEC * 2);

    if (cwnd > UINT32_MAX) cwnd = UINT32_MAX;
    return (uint32_t)cwnd;
}

//
// BBR 日志文件
//
static int g_BbrLogFileInitialized = 0;

static const char*
GetBbrLogFilePath(void) {
    static char Path[512];
    static int PathInitialized = 0;
    if (!PathInitialized) {
        const char* EnvPath = getenv("MSQUIC_BBR_LOG");
        if (EnvPath != NULL && EnvPath[0] != '\0') {
            snprintf(Path, sizeof(Path), "%s", EnvPath);
        } else {
            const char* Home = getenv("HOME");
            if (Home != NULL && Home[0] != '\0') {
                snprintf(Path, sizeof(Path), "%s/msquic_cellular/artifacts/bbr_logs/bbr_log.txt", Home);
            } else {
                snprintf(Path, sizeof(Path), "/home/qwu26/msquic_cellular/artifacts/bbr_logs/bbr_log.txt");
            }
        }
        PathInitialized = 1;
    }
    return Path;
}

static FILE*
OpenBbrLogFile(void) {
    const char* path = GetBbrLogFilePath();
    FILE* f;
    if (!g_BbrLogFileInitialized) {
        f = fopen(path, "w");
        g_BbrLogFileInitialized = 1;
    } else {
        f = fopen(path, "a");
    }
    return f;
}

typedef enum BBR_STATE {

    BBR_STATE_STARTUP,

    BBR_STATE_DRAIN,

    BBR_STATE_PROBE_BW,

    BBR_STATE_PROBE_RTT

} BBR_STATE;

typedef enum RECOVERY_STATE {

    RECOVERY_STATE_NOT_RECOVERY = 0,

    RECOVERY_STATE_CONSERVATIVE = 1,

    RECOVERY_STATE_GROWTH = 2,

} RECOVERY_STATE;

//
// Forward declarations
//
_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlLogPacketSent(
    _In_ const QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint32_t PacketSize
    );

_IRQL_requires_max_(PASSIVE_LEVEL)
void
BbrCongestionControlGeneratePerformanceSummary(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    );

//
// Bandwidth is measured as (bytes / BW_UNIT) per second
//
#define BW_UNIT 8 // 1 << 3

//
// Gain is measured as (1 / GAIN_UNIT)
//
#define GAIN_UNIT 256 // 1 << 8

//
// The length of the gain cycle
//
#define GAIN_CYCLE_LENGTH 8

const uint64_t kQuantaFactor = 3;

const uint32_t kMinCwndInMss = 4;

const uint32_t kDefaultRecoveryCwndInMss = 2000;

const uint64_t kMicroSecsInSec = 1000000;

const uint64_t kMilliSecsInSec = 1000;

const uint64_t kLowPacingRateThresholdBytesPerSecond = 1200ULL * 1000;

const uint64_t kHighPacingRateThresholdBytesPerSecond = 24ULL * 1000 * 1000;

const uint32_t kHighGain = GAIN_UNIT * 2885 / 1000 + 1; // 2/ln(2)

const uint32_t kDrainGain = GAIN_UNIT * 1000 / 2885; // 1/kHighGain

//
// Cwnd gain during ProbeBw
//
const uint32_t kCwndGain = GAIN_UNIT * 2;

//
// The expected of bandwidth growth in each round trip time during STARTUP
//
const uint32_t kStartupGrowthTarget = GAIN_UNIT * 5 / 4;

//
// How many rounds of rtt to stay in STARTUP when the bandwidth isn't growing as
// fast as kStartupGrowthTarget
//
const uint8_t kStartupSlowGrowRoundLimit = 3;

//
// The cycle of gains used during the PROBE_BW stage
//
const uint32_t kPacingGain[GAIN_CYCLE_LENGTH] = {
    GAIN_UNIT * 5 / 4,
    GAIN_UNIT * 3 / 4,
    GAIN_UNIT, GAIN_UNIT, GAIN_UNIT,
    GAIN_UNIT, GAIN_UNIT, GAIN_UNIT
};

//
// During ProbeRtt, we need to stay in low inflight condition for at least kProbeRttTimeInUs
//
const uint32_t kProbeRttTimeInUs = 200 * 1000;

//
// Time until a MinRtt measurement is expired.
//
const uint32_t kBbrMinRttExpirationInMicroSecs = S_TO_US(10);

const uint32_t kBbrMaxBandwidthFilterLen = 10;

const uint32_t kBbrMaxAckHeightFilterLen = 10;

#ifdef QUIC_ENHANCED_PACKET_LOGGING
//
// Global BBR packet level logger
//
static BBR_PACKET_LOGGER g_BbrPacketLogger = { 0 };
static BOOLEAN g_BbrPacketLoggerInitialized = FALSE;
#endif

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrBandwidthFilterOnPacketAcked(
    _In_ BBR_BANDWIDTH_FILTER* b,
    _In_ const QUIC_ACK_EVENT* AckEvent,
    _In_ uint64_t RttCounter
    )
{
    if (b->AppLimited && b->AppLimitedExitTarget < AckEvent->LargestAck) {
        b->AppLimited = FALSE;
    }

    uint64_t TimeNow = AckEvent->TimeNow;

    QUIC_SENT_PACKET_METADATA* AckedPacketsIterator = AckEvent->AckedPackets;
    while (AckedPacketsIterator != NULL) {
        QUIC_SENT_PACKET_METADATA* AckedPacket = AckedPacketsIterator;
        AckedPacketsIterator = AckedPacketsIterator->Next;

        if (AckedPacket->PacketLength == 0) {
            continue;
        }

        uint64_t SendRate = UINT64_MAX;
        uint64_t AckRate = UINT64_MAX;
        uint64_t SendElapsed = 0;  // Declare outside of if block for later use
        uint64_t AckElapsed = 0;   // Declare outside of if block for later use

        if (AckedPacket->Flags.HasLastAckedPacketInfo) {
            CXPLAT_DBG_ASSERT(AckedPacket->TotalBytesSent >= AckedPacket->LastAckedPacketInfo.TotalBytesSent);
            CXPLAT_DBG_ASSERT(CxPlatTimeAtOrBefore64(AckedPacket->LastAckedPacketInfo.SentTime, AckedPacket->SentTime));

            SendElapsed = CxPlatTimeDiff64(AckedPacket->LastAckedPacketInfo.SentTime, AckedPacket->SentTime);

            if (SendElapsed) {
                SendRate = (kMicroSecsInSec * BW_UNIT *
                    (AckedPacket->TotalBytesSent - AckedPacket->LastAckedPacketInfo.TotalBytesSent) /
                    SendElapsed);
            }

            if (!CxPlatTimeAtOrBefore64(AckEvent->AdjustedAckTime, AckedPacket->LastAckedPacketInfo.AdjustedAckTime)) {
                AckElapsed = CxPlatTimeDiff64(AckedPacket->LastAckedPacketInfo.AdjustedAckTime, AckEvent->AdjustedAckTime);
            } else {
                AckElapsed = CxPlatTimeDiff64(AckedPacket->LastAckedPacketInfo.AckTime, TimeNow);
            }

            CXPLAT_DBG_ASSERT(AckEvent->NumTotalAckedRetransmittableBytes >= AckedPacket->LastAckedPacketInfo.TotalBytesAcked);
            if (AckElapsed) {
                AckRate = (kMicroSecsInSec * BW_UNIT *
                           (AckEvent->NumTotalAckedRetransmittableBytes - AckedPacket->LastAckedPacketInfo.TotalBytesAcked) /
                           AckElapsed);
            }
        } else if (!CxPlatTimeAtOrBefore64(TimeNow, AckedPacket->SentTime)) {
            CXPLAT_DBG_ASSERT(CxPlatTimeDiff64(AckedPacket->SentTime, TimeNow) != 0);
            SendRate = (kMicroSecsInSec * BW_UNIT *
                        AckEvent->NumTotalAckedRetransmittableBytes /
                        CxPlatTimeDiff64(AckedPacket->SentTime, TimeNow));
        }

        if (SendRate == UINT64_MAX && AckRate == UINT64_MAX) {
            continue;
        }

        uint64_t DeliveryRate = CXPLAT_MIN(SendRate, AckRate);

        QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY Entry = (QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY) { .Value = 0, .Time = 0 };
        QUIC_STATUS Status = QuicSlidingWindowExtremumGet(&b->WindowedMaxFilter, &Entry);

        uint64_t PreviousMaxDeliveryRate = 0;
        if (QUIC_SUCCEEDED(Status)) {
            PreviousMaxDeliveryRate = Entry.Value;
        }

        if (DeliveryRate >= PreviousMaxDeliveryRate || !AckedPacket->Flags.IsAppLimited) {
            QuicSlidingWindowExtremumUpdateMax(&b->WindowedMaxFilter, DeliveryRate, RttCounter);
        }

        // TODO: Add delay tracking here later when CLOG issues are resolved
    }
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint64_t
BbrCongestionControlGetBandwidth(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY Entry = (QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY) { .Value = 0, .Time = 0 };
    QUIC_STATUS Status = QuicSlidingWindowExtremumGet(&Cc->Bbr.BandwidthFilter.WindowedMaxFilter, &Entry);
    if (QUIC_SUCCEEDED(Status)) {
        return Entry.Value;
    }
    return 0;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlInRecovery(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
)
{
    return Cc->Bbr.RecoveryState != RECOVERY_STATE_NOT_RECOVERY;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint32_t
BbrCongestionControlGetCongestionWindow(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    const QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    uint32_t MinCongestionWindow = kMinCwndInMss * DatagramPayloadLength;

    //
    // cellular:1 uses the cellular pacing rate for cwnd. The rate is normally
    // driven by GBR ratio and can be clamped by the outer delay backoff.
    //
    if (IsCellularControlActive()) {
        uint32_t ratioCwnd = GetRatioCwnd(Bbr->MinRtt);
        if (ratioCwnd > 0) {
            // 确保不低于最小 cwnd
            return CXPLAT_MAX(ratioCwnd, MinCongestionWindow);
        }
        // ratio cwnd 未初始化时，返回当前 cwnd
        return Bbr->CongestionWindow;
    }

    // 非 cellular 模式下，使用原始 BBR 逻辑
    if (Bbr->BbrState == BBR_STATE_PROBE_RTT) {
        return MinCongestionWindow;
    }

    if (BbrCongestionControlInRecovery(Cc)) {
        return CXPLAT_MIN(Bbr->CongestionWindow, Bbr->RecoveryWindow);
    }

    return Bbr->CongestionWindow;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlTransitToProbeBw(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint64_t CongestionEventTime
    )
{
    QUIC_CONGESTION_CONTROL_BBR *Bbr = &Cc->Bbr;

    Bbr->BbrState = BBR_STATE_PROBE_BW;
    Bbr->CwndGain = kCwndGain;

    uint32_t RandomValue = 0;
    CxPlatRandom(sizeof(uint32_t), &RandomValue);
    Bbr->PacingCycleIndex = (RandomValue % (GAIN_CYCLE_LENGTH - 1) + 2) % GAIN_CYCLE_LENGTH;
    CXPLAT_DBG_ASSERT(Bbr->PacingCycleIndex != 1);
    Bbr->PacingGain = kPacingGain[Bbr->PacingCycleIndex];

    Bbr->CycleStart = CongestionEventTime;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlTransitToStartup(
    _In_ QUIC_CONGESTION_CONTROL* Cc
    )
{
    Cc->Bbr.BbrState = BBR_STATE_STARTUP;
    Cc->Bbr.PacingGain = kHighGain;
    Cc->Bbr.CwndGain = kHighGain;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlIsAppLimited(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    return Cc->Bbr.BandwidthFilter.AppLimited;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
QuicConnLogBbr(
    _In_ QUIC_CONNECTION* const Connection
    )
{
    QUIC_CONGESTION_CONTROL* Cc = &Connection->CongestionControl;
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    QuicTraceEvent(
        ConnBbr,
        "[conn][%p] BBR: State=%u RState=%u CongestionWindow=%u BytesInFlight=%u BytesInFlightMax=%u MinRttEst=%lu EstBw=%lu AppLimited=%u",
        Connection,
        Bbr->BbrState,
        Bbr->RecoveryState,
        BbrCongestionControlGetCongestionWindow(Cc),
        Bbr->BytesInFlight,
        Bbr->BytesInFlightMax,
        Bbr->MinRtt,
        BbrCongestionControlGetBandwidth(Cc) / BW_UNIT,
        BbrCongestionControlIsAppLimited(Cc));
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlIndicateConnectionEvent(
    _In_ QUIC_CONNECTION* const Connection,
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    const QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    const QUIC_PATH* Path = &Connection->Paths[0];
    QUIC_CONNECTION_EVENT Event;
    Event.Type = QUIC_CONNECTION_EVENT_NETWORK_STATISTICS;
    Event.NETWORK_STATISTICS.BytesInFlight = Bbr->BytesInFlight;
    Event.NETWORK_STATISTICS.PostedBytes = Connection->SendBuffer.PostedBytes;
    Event.NETWORK_STATISTICS.IdealBytes = Connection->SendBuffer.IdealBytes;
    Event.NETWORK_STATISTICS.SmoothedRTT = Path->SmoothedRtt;
    Event.NETWORK_STATISTICS.CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
    Event.NETWORK_STATISTICS.Bandwidth = BbrCongestionControlGetBandwidth(Cc) / BW_UNIT;

    QuicTraceLogConnVerbose(
        IndicateDataAcked,
        Connection,
        "Indicating QUIC_CONNECTION_EVENT_NETWORK_STATISTICS [BytesInFlight=%u,PostedBytes=%llu,IdealBytes=%llu,SmoothedRTT=%llu,CongestionWindow=%u,Bandwidth=%llu]",
        Event.NETWORK_STATISTICS.BytesInFlight,
        Event.NETWORK_STATISTICS.PostedBytes,
        Event.NETWORK_STATISTICS.IdealBytes,
        Event.NETWORK_STATISTICS.SmoothedRTT,
        Event.NETWORK_STATISTICS.CongestionWindow,
        Event.NETWORK_STATISTICS.Bandwidth);
    QuicConnIndicateEvent(Connection, &Event);
}

_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlCanSend(
    _In_ QUIC_CONGESTION_CONTROL* Cc
    )
{
    uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
    return Cc->Bbr.BytesInFlight < CongestionWindow || Cc->Bbr.Exemptions > 0;
}

void
BbrCongestionControlLogOutFlowStatus(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    const QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    const QUIC_PATH* Path = &Connection->Paths[0];
    const QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    QuicTraceEvent(
        ConnOutFlowStatsV2,
        "[conn][%p] OUT: BytesSent=%llu InFlight=%u CWnd=%u ConnFC=%llu ISB=%llu PostedBytes=%llu SRtt=%llu 1Way=%llu",
        Connection,
        Connection->Stats.Send.TotalBytes,
        Bbr->BytesInFlight,
        Bbr->CongestionWindow,
        Connection->Send.PeerMaxData - Connection->Send.OrderedStreamBytesSent,
        Connection->SendBuffer.IdealBytes,
        Connection->SendBuffer.PostedBytes,
        Path->GotFirstRttSample ? Path->SmoothedRtt : 0,
        Path->OneWayDelay);
}

//
// Returns TRUE if we became unblocked.
//
_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlUpdateBlockedState(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ BOOLEAN PreviousCanSendState
    )
{
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    QuicConnLogOutFlowStats(Connection);

    if (PreviousCanSendState != BbrCongestionControlCanSend(Cc)) {
        if (PreviousCanSendState) {
            QuicConnAddOutFlowBlockedReason(
                Connection, QUIC_FLOW_BLOCKED_CONGESTION_CONTROL);
        } else {
            QuicConnRemoveOutFlowBlockedReason(
                Connection, QUIC_FLOW_BLOCKED_CONGESTION_CONTROL);
            Connection->Send.LastFlushTime = CxPlatTimeUs64(); // Reset last flush time
            return TRUE;
        }
    }
    return FALSE;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint32_t
BbrCongestionControlGetBytesInFlightMax(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    return Cc->Bbr.BytesInFlightMax;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint8_t
BbrCongestionControlGetExemptions(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    return Cc->Bbr.Exemptions;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlSetExemption(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint8_t NumPackets
    )
{
    Cc->Bbr.Exemptions = NumPackets;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlOnDataSent(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint32_t NumRetransmittableBytes
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    BOOLEAN PreviousCanSendState = BbrCongestionControlCanSend(Cc);

    if (!Bbr->BytesInFlight && BbrCongestionControlIsAppLimited(Cc)) {
        Bbr->ExitingQuiescence = TRUE;
    }



    Bbr->BytesInFlight += NumRetransmittableBytes;
    if (Bbr->BytesInFlightMax < Bbr->BytesInFlight) {
        Bbr->BytesInFlightMax = Bbr->BytesInFlight;
        QuicSendBufferConnectionAdjust(QuicCongestionControlGetConnection(Cc));
    }

    if (Bbr->Exemptions > 0) {
        --Bbr->Exemptions;
    }

    // Log BBR state for each packet transmission
     BbrCongestionControlLogPacketSent(Cc, NumRetransmittableBytes);

#ifdef QUIC_ENHANCED_PACKET_LOGGING
    // Enhanced packet level logging
    if (g_BbrPacketLoggerInitialized) {
        QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
        uint64_t PacketNumber = Connection->LossDetection.LargestSentPacketNumber + 1;
        BbrPacketLevelLoggingRecordPacketSent(&g_BbrPacketLogger, Cc, PacketNumber, NumRetransmittableBytes);
    }
#endif

    BbrCongestionControlUpdateBlockedState(Cc, PreviousCanSendState);
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlLogPacketSent(
    _In_ const QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint32_t PacketSize
    )
{
    const QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    const QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    const QUIC_PATH* Path = &Connection->Paths[0];

    uint64_t EstimatedBandwidth = BbrCongestionControlGetBandwidth(Cc);
    uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
    uint64_t SmoothedRtt = Path->GotFirstRttSample ? Path->SmoothedRtt : 0;
    uint64_t MinRtt = Bbr->MinRtt != UINT64_MAX ? Bbr->MinRtt : 0;
    uint32_t BytesInFlight = Bbr->BytesInFlight;
    uint64_t TotalPacketsSent = Connection->Stats.Send.TotalPackets;
    uint64_t TotalPacketsLost = Connection->Stats.Send.SuspectedLostPackets;
    double LossRate = TotalPacketsSent > 0 ? ((double)TotalPacketsLost * 100.0) / (double)TotalPacketsSent : 0.0;

    // Calculate connection duration
    uint64_t CurrentTime = CxPlatTimeUs64();
    uint64_t ConnectionDuration = CurrentTime - Connection->Stats.Timing.Start;

    // Test mode logging
    if (IsTestModeEnabled()) {
        uint64_t testRate = TestGetCurrentRateBytes();
        double elapsedSec = TestGetElapsedSec();
        double ratio = GetCellularRatio();
        double actualRateMbps = (ConnectionDuration > 0) ?
            (double)Connection->Stats.Send.TotalBytes * 8.0 / (double)ConnectionDuration : 0.0;

        FILE* testLog;
        if (!g_TestLogInitialized) {
            testLog = fopen(TEST_LOG_PATH, "w");
            if (testLog) {
                fprintf(testLog, "# Test Mode: Linear rate ramp 0 -> 50 Mbps over 60s\n");
                fprintf(testLog, "# T(s), TargetRate(Mbps), ActualRate(Mbps), RTT(ms), Ratio, CWND(B), InFlight(B), PKT, Size(B)\n");
                g_TestLogInitialized = 1;
            }
        } else {
            testLog = fopen(TEST_LOG_PATH, "a");
        }
        if (testLog) {
            fprintf(testLog, "%.3f, %.2f, %.2f, %.1f, %.3f, %u, %u, %lu, %u\n",
                elapsedSec,
                (double)testRate * 8.0 / 1000000.0,
                actualRateMbps,
                (double)SmoothedRtt / 1000.0,
                ratio,
                CongestionWindow,
                BytesInFlight,
                (unsigned long)TotalPacketsSent,
                PacketSize);
            fclose(testLog);
        }
        return;  // Skip normal BBR logging in test mode
    }

    // Calculate pacing rate and delivery rate
    uint64_t PacingRate = EstimatedBandwidth * Bbr->PacingGain / GAIN_UNIT;
    uint64_t DeliveryRate = Bbr->RecentDeliveryRate;

    // Write detailed BBR packet log to the same log file as periodic logs
    FILE* logFile = fopen("/home/qwu26/msquic_cellular/artifacts/bbr_logs/bbr_log.txt", "a");
    if (logFile != NULL) {
        double trendlineSlope = TrendlineEstimatorGetSlope(&Bbr->TrendlineEstimator);
        fprintf(logFile, "[BBR-PKT-SENT] T=%lu.%03lu s, PKT=%lu, Size=%u B, "
               "EstBW=%.2f Mbps, PacingRate=%.2f Mbps, DeliveryRate=%.2f Mbps, "
               "RTT=%lu us, MinRTT=%lu us, CWND=%u B, InFlight=%u B, "
               "Loss=%.2f%%, State=%s, TotalSent=%lu, TotalLost=%lu, "
               "SendDelay=%lu us, AckDelay=%lu us, PacingGain=%.2fx, CwndGain=%.2fx, "
               "TrendlineSlope=%.6f\n",
               (unsigned long)(ConnectionDuration / 1000000),
               (unsigned long)((ConnectionDuration % 1000000) / 1000),
               (unsigned long)TotalPacketsSent,
               PacketSize,
               EstimatedBandwidth / 1000000.0,
               PacingRate / 1000000.0,
               DeliveryRate / 1000000.0,
               (unsigned long)SmoothedRtt,
               (unsigned long)MinRtt,
               CongestionWindow,
               BytesInFlight,
               LossRate,
               Bbr->BbrState == 0 ? "STARTUP" :
               Bbr->BbrState == 1 ? "DRAIN" :
               Bbr->BbrState == 2 ? "PROBE_BW" :
               Bbr->BbrState == 3 ? "PROBE_RTT" : "UNKNOWN",
               (unsigned long)TotalPacketsSent,
               (unsigned long)TotalPacketsLost,
               (unsigned long)Bbr->RecentSendDelay,
               (unsigned long)Bbr->RecentAckDelay,
               (double)Bbr->PacingGain / (double)GAIN_UNIT,
               (double)Bbr->CwndGain / (double)GAIN_UNIT,
               trendlineSlope);
        fclose(logFile);
    }
}

_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlOnDataInvalidated(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint32_t NumRetransmittableBytes
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    BOOLEAN PreviousCanSendState = BbrCongestionControlCanSend(Cc);

    CXPLAT_DBG_ASSERT(Bbr->BytesInFlight >= NumRetransmittableBytes);
    Bbr->BytesInFlight -= NumRetransmittableBytes;

    return BbrCongestionControlUpdateBlockedState(Cc, PreviousCanSendState);
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlUpdateRecoveryWindow(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint32_t BytesAcked
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    CXPLAT_DBG_ASSERT(Bbr->RecoveryState != RECOVERY_STATE_NOT_RECOVERY);

    if (Bbr->RecoveryState == RECOVERY_STATE_GROWTH) {
        Bbr->RecoveryWindow += BytesAcked;
    }

    uint32_t RecoveryWindow = CXPLAT_MAX(
        Bbr->RecoveryWindow, Bbr->BytesInFlight + BytesAcked);

    uint32_t MinCongestionWindow = kMinCwndInMss * DatagramPayloadLength;

    Bbr->RecoveryWindow = CXPLAT_MAX(RecoveryWindow, MinCongestionWindow);
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlHandleAckInProbeRtt(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ BOOLEAN NewRoundTrip,
    _In_ uint64_t LargestSentPacketNumber,
    _In_ uint64_t AckTime
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    Bbr->BandwidthFilter.AppLimited = TRUE;
    Bbr->BandwidthFilter.AppLimitedExitTarget = LargestSentPacketNumber;

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    if (!Bbr->ProbeRttEndTimeValid &&
        Bbr->BytesInFlight < BbrCongestionControlGetCongestionWindow(Cc) + DatagramPayloadLength) {

        Bbr->ProbeRttEndTime = AckTime + kProbeRttTimeInUs;
        Bbr->ProbeRttEndTimeValid = TRUE;

        Bbr->ProbeRttRoundValid = FALSE;

        return;
    }

    if (Bbr->ProbeRttEndTimeValid) {

        if (!Bbr->ProbeRttRoundValid && NewRoundTrip) {
            Bbr->ProbeRttRoundValid = TRUE;
            Bbr->ProbeRttRound = Bbr->RoundTripCounter;
        }

        if (Bbr->ProbeRttRoundValid && CxPlatTimeAtOrBefore64(Bbr->ProbeRttEndTime, AckTime)) {
            Bbr->MinRttTimestamp = AckTime;
            Bbr->MinRttTimestampValid = TRUE;

            if (Bbr->BtlbwFound) {
                BbrCongestionControlTransitToProbeBw(Cc, AckTime);
            } else {
                BbrCongestionControlTransitToStartup(Cc);
            }
        }

    }
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint64_t
BbrCongestionControlUpdateAckAggregation(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ const QUIC_ACK_EVENT* AckEvent
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    if (!Bbr->AckAggregationStartTimeValid) {
        Bbr->AckAggregationStartTime = AckEvent->TimeNow;
        Bbr->AckAggregationStartTimeValid = TRUE;
        return 0;
    }

    uint64_t ExpectedAckBytes = BbrCongestionControlGetBandwidth(Cc) *
                                CxPlatTimeDiff64(Bbr->AckAggregationStartTime, AckEvent->TimeNow) /
                                kMicroSecsInSec /
                                BW_UNIT;

    //
    // Reset current ack aggregation status when we witness ack arrival rate being less or equal than
    // estimated bandwidth
    //
    if (Bbr->AggregatedAckBytes <= ExpectedAckBytes) {
        Bbr->AggregatedAckBytes = AckEvent->NumRetransmittableBytes;
        Bbr->AckAggregationStartTime = AckEvent->TimeNow;
        Bbr->AckAggregationStartTimeValid = TRUE;

        return 0;
    }

    Bbr->AggregatedAckBytes += AckEvent->NumRetransmittableBytes;

    QuicSlidingWindowExtremumUpdateMax(&Bbr->MaxAckHeightFilter,
        Bbr->AggregatedAckBytes - ExpectedAckBytes, Bbr->RoundTripCounter);

    return Bbr->AggregatedAckBytes - ExpectedAckBytes;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint32_t
BbrCongestionControlGetTargetCwnd(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint32_t Gain
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    uint64_t BandwidthEst = BbrCongestionControlGetBandwidth(Cc);

    if (!BandwidthEst || Bbr->MinRtt == UINT32_MAX) {
        return (uint64_t)(Gain) * Bbr->InitialCongestionWindow / GAIN_UNIT;
    }

    uint64_t Bdp = BandwidthEst * Bbr->MinRtt / kMicroSecsInSec / BW_UNIT;
    uint64_t TargetCwnd = (Bdp * Gain / GAIN_UNIT) + (kQuantaFactor * Bbr->SendQuantum);
    return (uint32_t)TargetCwnd;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
uint32_t
BbrCongestionControlGetSendAllowance(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint64_t TimeSinceLastSend, // microsec
    _In_ BOOLEAN TimeSinceLastSendValid
    )
{
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    uint64_t BandwidthEst = BbrCongestionControlGetBandwidth(Cc);
    uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);

    uint32_t SendAllowance = 0;

    //
    // Test mode: linear rate ramp from 0 to 50 Mbps
    //
    if (IsTestModeEnabled()) {
        uint64_t testRate = TestGetCurrentRateBytes();  // bytes/sec

        if (Bbr->BytesInFlight >= CongestionWindow) {
            SendAllowance = 0;
        } else if (!TimeSinceLastSendValid || !Connection->Settings.PacingEnabled) {
            SendAllowance = CongestionWindow - Bbr->BytesInFlight;
        } else {
            // SendAllowance = rate * time
            SendAllowance = (uint32_t)(testRate * TimeSinceLastSend / 1000000);

            uint32_t AvailableWindow = CongestionWindow - Bbr->BytesInFlight;
            if (SendAllowance > AvailableWindow) {
                SendAllowance = AvailableWindow;
            }
        }
        return SendAllowance;
    }

    //
    // cellular:1 uses the cellular pacing rate for send allowance. The rate is
    // normally driven by GBR ratio and can be clamped by the outer delay backoff.
    //
    if (IsCellularControlActive()) {
        uint64_t ratioPacingRate = GetRatioPacingRate();

        if (Bbr->BytesInFlight >= CongestionWindow) {
            SendAllowance = 0;
        } else if (!TimeSinceLastSendValid || !Connection->Settings.PacingEnabled) {
            SendAllowance = CongestionWindow - Bbr->BytesInFlight;
        } else {
            // 使用 ratioPacingRate 计算发送配额
            // SendAllowance = ratioPacingRate * TimeSinceLastSend (bytes)
            SendAllowance = (uint32_t)(ratioPacingRate * TimeSinceLastSend / RATIO_MICROSECS_IN_SEC);

            if (SendAllowance > CongestionWindow - Bbr->BytesInFlight) {
                SendAllowance = CongestionWindow - Bbr->BytesInFlight;
            }

            if (SendAllowance > (CongestionWindow >> 2)) {
                SendAllowance = CongestionWindow >> 2;
            }
        }
        return SendAllowance;
    }

    // 非 cellular 模式，使用原始 BBR 逻辑
    if (Bbr->BytesInFlight >= CongestionWindow) {
        //
        // We are CC blocked, so we can't send anything.
        //
        SendAllowance = 0;

    } else if (
        !TimeSinceLastSendValid ||
        !Connection->Settings.PacingEnabled ||
        Bbr->MinRtt == UINT32_MAX ||
        Bbr->MinRtt < QUIC_SEND_PACING_INTERVAL) {
        //
        // We're not in the necessary state to pace.
        //
        SendAllowance = CongestionWindow - Bbr->BytesInFlight;

    } else {
        //
        // We are pacing, so split the congestion window into chunks which are
        // spread out over the RTT. Calculate the current send allowance (chunk
        // size) as the time since the last send times the pacing rate (CWND / RTT).
        //
        if (Bbr->BbrState == BBR_STATE_STARTUP) {
            SendAllowance = (uint32_t)CXPLAT_MAX(
                BandwidthEst * Bbr->PacingGain * TimeSinceLastSend / GAIN_UNIT,
                CongestionWindow * Bbr->PacingGain / GAIN_UNIT - Bbr->BytesInFlight);
        } else {
            SendAllowance = (uint32_t)(BandwidthEst * Bbr->PacingGain * TimeSinceLastSend / GAIN_UNIT);
        }

        if (SendAllowance > CongestionWindow - Bbr->BytesInFlight) {
            SendAllowance = CongestionWindow - Bbr->BytesInFlight;
        }

        if (SendAllowance > (CongestionWindow >> 2)) {
            SendAllowance = CongestionWindow >> 2; // Don't send more than a quarter of the current window.
        }
    }
    return SendAllowance;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlTransitToProbeRtt(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint64_t LargestSentPacketNumber
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    Bbr->BbrState = BBR_STATE_PROBE_RTT;
    Bbr->PacingGain = GAIN_UNIT;
    Bbr->ProbeRttEndTimeValid = FALSE;
    Bbr->ProbeRttRoundValid = FALSE;

    Bbr->BandwidthFilter.AppLimited = TRUE;
    Bbr->BandwidthFilter.AppLimitedExitTarget = LargestSentPacketNumber;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlTransitToDrain(
    _In_ QUIC_CONGESTION_CONTROL* Cc
    )
{
    Cc->Bbr.BbrState = BBR_STATE_DRAIN;
    Cc->Bbr.PacingGain = kDrainGain;
    Cc->Bbr.CwndGain = kHighGain;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlSetSendQuantum(
    _In_ QUIC_CONGESTION_CONTROL* Cc
)
{
    QUIC_CONGESTION_CONTROL_BBR *Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    uint64_t Bandwidth = BbrCongestionControlGetBandwidth(Cc);

    uint64_t PacingRate = Bandwidth * Bbr->PacingGain / GAIN_UNIT;

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    if (PacingRate < kLowPacingRateThresholdBytesPerSecond * BW_UNIT) {
        Bbr->SendQuantum = (uint64_t)DatagramPayloadLength;
    } else if (PacingRate < kHighPacingRateThresholdBytesPerSecond * BW_UNIT) {
        Bbr->SendQuantum = (uint64_t)DatagramPayloadLength * 2;
    } else {
        Bbr->SendQuantum = CXPLAT_MIN(PacingRate * kMilliSecsInSec / BW_UNIT, 64 * 1024 /* 64k */);
    }
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlUpdateCongestionWindow(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ uint64_t TotalBytesAcked,
    _In_ uint64_t AckedBytes
    )
{
    QUIC_CONGESTION_CONTROL_BBR *Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    if (Bbr->BbrState == BBR_STATE_PROBE_RTT) {
        return;
    }

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    BbrCongestionControlSetSendQuantum(Cc);

    uint64_t TargetCwnd = BbrCongestionControlGetTargetCwnd(Cc, Bbr->CwndGain);
    if (Bbr->BtlbwFound) {
        QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY Entry = (QUIC_SLIDING_WINDOW_EXTREMUM_ENTRY) { .Value = 0, .Time = 0 };
        QUIC_STATUS Status = QuicSlidingWindowExtremumGet(&Bbr->MaxAckHeightFilter, &Entry);
        if (QUIC_SUCCEEDED(Status)) {
            TargetCwnd += Entry.Value;
        }
    }

    uint32_t CongestionWindow = Bbr->CongestionWindow;
    uint32_t MinCongestionWindow = kMinCwndInMss * DatagramPayloadLength;

    if (Bbr->BtlbwFound) {
        CongestionWindow = (uint32_t)CXPLAT_MIN(TargetCwnd, CongestionWindow + AckedBytes);
    } else if (CongestionWindow < TargetCwnd || TotalBytesAcked < Bbr->InitialCongestionWindow) {
        CongestionWindow += (uint32_t)AckedBytes;
    }

    Bbr->CongestionWindow = CXPLAT_MAX(CongestionWindow, MinCongestionWindow);

    QuicConnLogBbr(QuicCongestionControlGetConnection(Cc));
}

_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlOnDataAcknowledged(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ const QUIC_ACK_EVENT* AckEvent
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    BOOLEAN PreviousCanSendState = BbrCongestionControlCanSend(Cc);
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    if (AckEvent->IsImplicit) {
        BbrCongestionControlUpdateCongestionWindow(
            Cc, AckEvent->NumTotalAckedRetransmittableBytes, AckEvent->NumRetransmittableBytes);

        if (Connection->Settings.NetStatsEventEnabled) {
            BbrCongestionControlIndicateConnectionEvent(Connection, Cc);
        }
        return BbrCongestionControlUpdateBlockedState(Cc, PreviousCanSendState);
    }

    uint32_t PrevInflightBytes = Bbr->BytesInFlight;

    CXPLAT_DBG_ASSERT(Bbr->BytesInFlight >= AckEvent->NumRetransmittableBytes);
    Bbr->BytesInFlight -= AckEvent->NumRetransmittableBytes;

    if (AckEvent->MinRttValid) {
        Bbr->RttSampleExpired = Bbr->MinRttTimestampValid ?
           CxPlatTimeAtOrBefore64(Bbr->MinRttTimestamp + kBbrMinRttExpirationInMicroSecs, AckEvent->TimeNow) :
           FALSE;
        if (Bbr->RttSampleExpired || Bbr->MinRtt > AckEvent->MinRtt) {
            Bbr->MinRtt = AckEvent->MinRtt;
            Bbr->MinRttTimestamp = AckEvent->TimeNow;
            Bbr->MinRttTimestampValid = TRUE;
        }
    }

    BOOLEAN NewRoundTrip = FALSE;
    if (!Bbr->EndOfRoundTripValid || Bbr->EndOfRoundTrip < AckEvent->LargestAck) {
        Bbr->RoundTripCounter++;
        Bbr->EndOfRoundTripValid = TRUE;
        Bbr->EndOfRoundTrip = AckEvent->LargestSentPacketNumber;
        NewRoundTrip = TRUE;
    }

    BOOLEAN LastAckedPacketAppLimited =
        AckEvent->AckedPackets == NULL ? FALSE : AckEvent->IsLargestAckedPacketAppLimited;

    BbrBandwidthFilterOnPacketAcked(&Bbr->BandwidthFilter, AckEvent, Bbr->RoundTripCounter);

    // Update recent delivery rate tracking for logging
    // Calculate delivery rate based on recent ACK events
    QUIC_SENT_PACKET_METADATA* AckedPacketsIterator = AckEvent->AckedPackets;
    while (AckedPacketsIterator != NULL) {
        QUIC_SENT_PACKET_METADATA* AckedPacket = AckedPacketsIterator;
        AckedPacketsIterator = AckedPacketsIterator->Next;

        if (AckedPacket->PacketLength == 0) {
            continue;
        }

        uint64_t SendRate = UINT64_MAX;
        uint64_t AckRate = UINT64_MAX;
        uint64_t TimeNow = AckEvent->TimeNow;

        if (AckedPacket->Flags.HasLastAckedPacketInfo) {
            uint64_t SendElapsed = CxPlatTimeDiff64(AckedPacket->LastAckedPacketInfo.SentTime, AckedPacket->SentTime);
            if (SendElapsed) {
                SendRate = (kMicroSecsInSec * BW_UNIT *
                    (AckedPacket->TotalBytesSent - AckedPacket->LastAckedPacketInfo.TotalBytesSent) /
                    SendElapsed);
                // Record the actual send delay used in delivery rate calculation
                Bbr->RecentSendDelay = SendElapsed;
            }

            uint64_t AckElapsed = 0;
            if (!CxPlatTimeAtOrBefore64(AckEvent->AdjustedAckTime, AckedPacket->LastAckedPacketInfo.AdjustedAckTime)) {
                AckElapsed = CxPlatTimeDiff64(AckedPacket->LastAckedPacketInfo.AdjustedAckTime, AckEvent->AdjustedAckTime);
            } else {
                AckElapsed = CxPlatTimeDiff64(AckedPacket->LastAckedPacketInfo.AckTime, TimeNow);
            }

            if (AckElapsed) {
                AckRate = (kMicroSecsInSec * BW_UNIT *
                           (AckEvent->NumTotalAckedRetransmittableBytes - AckedPacket->LastAckedPacketInfo.TotalBytesAcked) /
                           AckElapsed);
                // Record the actual ack delay used in delivery rate calculation
                Bbr->RecentAckDelay = AckElapsed;
            }
        } else if (!CxPlatTimeAtOrBefore64(TimeNow, AckedPacket->SentTime)) {
            uint64_t RttElapsed = CxPlatTimeDiff64(AckedPacket->SentTime, TimeNow);
            SendRate = (kMicroSecsInSec * BW_UNIT *
                        AckEvent->NumTotalAckedRetransmittableBytes /
                        RttElapsed);
            // Record RTT-based delay when no previous packet info available
            Bbr->RecentSendDelay = RttElapsed;
            Bbr->RecentAckDelay = RttElapsed;
        }

        if (SendRate != UINT64_MAX || AckRate != UINT64_MAX) {
            uint64_t DeliveryRate = CXPLAT_MIN(SendRate, AckRate);
            if (DeliveryRate != UINT64_MAX) {
                Bbr->RecentSendRate = SendRate;
                Bbr->RecentAckRate = AckRate;
                Bbr->RecentDeliveryRate = DeliveryRate;



                break; // Use the first valid delivery rate
            }
        }
    }

    //
    // Update trendline slope estimator (batch-level)
    // Use the last packet's send time in the batch to avoid ACK batching bias
    //
    uint64_t BatchLastSendTimeUs = 0;
    uint32_t BatchTotalBytes = 0;

    AckedPacketsIterator = AckEvent->AckedPackets;
    while (AckedPacketsIterator != NULL) {
        QUIC_SENT_PACKET_METADATA* AckedPacket = AckedPacketsIterator;
        AckedPacketsIterator = AckedPacketsIterator->Next;

        if (AckedPacket->PacketLength == 0) {
            continue;
        }

        // Track the last (most recent) send time in this batch
        if (AckedPacket->SentTime > BatchLastSendTimeUs) {
            BatchLastSendTimeUs = AckedPacket->SentTime;
        }
        BatchTotalBytes += AckedPacket->PacketLength;
    }

    // Update trendline once per ACK batch
    if (BatchLastSendTimeUs > 0) {
        TrendlineEstimatorUpdateBatch(
            &Bbr->TrendlineEstimator,
            BatchLastSendTimeUs,
            AckEvent->AdjustedAckTime,
            BatchTotalBytes);
    }
    UpdateCellularOuterLoopRatioGate(Bbr, AckEvent->TimeNow);
    UpdateCellularOuterLoopOveruseGate(Bbr, AckEvent->TimeNow);

    // Get trendline slope (available for rate adjustment or logging)
    // double trendlineSlope = TrendlineEstimatorGetSlope(&Bbr->TrendlineEstimator);
    // Slope meaning:
    //   slope > 0: delay increasing, queue building up -> reduce rate
    //   slope ≈ 0: delay stable -> maintain rate
    //   slope < 0: delay decreasing, queue draining -> can increase rate

    if (BbrCongestionControlInRecovery(Cc)) {
        CXPLAT_DBG_ASSERT(Bbr->EndOfRecoveryValid);
        if (NewRoundTrip && Bbr->RecoveryState != RECOVERY_STATE_GROWTH) {
            Bbr->RecoveryState = RECOVERY_STATE_GROWTH;
        }
        if (!AckEvent->HasLoss && Bbr->EndOfRecovery < AckEvent->LargestAck) {
            Bbr->RecoveryState = RECOVERY_STATE_NOT_RECOVERY;
            QuicTraceEvent(
                ConnRecoveryExit,
                "[conn][%p] Recovery complete",
                Connection);
        } else {
            BbrCongestionControlUpdateRecoveryWindow(Cc, AckEvent->NumRetransmittableBytes);
        }
    }

    BbrCongestionControlUpdateAckAggregation(Cc, AckEvent);

    //
    // cellular:1 模式下跳过 PROBE_BW pacing cycle 和 STARTUP bandwidth probing
    //
    if (!IsCellularControlActive()) {
        if (Bbr->BbrState == BBR_STATE_PROBE_BW) {
            BOOLEAN ShouldAdvancePacingGainCycle = CxPlatTimeDiff64(AckEvent->TimeNow, Bbr->CycleStart) > Bbr->MinRtt;

            if (Bbr->PacingGain > GAIN_UNIT && !AckEvent->HasLoss &&
                PrevInflightBytes < BbrCongestionControlGetTargetCwnd(Cc, Bbr->PacingGain)) {
                ShouldAdvancePacingGainCycle = FALSE;
            }

            if (Bbr->PacingGain < GAIN_UNIT) {
                uint64_t TargetCwnd = BbrCongestionControlGetTargetCwnd(Cc, GAIN_UNIT);
                if (Bbr->BytesInFlight <= TargetCwnd) {
                    ShouldAdvancePacingGainCycle = TRUE;
                }
            }

            if (ShouldAdvancePacingGainCycle) {
                Bbr->PacingCycleIndex = (Bbr->PacingCycleIndex + 1) % GAIN_CYCLE_LENGTH;
                Bbr->CycleStart = AckEvent->TimeNow;
                Bbr->PacingGain = kPacingGain[Bbr->PacingCycleIndex];
            }
        }

        if (!Bbr->BtlbwFound && NewRoundTrip && !LastAckedPacketAppLimited) {
            uint64_t BandwidthTarget = (uint64_t)(Bbr->LastEstimatedStartupBandwidth * kStartupGrowthTarget / GAIN_UNIT);
            uint64_t CurrentBandwidth = BbrCongestionControlGetBandwidth(Cc);

            if (CurrentBandwidth >= BandwidthTarget) {
                Bbr->LastEstimatedStartupBandwidth = CurrentBandwidth;
                Bbr->SlowStartupRoundCounter = 0;
            } else if (++Bbr->SlowStartupRoundCounter >= kStartupSlowGrowRoundLimit) {
                Bbr->BtlbwFound = TRUE;
            }
        }
    }

    //
    // cellular:1 模式下跳过所有 BBR 状态转换，完全由 ratio/outer backoff 控制
    //
    if (!IsCellularControlActive()) {
        // 非 cellular 模式：正常 BBR 状态机
        if (Bbr->BbrState == BBR_STATE_STARTUP && Bbr->BtlbwFound) {
            BbrCongestionControlTransitToDrain(Cc);
        }

        if (Bbr->BbrState == BBR_STATE_DRAIN &&
               Bbr->BytesInFlight <= BbrCongestionControlGetTargetCwnd(Cc, GAIN_UNIT)) {
            BbrCongestionControlTransitToProbeBw(Cc, AckEvent->TimeNow);
        }

        if (Bbr->BbrState != BBR_STATE_PROBE_RTT &&
            !Bbr->ExitingQuiescence &&
            Bbr->RttSampleExpired) {
            BbrCongestionControlTransitToProbeRtt(Cc, AckEvent->LargestSentPacketNumber);
        }

        Bbr->ExitingQuiescence = FALSE;

        if (Bbr->BbrState == BBR_STATE_PROBE_RTT) {
            BbrCongestionControlHandleAckInProbeRtt(
                Cc, NewRoundTrip, AckEvent->LargestSentPacketNumber, AckEvent->TimeNow);
        }
    }

    BbrCongestionControlUpdateCongestionWindow(
        Cc, AckEvent->NumTotalAckedRetransmittableBytes, AckEvent->NumRetransmittableBytes);

    //
    // Cellular Ratio 处理和日志记录
    //
    if (AckEvent->AckedPackets != NULL) {
        const QUIC_PATH* Path = &Connection->Paths[0];
        QUIC_SENT_PACKET_METADATA* AckedPacket = AckEvent->AckedPackets;

        // 获取 BBR 指标
        uint64_t EstimatedBandwidth = BbrCongestionControlGetBandwidth(Cc);
        uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
        uint64_t SmoothedRtt = Path->GotFirstRttSample ? Path->SmoothedRtt : 0;
        uint64_t MinRtt = Bbr->MinRtt != UINT64_MAX ? Bbr->MinRtt : 0;
        uint64_t SampleRtt = AckEvent->MinRttValid ? AckEvent->MinRtt : SmoothedRtt;  // 瞬时RTT样本
        uint64_t BbrPacingRate = EstimatedBandwidth * Bbr->PacingGain / GAIN_UNIT;
        uint64_t DeliveryRate = Bbr->RecentDeliveryRate;
        uint64_t ConnectionDuration = AckEvent->TimeNow - Connection->Stats.Timing.Start;
        uint64_t TotalSent = Connection->Stats.Send.TotalPackets;
        uint64_t TotalLost = Connection->Stats.Send.SuspectedLostPackets;

        // Cellular Ratio 相关
        double cellularRatio = GetCellularRatio();
        double cellularRawRatio = GetCellularRawRatio();
        uint64_t cellularRatioSeq = GetCellularRatioUpdateSeq();
        int cellularRatioFresh = 0;
        double cellularGain = GetRatioGain(cellularRatio);
        uint64_t ratioPacingRate = 0;
        uint64_t ratioBaseRtt = GetRatioBaseRtt(MinRtt);
        uint32_t ratioCwnd = 0;
        BOOLEAN cellularInnerLoopActive = IsCellularInnerLoopActive(Bbr);
        BOOLEAN cellularOuterLoopActive = IsCellularOuterLoopActive(Bbr);
        BOOLEAN cellularOuterBackoffApplied = FALSE;
        BOOLEAN cellularOveruseGateOpen =
            IsCellularOuterLoopOveruseGateOpen(Bbr);
        BOOLEAN cellularBackoffCooldown =
            IsCellularOuterLoopBackoffCooldownActive(Bbr);
        double trendlineSlope = TrendlineEstimatorGetSlope(&Bbr->TrendlineEstimator);
        const char* overuseState =
            TrendlineEstimatorGetOveruseStateName(&Bbr->TrendlineEstimator);
        const char* loopName =
            !IsCellularControlActive() ? "BBR" :
            (cellularOuterLoopActive ? "OUTER_GCC" : "INNER_GBR");

        // 只有收到新的 cellular ratio 样本时才更新速率；旧 ratio 在多个 ACK 上复用时保持当前速率。
        if (IsCellularRatioAvailable()) {
            if (cellularOuterLoopActive) {
                cellularOuterBackoffApplied =
                    ApplyCellularOuterLoopBackoff(
                        Bbr, AckEvent->TimeNow, DeliveryRate, MinRtt);
            } else if (cellularInnerLoopActive) {
                cellularRatioFresh = ConsumeCellularRatioUpdate(cellularRatioSeq);
                if (cellularRatioFresh) {
                    UpdateRatioPacingRate(cellularRatio, EstimatedBandwidth);
                }
            }
            ratioPacingRate = GetRatioPacingRate();
            ratioBaseRtt = GetRatioBaseRtt(MinRtt);
            ratioCwnd = GetRatioCwnd(MinRtt);
        }

        // 遍历所有ACK的包，写入日志
        while (AckedPacket != NULL) {
            // 写入 CSV 格式日志
            FILE* logFile = OpenBbrLogFile();
            if (logFile != NULL) {
                // CSV格式: timestamp,pkt,size,bw_est,delivery_rate,cwnd,inflight,rtt,min_rtt,state,
                //          reserved,smoothed_ratio*1000,reserved,gain*1000,ratio_pacing_rate,ratio_cwnd,sample_rtt,raw_ratio*1000,ratio_seq,ratio_fresh,ratio_base_rtt
                fprintf(logFile, "%lu,%lu,%u,%lu,%lu,%u,%u,%lu,%lu,%d,739,%d,%d,%d,%lu,%u,%lu,%d,%lu,%d,%lu\n",
                       (unsigned long)ConnectionDuration,
                       (unsigned long)AckedPacket->PacketNumber,
                       AckedPacket->PacketLength,
                       (unsigned long)(EstimatedBandwidth / RATIO_BW_UNIT),
                       (unsigned long)(DeliveryRate / RATIO_BW_UNIT),
                       CongestionWindow,
                       Bbr->BytesInFlight,
                       (unsigned long)SmoothedRtt,
                       (unsigned long)MinRtt,
                       (int)Bbr->BbrState,
                       (int)(cellularRatio * 1000),
                       (int)(cellularRatio * 1000),
                       (int)(cellularGain * 1000),
                       (unsigned long)ratioPacingRate,
                       ratioCwnd,
                       (unsigned long)SampleRtt,
                       (int)(cellularRawRatio * 1000),
                       (unsigned long)cellularRatioSeq,
                       cellularRatioFresh,
                       (unsigned long)ratioBaseRtt);

                // 同时写入人类可读的日志
                fprintf(logFile, "[BBR-PKT-ACKED] T=%.3f s, PKT=%lu, Size=%u B, "
                       "EstBW=%.2f Mbps, PacingRate=%.2f Mbps, DeliveryRate=%.2f Mbps, "
                       "RTT=%lu us, MinRTT=%lu us, SampleRTT=%lu us, CWND=%u B, InFlight=%u B, "
                       "Loss=%.2f%%, State=%s, TotalSent=%lu, TotalLost=%lu, PacingGain=%.2fx, CwndGain=%.2fx, "
                       "CellularRatioRaw=%.3f, CellularRatioSmoothed=%.3f, CellularGain=%.4f, "
                       "RatioPacingRate=%.2f Mbps, RatioCwnd=%u B, RatioBaseRTT=%lu us, RatioSeq=%lu, RatioFresh=%d, "
                       "TrendlineSlope=%.6f, OveruseState=%s, Loop=%s, "
                       "OuterBackoff=%d, OveruseGate=%d, BackoffCooldown=%d\n",
                       ConnectionDuration / 1000000.0,
                       (unsigned long)AckedPacket->PacketNumber,
                       AckedPacket->PacketLength,
                       EstimatedBandwidth / 1000000.0,
                       BbrPacingRate / 1000000.0,
                       DeliveryRate / 1000000.0,
                       (unsigned long)SmoothedRtt,
                       (unsigned long)MinRtt,
                       (unsigned long)SampleRtt,
                       CongestionWindow,
                       Bbr->BytesInFlight,
                       TotalSent > 0 ? ((double)TotalLost * 100.0) / (double)TotalSent : 0.0,
                       Bbr->BbrState == 0 ? "STARTUP" :
                       Bbr->BbrState == 1 ? "DRAIN" :
                       Bbr->BbrState == 2 ? "PROBE_BW" :
                       Bbr->BbrState == 3 ? "PROBE_RTT" : "UNKNOWN",
                       (unsigned long)TotalSent,
                       (unsigned long)TotalLost,
                       (double)Bbr->PacingGain / (double)GAIN_UNIT,
                       (double)Bbr->CwndGain / (double)GAIN_UNIT,
                       cellularRawRatio,
                       cellularRatio,
                       cellularGain,
                       ratioPacingRate * 8.0 / 1000000.0,
                       ratioCwnd,
                       (unsigned long)ratioBaseRtt,
                       (unsigned long)cellularRatioSeq,
                       cellularRatioFresh,
                       trendlineSlope,
                       overuseState,
                       loopName,
                       cellularOuterBackoffApplied,
                       cellularOveruseGateOpen,
                       cellularBackoffCooldown);
                fclose(logFile);
            }

            AckedPacket = AckedPacket->Next;
        }
    }

#ifdef QUIC_ENHANCED_PACKET_LOGGING
    // Enhanced packet level logging for acknowledged packets
    if (g_BbrPacketLoggerInitialized && AckEvent->AckedPackets != NULL) {
        QUIC_SENT_PACKET_METADATA* AckedPacket = AckEvent->AckedPackets;
        while (AckedPacket != NULL) {
            BbrPacketLevelLoggingRecordPacketAcknowledged(
                &g_BbrPacketLogger, Cc, AckedPacket->PacketNumber, 
                AckedPacket->PacketLength, AckEvent->TimeNow);
            AckedPacket = AckedPacket->Next;
        }
    }
#endif

    if (Connection->Settings.NetStatsEventEnabled) {
        BbrCongestionControlIndicateConnectionEvent(Connection, Cc);
    }

    // Generate periodic log
    BbrCongestionControlPeriodicLog(Cc);

    return BbrCongestionControlUpdateBlockedState(Cc, PreviousCanSendState);
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlOnDataLost(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ const QUIC_LOSS_EVENT* LossEvent
    )
{
    QUIC_CONGESTION_CONTROL_BBR *Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    QuicTraceEvent(
        ConnCongestionV2,
        "[conn][%p] Congestion event: IsEcn=%hu",
        Connection,
        FALSE);
    Connection->Stats.Send.CongestionCount++;

    BOOLEAN PreviousCanSendState = BbrCongestionControlCanSend(Cc);

    CXPLAT_DBG_ASSERT(LossEvent->NumRetransmittableBytes > 0);

    Bbr->EndOfRecoveryValid = TRUE;
    Bbr->EndOfRecovery = LossEvent->LargestSentPacketNumber;

    CXPLAT_DBG_ASSERT(Bbr->BytesInFlight >= LossEvent->NumRetransmittableBytes);
    Bbr->BytesInFlight -= LossEvent->NumRetransmittableBytes;

    // Log packet loss event (disabled for periodic logging)
    {
        const QUIC_PATH* Path = &Connection->Paths[0];
        uint64_t EstimatedBandwidth = BbrCongestionControlGetBandwidth(Cc);
        uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
        uint64_t SmoothedRtt = Path->GotFirstRttSample ? Path->SmoothedRtt : 0;
        uint64_t MinRtt = Bbr->MinRtt != UINT64_MAX ? Bbr->MinRtt : 0;
        uint64_t PacingRate = EstimatedBandwidth * Bbr->PacingGain / GAIN_UNIT;
        uint64_t DeliveryRate = Bbr->RecentDeliveryRate;
        
        // Calculate connection duration
        uint64_t CurrentTime = CxPlatTimeUs64();
        uint64_t ConnectionDuration = CurrentTime - Connection->Stats.Timing.Start;
        
        // Calculate loss rate
        uint64_t TotalSent = Connection->Stats.Send.TotalPackets;
        uint64_t TotalLost = Connection->Stats.Send.SuspectedLostPackets;
        double LossRate = TotalSent > 0 ? ((double)TotalLost * 100.0) / (double)TotalSent : 0.0;
        
        // Write detailed BBR loss log to file
        FILE* logFile = fopen("/home/qwu26/msquic_cellular/artifacts/bbr_logs/bbr_log.txt", "a");
        if (logFile != NULL) {
            double trendlineSlope = TrendlineEstimatorGetSlope(&Bbr->TrendlineEstimator);
            fprintf(logFile, "[BBR-PKT-LOST] T=%lu.%03lu s, PKT=%lu, Size=%u B, "
                   "EstBW=%.2f Mbps, PacingRate=%.2f Mbps, DeliveryRate=%.2f Mbps, "
                   "RTT=%lu us, MinRTT=%lu us, CWND=%u B, InFlight=%u B, "
                   "Loss=%.2f%%, State=%s, TotalSent=%lu, TotalLost=%lu, PersistentCongestion=%s, "
                   "PacingGain=%.2fx, CwndGain=%.2fx, TrendlineSlope=%.6f\n",
                   (unsigned long)(ConnectionDuration / 1000000),
                   (unsigned long)((ConnectionDuration % 1000000) / 1000),
                   (unsigned long)LossEvent->LargestPacketNumberLost,
                   LossEvent->NumRetransmittableBytes,
                   EstimatedBandwidth / 1000000.0,
                   PacingRate / 1000000.0,
                   DeliveryRate / 1000000.0,
                   (unsigned long)SmoothedRtt,
                   (unsigned long)MinRtt,
                   CongestionWindow,
                   Bbr->BytesInFlight,
                   LossRate,
                   Bbr->BbrState == 0 ? "STARTUP" :
                   Bbr->BbrState == 1 ? "DRAIN" :
                   Bbr->BbrState == 2 ? "PROBE_BW" :
                   Bbr->BbrState == 3 ? "PROBE_RTT" : "UNKNOWN",
                   (unsigned long)TotalSent,
                   (unsigned long)TotalLost,
                   LossEvent->PersistentCongestion ? "YES" : "NO",
                   (double)Bbr->PacingGain / (double)GAIN_UNIT,
                   (double)Bbr->CwndGain / (double)GAIN_UNIT,
                   trendlineSlope);
            fclose(logFile);
        }
    }

#ifdef QUIC_ENHANCED_PACKET_LOGGING
    // Enhanced packet level logging for lost packets
    if (g_BbrPacketLoggerInitialized) {
        // Log the largest lost packet number as a representative
        BbrPacketLevelLoggingRecordPacketLost(
            &g_BbrPacketLogger, Cc, LossEvent->LargestPacketNumberLost, LossEvent->NumRetransmittableBytes);
    }
#endif

    uint32_t RecoveryWindow = Bbr->RecoveryWindow;
    uint32_t MinCongestionWindow = kMinCwndInMss * DatagramPayloadLength;

    if (!BbrCongestionControlInRecovery(Cc)) {
        Bbr->RecoveryState = RECOVERY_STATE_CONSERVATIVE;
        RecoveryWindow = Bbr->BytesInFlight;

        RecoveryWindow = CXPLAT_MAX(RecoveryWindow, MinCongestionWindow);

        Bbr->EndOfRoundTripValid = TRUE;
        Bbr->EndOfRoundTrip = LossEvent->LargestSentPacketNumber;
    }

    if (LossEvent->PersistentCongestion) {
        Bbr->RecoveryWindow = MinCongestionWindow;

        QuicTraceEvent(
            ConnPersistentCongestion,
            "[conn][%p] Persistent congestion event",
            Connection);
        Connection->Stats.Send.PersistentCongestionCount++;
    } else {
        Bbr->RecoveryWindow =
            RecoveryWindow > LossEvent->NumRetransmittableBytes + MinCongestionWindow
            ? RecoveryWindow - LossEvent->NumRetransmittableBytes
            : MinCongestionWindow;
    }

    BbrCongestionControlUpdateBlockedState(Cc, PreviousCanSendState);
    QuicConnLogBbr(QuicCongestionControlGetConnection(Cc));
}

_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
BbrCongestionControlOnSpuriousCongestionEvent(
    _In_ QUIC_CONGESTION_CONTROL* Cc
    )
{
    UNREFERENCED_PARAMETER(Cc);
    return FALSE;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlSetAppLimited(
    _In_ struct QUIC_CONGESTION_CONTROL* Cc
    )
{
    QUIC_CONGESTION_CONTROL_BBR *Bbr = &Cc->Bbr;

    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    uint64_t LargestSentPacketNumber = Connection->LossDetection.LargestSentPacketNumber;

    if (Bbr->BytesInFlight > BbrCongestionControlGetCongestionWindow(Cc)) {
        return;
    }

    Bbr->BandwidthFilter.AppLimited = TRUE;
    Bbr->BandwidthFilter.AppLimitedExitTarget = LargestSentPacketNumber;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlReset(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ BOOLEAN FullReset
    )
{
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    Bbr->CongestionWindow = Bbr->InitialCongestionWindowPackets * DatagramPayloadLength;
    Bbr->InitialCongestionWindow = Bbr->InitialCongestionWindowPackets * DatagramPayloadLength;
    Bbr->RecoveryWindow = kDefaultRecoveryCwndInMss * DatagramPayloadLength;
    Bbr->BytesInFlightMax = Bbr->CongestionWindow / 2;

    if (FullReset) {
        Bbr->BytesInFlight = 0;
    }
    Bbr->Exemptions = 0;

    Bbr->RecoveryState = RECOVERY_STATE_NOT_RECOVERY;
    Bbr->BbrState = BBR_STATE_STARTUP;
    Bbr->RoundTripCounter = 0;
    Bbr->CwndGain = kHighGain;
    Bbr->PacingGain = kHighGain;
    Bbr->BtlbwFound = FALSE;
    Bbr->SendQuantum = 0;
    Bbr->SlowStartupRoundCounter = 0 ;

    Bbr->PacingCycleIndex = 0;
    Bbr->AggregatedAckBytes = 0;
    Bbr->ExitingQuiescence = FALSE;
    Bbr->LastEstimatedStartupBandwidth = 0;

    Bbr->AckAggregationStartTimeValid = FALSE;
    Bbr->AckAggregationStartTime = CxPlatTimeUs64();
    Bbr->CycleStart = 0;

    Bbr->EndOfRecoveryValid = FALSE;
    Bbr->EndOfRecovery = 0;

    Bbr->ProbeRttRoundValid = FALSE;
    Bbr->ProbeRttRound = 0;

    Bbr->EndOfRoundTripValid = FALSE;
    Bbr->EndOfRoundTrip = 0;

    Bbr->ProbeRttEndTimeValid = FALSE;
    Bbr->ProbeRttEndTime = CxPlatTimeUs64();

    Bbr->RttSampleExpired = TRUE;
    Bbr->MinRttTimestampValid = FALSE;
    Bbr->MinRtt = UINT64_MAX;
    Bbr->MinRttTimestamp = 0;

    // Initialize recent delivery rate tracking fields
    Bbr->RecentSendRate = 0;
    Bbr->RecentAckRate = 0;
    Bbr->RecentDeliveryRate = 0;
    Bbr->CellularRatioAboveThresholdStartTimeValid = FALSE;
    Bbr->CellularRatioAboveThresholdStartTime = 0;
    Bbr->CellularOveruseStartTimeValid = FALSE;
    Bbr->CellularOveruseStartTime = 0;
    Bbr->CellularOuterLoopLastBackoffTimeValid = FALSE;
    Bbr->CellularOuterLoopLastBackoffTime = 0;

    QuicSlidingWindowExtremumReset(&Bbr->MaxAckHeightFilter);

    QuicSlidingWindowExtremumReset(&Bbr->BandwidthFilter.WindowedMaxFilter);
    Bbr->BandwidthFilter.AppLimited = FALSE;
    Bbr->BandwidthFilter.AppLimitedExitTarget = 0;

    BbrCongestionControlLogOutFlowStatus(Cc);
    QuicConnLogBbr(Connection);
}


static const QUIC_CONGESTION_CONTROL QuicCongestionControlBbr = {
    .Name = "BBR",
    .QuicCongestionControlCanSend = BbrCongestionControlCanSend,
    .QuicCongestionControlSetExemption = BbrCongestionControlSetExemption,
    .QuicCongestionControlReset = BbrCongestionControlReset,
    .QuicCongestionControlGetSendAllowance = BbrCongestionControlGetSendAllowance,
    .QuicCongestionControlGetCongestionWindow = BbrCongestionControlGetCongestionWindow,
    .QuicCongestionControlOnDataSent = BbrCongestionControlOnDataSent,
    .QuicCongestionControlOnDataInvalidated = BbrCongestionControlOnDataInvalidated,
    .QuicCongestionControlOnDataAcknowledged = BbrCongestionControlOnDataAcknowledged,
    .QuicCongestionControlOnDataLost = BbrCongestionControlOnDataLost,
    .QuicCongestionControlOnEcn = NULL,
    .QuicCongestionControlOnSpuriousCongestionEvent = BbrCongestionControlOnSpuriousCongestionEvent,
    .QuicCongestionControlLogOutFlowStatus = BbrCongestionControlLogOutFlowStatus,
    .QuicCongestionControlGetExemptions = BbrCongestionControlGetExemptions,
    .QuicCongestionControlGetBytesInFlightMax = BbrCongestionControlGetBytesInFlightMax,
    .QuicCongestionControlIsAppLimited = BbrCongestionControlIsAppLimited,
    .QuicCongestionControlSetAppLimited = BbrCongestionControlSetAppLimited,
    .QuicCongestionControlLogPacketSent = BbrCongestionControlLogPacketSent,
};

_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlInitialize(
    _In_ QUIC_CONGESTION_CONTROL* Cc,
    _In_ const QUIC_SETTINGS_INTERNAL* Settings
    )
{
    *Cc = QuicCongestionControlBbr;

    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;

    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);

    const uint16_t DatagramPayloadLength =
        QuicPathGetDatagramPayloadSize(&Connection->Paths[0]);

    Bbr->InitialCongestionWindowPackets = Settings->InitialWindowPackets;

    Bbr->CongestionWindow = Bbr->InitialCongestionWindowPackets * DatagramPayloadLength;
    Bbr->InitialCongestionWindow = Bbr->InitialCongestionWindowPackets * DatagramPayloadLength;
    Bbr->RecoveryWindow = kDefaultRecoveryCwndInMss * DatagramPayloadLength;
    Bbr->BytesInFlightMax = Bbr->CongestionWindow / 2;

    Bbr->BytesInFlight = 0;
    Bbr->Exemptions = 0;

    Bbr->RecoveryState = RECOVERY_STATE_NOT_RECOVERY;
    Bbr->BbrState = BBR_STATE_STARTUP;
    Bbr->RoundTripCounter = 0;
    Bbr->CwndGain = kHighGain;
    Bbr->PacingGain = kHighGain;
    Bbr->BtlbwFound = FALSE;
    Bbr->SendQuantum = 0;
    Bbr->SlowStartupRoundCounter = 0 ;

    Bbr->PacingCycleIndex = 0;
    Bbr->AggregatedAckBytes = 0;
    Bbr->ExitingQuiescence = FALSE;
    Bbr->LastEstimatedStartupBandwidth = 0;
    Bbr->CycleStart = 0;

    Bbr->AckAggregationStartTimeValid = FALSE;
    Bbr->AckAggregationStartTime = CxPlatTimeUs64();

    Bbr->EndOfRecoveryValid = FALSE;
    Bbr->EndOfRecovery = 0;

    Bbr->ProbeRttRoundValid = FALSE;
    Bbr->ProbeRttRound = 0;

    Bbr->EndOfRoundTripValid = FALSE;
    Bbr->EndOfRoundTrip = 0;

    Bbr->ProbeRttEndTimeValid = FALSE;
    Bbr->ProbeRttEndTime = 0;

    Bbr->RttSampleExpired = TRUE;
    Bbr->MinRttTimestampValid = FALSE;
    Bbr->MinRtt = UINT64_MAX;
    Bbr->MinRttTimestamp = 0;

    Bbr->MaxAckHeightFilter = QuicSlidingWindowExtremumInitialize(
            kBbrMaxAckHeightFilterLen, kBbrDefaultFilterCapacity, Bbr->MaxAckHeightFilterEntries);

    Bbr->BandwidthFilter = (BBR_BANDWIDTH_FILTER) {
        .WindowedMaxFilter = QuicSlidingWindowExtremumInitialize(
                kBbrMaxBandwidthFilterLen, kBbrDefaultFilterCapacity, Bbr->BandwidthFilter.WindowedMaxFilterEntries),
        .AppLimited = FALSE,
        .AppLimitedExitTarget = 0,
    };

    // Initialize periodic logging fields
    Bbr->LastPeriodicLogTime = CxPlatTimeUs64();
    Bbr->LastLoggedSendBytes = 0;
    Bbr->LastLoggedRecvBytes = 0;
    Bbr->LastLoggedSentPackets = 0;
    Bbr->LastLoggedLostPackets = 0;

    // Initialize delay tracking fields
    Bbr->RecentSendDelay = 0;
    Bbr->RecentAckDelay = 0;
    Bbr->CellularRatioAboveThresholdStartTimeValid = FALSE;
    Bbr->CellularRatioAboveThresholdStartTime = 0;
    Bbr->CellularOveruseStartTimeValid = FALSE;
    Bbr->CellularOveruseStartTime = 0;
    Bbr->CellularOuterLoopLastBackoffTimeValid = FALSE;
    Bbr->CellularOuterLoopLastBackoffTime = 0;

    // Initialize trendline slope estimator
    TrendlineEstimatorInitialize(&Bbr->TrendlineEstimator);
    Bbr->PrevSendTimeUs = 0;
    Bbr->PrevRecvTimeUs = 0;

    QuicConnLogOutFlowStats(Connection);
    QuicConnLogBbr(Connection);

#ifdef QUIC_ENHANCED_PACKET_LOGGING
    // Initialize the global BBR packet logger if not already done
    if (!g_BbrPacketLoggerInitialized) {
        if (QUIC_SUCCEEDED(BbrPacketLevelLoggingInitialize(&g_BbrPacketLogger, 10000))) {
            g_BbrPacketLoggerInitialized = TRUE;
            printf("BBR Enhanced Packet Logging: Initialized with 10000 entries\n");
        }
    }
#endif
}

//
// Generate BBR performance summary when connection ends
//
_IRQL_requires_max_(PASSIVE_LEVEL)
void
BbrCongestionControlGeneratePerformanceSummary(
    _In_ const QUIC_CONGESTION_CONTROL* Cc
    )
{
    const QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    const QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    const QUIC_PATH* Path = &Connection->Paths[0];
    
    // Use a simple flag to ensure we only print once per connection
    // This is a simple approach - in production you might want a more sophisticated mechanism
    static const QUIC_CONNECTION* LastConnection = NULL;
    if (LastConnection == Connection) {
        return; // Already printed for this connection
    }
    LastConnection = Connection;
    
    // Calculate connection duration
    uint64_t CurrentTime = CxPlatTimeUs64();
    uint64_t ConnectionDuration = CurrentTime - Connection->Stats.Timing.Start;
    
    // Calculate bandwidth metrics
    uint64_t EstimatedBandwidth = BbrCongestionControlGetBandwidth(Cc);
    uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
    
    // Calculate loss rate
    uint64_t TotalSent = Connection->Stats.Send.TotalPackets;
    uint64_t TotalLost = Connection->Stats.Send.SuspectedLostPackets;
    uint32_t LossRate = TotalSent > 0 ? (uint32_t)((TotalLost * 10000) / TotalSent) : 0;
    
    // Calculate actual bandwidth from bytes transferred
    uint64_t SendBytes = Connection->Stats.Send.TotalBytes;
    uint64_t RecvBytes = Connection->Stats.Recv.TotalBytes;
    uint64_t TotalBytes = SendBytes + RecvBytes;
    
    double SendBandwidthMbps = 0.0;
    double RecvBandwidthMbps = 0.0;
    double TotalBandwidthMbps = 0.0;
    
    if (ConnectionDuration > 0) {
        // Convert: bytes -> bits -> Mbps
        // bytes * 8 = bits, ConnectionDuration is in microseconds
        // (bits * 1,000,000) / ConnectionDuration = bits per second
        // bits per second / 1,000,000 = Mbps
        SendBandwidthMbps = ((double)SendBytes * 8.0 * 1000000.0) / ((double)ConnectionDuration * 1000000.0);
        RecvBandwidthMbps = ((double)RecvBytes * 8.0 * 1000000.0) / ((double)ConnectionDuration * 1000000.0);
        TotalBandwidthMbps = ((double)TotalBytes * 8.0 * 1000000.0) / ((double)ConnectionDuration * 1000000.0);
    }
    
    // Print BBR performance summary to file
    FILE* summaryFile = fopen("/home/qwu26/msquic_cellular/artifacts/bbr_logs/bbr_summary.txt", "w");
    if (summaryFile != NULL) {
        fprintf(summaryFile, "\n=== BBR Performance Summary ===\n");
        fprintf(summaryFile, "Connection Duration: %lu.%03lu s\n", 
               (unsigned long)(ConnectionDuration / 1000000), 
               (unsigned long)((ConnectionDuration % 1000000) / 1000));
        fprintf(summaryFile, "Debug: Start Time: %lu us, Current Time: %lu us, Duration: %lu us\n",
               (unsigned long)Connection->Stats.Timing.Start,
               (unsigned long)CurrentTime,
               (unsigned long)ConnectionDuration);
        fprintf(summaryFile, "BBR State: %s\n", 
               Bbr->BbrState == 0 ? "STARTUP" :
               Bbr->BbrState == 1 ? "DRAIN" :
               Bbr->BbrState == 2 ? "PROBE_BW" :
               Bbr->BbrState == 3 ? "PROBE_RTT" : "UNKNOWN");
        fprintf(summaryFile, "Estimated Bandwidth: %.2f Mbps\n", EstimatedBandwidth / 1000000.0);
        fprintf(summaryFile, "Send Bandwidth: %.2f Mbps\n", SendBandwidthMbps);
        fprintf(summaryFile, "Recv Bandwidth: %.2f Mbps\n", RecvBandwidthMbps);
        fprintf(summaryFile, "Total Bandwidth: %.2f Mbps\n", TotalBandwidthMbps);
        fprintf(summaryFile, "Congestion Window: %u bytes\n", CongestionWindow);
        fprintf(summaryFile, "Pacing Gain: %.2fx\n", (double)Bbr->PacingGain / (double)GAIN_UNIT);
        fprintf(summaryFile, "Cwnd Gain: %.2fx\n", (double)Bbr->CwndGain / (double)GAIN_UNIT);
        fprintf(summaryFile, "RTT: %lu us (Min: %lu us)\n", 
               (unsigned long)(Path->GotFirstRttSample ? Path->SmoothedRtt : 0), 
               (unsigned long)Bbr->MinRtt);
        fprintf(summaryFile, "Packets Sent: %lu\n", (unsigned long)TotalSent);
        fprintf(summaryFile, "Packets Lost: %lu (%.2f%%)\n", (unsigned long)TotalLost, LossRate / 100.0);
        fprintf(summaryFile, "Congestion Events: %u\n", Connection->Stats.Send.CongestionCount);
        fprintf(summaryFile, "Bytes Sent: %lu bytes\n", (unsigned long)SendBytes);
        fprintf(summaryFile, "Bytes Received: %lu bytes\n", (unsigned long)RecvBytes);
        fprintf(summaryFile, "Total Bytes: %lu bytes\n", (unsigned long)TotalBytes);
        fprintf(summaryFile, "Bytes In Flight: %u bytes\n", Bbr->BytesInFlight);
        fprintf(summaryFile, "App Limited: %s\n", BbrCongestionControlIsAppLimited(Cc) ? "YES" : "NO");
        fprintf(summaryFile, "==============================\n\n");
        fclose(summaryFile);
    }
}

//
// Generate periodic BBR performance log (every second)
//
_IRQL_requires_max_(DISPATCH_LEVEL)
void
BbrCongestionControlPeriodicLog(
    _In_ QUIC_CONGESTION_CONTROL* Cc
    )
{
    QUIC_CONNECTION* Connection = QuicCongestionControlGetConnection(Cc);
    QUIC_CONGESTION_CONTROL_BBR* Bbr = &Cc->Bbr;
    const QUIC_PATH* Path = &Connection->Paths[0];
    
    uint64_t CurrentTime = CxPlatTimeUs64();
    uint64_t TimeSinceLastLog = CurrentTime - Bbr->LastPeriodicLogTime;
    
    // Log every 10ms (10,000 microseconds)
    if (TimeSinceLastLog < 10000) {
        return;
    }
    
    // Get current statistics
    uint64_t CurrentSendBytes = Connection->Stats.Send.TotalBytes;
    uint64_t CurrentRecvBytes = Connection->Stats.Recv.TotalBytes;
    uint64_t CurrentSentPackets = Connection->Stats.Send.TotalPackets;
    uint64_t CurrentLostPackets = Connection->Stats.Send.SuspectedLostPackets;
    
    // Calculate deltas since last log
    uint64_t DeltaSendBytes = CurrentSendBytes - Bbr->LastLoggedSendBytes;
    uint64_t DeltaRecvBytes = CurrentRecvBytes - Bbr->LastLoggedRecvBytes;
    uint64_t DeltaSentPackets = CurrentSentPackets - Bbr->LastLoggedSentPackets;
    uint64_t DeltaLostPackets = CurrentLostPackets - Bbr->LastLoggedLostPackets;
    
    // Calculate bandwidth (convert to Mbps)
    double SendBandwidthMbps = 0.0;
    double RecvBandwidthMbps = 0.0;
    double TotalBandwidthMbps = 0.0;
    
    if (TimeSinceLastLog > 0) {
        // Convert: bytes -> bits -> Mbps
        // (bytes * 8 * 1,000,000) / TimeSinceLastLog = bits per second
        // bits per second / 1,000,000 = Mbps
        SendBandwidthMbps = ((double)DeltaSendBytes * 8.0 * 1000000.0) / ((double)TimeSinceLastLog * 1000000.0);
        RecvBandwidthMbps = ((double)DeltaRecvBytes * 8.0 * 1000000.0) / ((double)TimeSinceLastLog * 1000000.0);
        TotalBandwidthMbps = SendBandwidthMbps + RecvBandwidthMbps;
    }
    
    // Use delta lost packets for this time interval
    uint64_t IntervalLostPackets = DeltaLostPackets;
    
    // Get BBR metrics
    uint64_t EstimatedBandwidth = BbrCongestionControlGetBandwidth(Cc);
    uint32_t CongestionWindow = BbrCongestionControlGetCongestionWindow(Cc);
    uint64_t SmoothedRtt = Path->GotFirstRttSample ? Path->SmoothedRtt : 0;
    uint64_t MinRtt = Bbr->MinRtt != UINT64_MAX ? Bbr->MinRtt : 0;
    
    // Calculate pacing rate: Bandwidth * PacingGain / GAIN_UNIT
    uint64_t PacingRate = EstimatedBandwidth * Bbr->PacingGain / GAIN_UNIT;
    double PacingRateMbps = PacingRate / 1000000.0;
    
    // Debug: Show pacing gain value
    double PacingGainRatio = (double)Bbr->PacingGain / (double)GAIN_UNIT;
    
    // Get delivery rate from BBR structure (already calculated as min(send rate, ack rate))
    uint64_t DeliveryRate = Bbr->RecentDeliveryRate;
    double DeliveryRateMbps = DeliveryRate / 1000000.0;
    
    // Calculate connection duration
    uint64_t ConnectionDuration = CurrentTime - Connection->Stats.Timing.Start;
    
    // Print periodic log to file
    FILE* logFile = fopen("/home/qwu26/msquic_cellular/artifacts/bbr_logs/bbr_log_10ms.txt", "a");
    if (logFile != NULL) {
        double trendlineSlope = TrendlineEstimatorGetSlope(&Bbr->TrendlineEstimator);
        fprintf(logFile, "[BBR-LOG] T=%lu.%03lu s, Send=%.2f Mbps, Recv=%.2f Mbps, Total=%.2f Mbps, "
                "EstBW=%.2f Mbps, PacingRate=%.2f Mbps, PacingGain=%.2fx, CwndGain=%.2fx, DeliveryRate=%.2f Mbps, "
                "RTT=%lu us, MinRTT=%lu us, CWND=%u B, InFlight=%u B, "
                "Lost=%lu, State=%s, Pkts=%lu/%lu, Bytes=%lu/%lu, "
                "SendDelay=%lu us, AckDelay=%lu us, TrendlineSlope=%.6f\n",
                (unsigned long)(ConnectionDuration / 1000000),
                (unsigned long)((ConnectionDuration % 1000000) / 1000),
                SendBandwidthMbps,
                RecvBandwidthMbps,
                TotalBandwidthMbps,
                EstimatedBandwidth / 1000000.0,
                PacingRateMbps,
                PacingGainRatio,
                (double)Bbr->CwndGain / (double)GAIN_UNIT,
                DeliveryRateMbps,
                (unsigned long)SmoothedRtt,
                (unsigned long)MinRtt,
                CongestionWindow,
                Bbr->BytesInFlight,
                (unsigned long)IntervalLostPackets,
                Bbr->BbrState == 0 ? "STARTUP" :
                Bbr->BbrState == 1 ? "DRAIN" :
                Bbr->BbrState == 2 ? "PROBE_BW" :
                Bbr->BbrState == 3 ? "PROBE_RTT" : "UNKNOWN",
                (unsigned long)DeltaSentPackets,
                (unsigned long)DeltaLostPackets,
                (unsigned long)DeltaSendBytes,
                (unsigned long)DeltaRecvBytes,
                (unsigned long)Bbr->RecentSendDelay,
                (unsigned long)Bbr->RecentAckDelay,
                trendlineSlope);
        fclose(logFile);
    }
    
    // Update last logged values
    Bbr->LastPeriodicLogTime = CurrentTime;
    Bbr->LastLoggedSendBytes = CurrentSendBytes;
    Bbr->LastLoggedRecvBytes = CurrentRecvBytes;
    Bbr->LastLoggedSentPackets = CurrentSentPackets;
    Bbr->LastLoggedLostPackets = CurrentLostPackets;
}

// Teensy 4.1 TMS 데이터 로거 (복원본)
//
// 보드에서 읽어낸 fixed_v4 펌웨어(2026-11-09 readback)를 역공학해 다시 작성했다.
// 원본 식별: "LG07 TMS Logger v6.15" / fw=LG07_TMS_GS6_16_SPEC975K_MAX_SD_FAST_FAILOVER
// 변수와 함수 이름은 복원하면서 새로 붙였다. 레지스터 값, 파일 형식, 시리얼 출력은 원본에 맞췄다.
//
// 파일 구성
//   Teensy_TMS_restored.ino  설정값, 전역 상태, setup, loop
//   adc_capture.ino          ADC, ADC_ETC, eDMA, 프레임 처리, FIR
//   logging.ino              SD 파일, 512바이트 헤더, 기록 시작과 중지, 전원 재개
//   fallback_usb.ino         SD 장애 대체 경로, PSRAM, USB 이진 스트림, SD 복구
//   commands.ino             USB 명령 처리
//   reports.ino              @LIVE, @DAQ, 상태 출력

#include <Arduino.h>
#include <DMAChannel.h>
#include <EEPROM.h>
#include <SD.h>

// ---------------------------------------------------------------------------
// 레지스터. Teensy 코어 헤더의 이름에 의존하지 않도록 주소로 정의한다.
// ---------------------------------------------------------------------------
#ifndef REG32
#define REG32(a) (*(volatile uint32_t*)(a))
#define REG16(a) (*(volatile uint16_t*)(a))
#endif

#define ETC_CTRL           REG32(0x403B0000)
#define ETC_DONE0_1_IRQ    REG32(0x403B0004)
#define ETC_DONE2_ERR_IRQ  REG32(0x403B0008)
#define ETC_DMA_CTRL       REG32(0x403B000C)
#define ETC_TRIG0_CTRL     REG32(0x403B0010)
#define ETC_TRIG0_CHAIN10  REG32(0x403B0018)
#define ETC_TRIG0_CHAIN32  REG32(0x403B001C)
#define ETC_TRIG0_CHAIN54  REG32(0x403B0020)
#define ETC_TRIG0_CHAIN76  REG32(0x403B0024)
#define ETC_TRIG0_RES10_ADDR 0x403B0028
#define ETC_TRIG0_RES32_ADDR 0x403B002C
#define ETC_TRIG4_CTRL     REG32(0x403B00B0)
#define ETC_TRIG4_CHAIN10  REG32(0x403B00B8)
#define ETC_TRIG4_CHAIN32  REG32(0x403B00BC)
#define ETC_TRIG4_CHAIN54  REG32(0x403B00C0)
#define ETC_TRIG4_CHAIN76  REG32(0x403B00C4)
#define ETC_TRIG4_RES10_ADDR 0x403B00C8

// FlexPWM4 서브모듈 2 (ADC 트리거 발생용)
#define PWM4_SM2_INIT   REG16(0x403E80C2)
#define PWM4_SM2_CTRL2  REG16(0x403E80C4)
#define PWM4_SM2_CTRL   REG16(0x403E80C6)
#define PWM4_SM2_VAL0   REG16(0x403E80CA)
#define PWM4_SM2_VAL1   REG16(0x403E80CE)
#define PWM4_SM2_TCTRL  REG16(0x403E80EA)
#define PWM4_MCTRL      REG16(0x403E8188)
#define PWM4_MCTRL_RUN_SM2 0x0400

#define ADC1_REG_HC0  REG32(0x400C4000)
#define ADC1_REG_HS   REG32(0x400C4020)
#define ADC1_REG_R0   REG32(0x400C4024)
#define ADC1_REG_CFG  REG32(0x400C4044)
#define ADC1_REG_GC   REG32(0x400C4048)
#define ADC1_REG_GS   REG32(0x400C404C)
#define ADC2_REG_HC0  REG32(0x400C8000)
#define ADC2_REG_CFG  REG32(0x400C8044)
#define ADC2_REG_GC   REG32(0x400C8048)
#define ADC2_REG_GS   REG32(0x400C804C)

#define CCM_REG_CCGR1 REG32(0x400FC06C)
#define CCM_REG_CCGR2 REG32(0x400FC070)
#define CCM_REG_CCGR4 REG32(0x400FC078)
#define GPIO8_REG_PSR REG32(0x42008008)   // SD 감지 핀 46 = 비트 17

#define ADC_ETC_ERROR_IRQ 121
#define DMAMUX_SRC_ADC_ETC 0x17

// ---------------------------------------------------------------------------
// 핀과 측정값 환산
// ---------------------------------------------------------------------------
const uint8_t PIN_LOADCELL  = 14;  // A0, ADC1 CH7
const uint8_t PIN_PRESSURE1 = 15;  // A1, ADC1 CH8
const uint8_t PIN_PRESSURE2 = 16;  // A2, ADC2 CH12
const uint8_t PIN_SD_DETECT = 46;

const uint32_t ADC_CH_LOADCELL  = 7;
const uint32_t ADC_CH_PRESSURE1 = 8;
const uint32_t ADC_CH_PRESSURE2 = 12;

const float ADC_VREF = 3.3f;
const float LC_K = 6503.0835f;
const float P1_K = 33.24468f;
const float P2_K = 33.2668f;

// ---------------------------------------------------------------------------
// ADC 설정
// ---------------------------------------------------------------------------
const uint32_t ADC_CFG_CALIBRATE = 0x73B;
const uint32_t ADC_CFG_SOFTWARE  = 0x10448;  // 12비트, 버스/4, 소프트웨어 트리거
const uint32_t ADC_CFG_HARDWARE  = 0x12448;  // 12비트, 버스/4, 하드웨어 트리거
const uint32_t ADC_DIRECT_CLOCK_HZ = 37500000;

const uint32_t SPEC_FRAME_HZ     = 243750;
const uint32_t SPEC_TARGET_SPS   = 975000;
const uint32_t SPEC_RATED_SPS    = 1000000;
const uint32_t MAX_INITIAL_HZ    = 500000;
const uint32_t FALLBACK_ADC_HZ   = 20000;
const uint32_t LOG_5K_HZ         = 5000;
const uint32_t CYCLES_PER_5K     = 120000;  // 600 MHz 기준 200 us
const uint32_t CPU_HZ_NOMINAL    = 600000000;
const uint32_t FALLBACK_US_STEP  = 50;
const uint32_t FIR_GROUP_DELAY_US = 2400;

const uint32_t DMA_HALF_FRAMES  = 128;
const uint32_t DMA_TOTAL_FRAMES = 256;
const uint32_t DMA_FRAME_BYTES  = 12;

// ---------------------------------------------------------------------------
// 저장 설정
// ---------------------------------------------------------------------------
enum { STORAGE_NONE = 0, STORAGE_SD = 1, STORAGE_FALLBACK = 2 };
enum { PROFILE_SPEC_SAFE = 0, PROFILE_MAX_SPEED = 1 };
enum { START_NORMAL = 0, START_CHECK_SD = 1, START_CHECK_FALLBACK = 2 };

const uint32_t FULL_RING_RECORDS  = 0x6C00;
const uint32_t FIVEK_RING_RECORDS = 0x800;
const uint32_t SD_RECORD_BYTES    = 16;
const uint32_t PS_RECORD_BYTES    = 9;
const uint32_t SD_WRITE_CHUNK_RECORDS    = 0x1000;
const uint32_t SD_5K_WRITE_CHUNK_RECORDS = 0x400;
const uint32_t LOG_HEADER_BYTES   = 512;

const uint32_t USB_PACKET_RECORDS = 64;
const uint32_t USB_PACKET_BUDGET  = 16;
const uint32_t USB_TEXT_RESERVE   = 64;

const uint32_t CMD_BUFFER_MAX = 39;

struct __attribute__((packed)) LogRecord {
  uint32_t cycles;
  uint32_t seq;
  uint16_t lc;
  uint16_t p1;
  uint16_t p2;
  uint16_t flags;   // bit0: 직전 데이터 누락, bit1: SD 기록 시작 또는 복구 직후
};

// ---------------------------------------------------------------------------
// DMA 버퍼 (RAM2). 배치 순서와 크기는 원본과 같다.
// ---------------------------------------------------------------------------
DMAMEM __attribute__((aligned(32))) uint32_t adcDmaBuf[(DMA_TOTAL_FRAMES * DMA_FRAME_BYTES) / 4];
DMAMEM __attribute__((aligned(32))) uint8_t sdProbeSector[512];
DMAMEM __attribute__((aligned(32))) uint8_t usbPacketBuf[USB_PACKET_RECORDS * PS_RECORD_BYTES];
DMAMEM __attribute__((aligned(32))) LogRecord fiveRing[FIVEK_RING_RECORDS];
DMAMEM __attribute__((aligned(32))) LogRecord fullRing[FULL_RING_RECORDS];

DMAChannel adcDma0;
DMAChannel adcDma1;
DMAChannel adcDma2;

// ---------------------------------------------------------------------------
// 전역 상태
// ---------------------------------------------------------------------------
volatile uint8_t storageMode = STORAGE_NONE;
volatile uint8_t recording = 0;
volatile uint8_t monitoring = 0;
volatile uint8_t storageHalted = 0;
uint8_t adcProfile = PROFILE_SPEC_SAFE;
volatile uint8_t dummyOn = 1;
volatile uint8_t adcRunning = 0;
volatile uint8_t adcActive = 0;
volatile uint8_t dmaHalf = 0;
uint8_t adcFallbackRun = 0;
uint8_t dummyOverride = 0;

// 최신 원시값과 FIR 출력
volatile uint16_t liveLc = 0, liveP1 = 0, liveP2 = 0;
volatile uint16_t filtLc = 0, filtP1 = 0, filtP2 = 0;
volatile uint32_t monitorSamples = 0;

// 프레임 시각과 번호
volatile uint32_t frameSeq = 0;
volatile uint32_t nextCycleTs = 0;
volatile uint32_t cyclesPerFrame = 1;
volatile uint32_t actualFrameHz = 0;
volatile uint32_t next5kCycle = 0;

// 64비트 사이클 확장
volatile uint8_t cyc64Valid = 0;
volatile uint32_t cycHi = 0;
volatile uint32_t lastIsrCycle = 0;
volatile uint32_t isrCycle = 0;
volatile uint32_t cyc64NowLo = 0;
volatile uint32_t cyc64NowHi = 0;

// FULL, 5K 링 버퍼
LogRecord isrBatch[DMA_HALF_FRAMES];
volatile uint32_t fullHead = 0, fullTail = 0, fullWr = 0;
volatile uint32_t fiveHead = 0, fiveTail = 0, fiveWr = 0;
volatile uint32_t fullQueued = 0, fiveQueued = 0;
volatile uint32_t fullMaxDepth = 0, fiveMaxDepth = 0;
volatile uint32_t fullDrops = 0, fiveDrops = 0;
volatile uint32_t fullDiscarded = 0, fiveDiscarded = 0;
volatile uint32_t fullExtraDrops = 0;
volatile uint8_t gapFlag = 0;
volatile uint8_t sdStartFlag = 0;

volatile uint64_t fiveScheduleMisses = 0;
volatile uint64_t fiveTimelineMissing = 0;
volatile uint32_t lastFiveTimelineUs = 0;
volatile uint8_t fiveTimelineValid = 0;

// 시간 기준
uint32_t sessionUs = 0;       // 사이클 카운터를 0으로 만든 시점의 micros()
uint32_t startMicros = 0;
uint32_t sessionStartMs = 0;

// 대체 경로 FIR
const uint32_t FIR_TAPS = 97;
const int16_t FIR_HALF[48] = {
    25,   -11,   -36,   -68,   -93,   -96,   -70,   -17,
    44,    89,    95,    53,   -23,  -101,  -141,  -118,
   -31,    86,   179,   198,   121,   -30,  -189,  -280,
  -245,   -82,   149,   343,   396,   258,   -33,  -358,
  -558,  -512,  -196,   282,   714,   872,   612,   -35,
  -844, -1450, -1477,  -680,   930,  3058,  5196,  6777
};
const int32_t FIR_CENTER = 7362;

uint16_t firLc[FIR_TAPS], firP1[FIR_TAPS], firP2[FIR_TAPS];
volatile uint8_t firIndex = 0;
volatile uint8_t firFill = 0;
volatile uint8_t firDecim = 0;
volatile uint8_t firResetRequest = 0;
volatile uint8_t firSettle = 0;
volatile uint32_t fbSampleUs = 0;
volatile uint32_t fbNextUs = 0;

// PSRAM
extern "C" uint8_t external_psram_size;
uint8_t* psramBuf = nullptr;
uint32_t psramBytes = 0;
volatile uint32_t psramCap = 0;
volatile uint32_t psramValid = 0;
volatile uint32_t psramWr = 0;
volatile uint64_t psramTotal = 0;
volatile uint64_t psramOverwritten = 0;
volatile uint8_t psramHasData = 0;
volatile uint8_t preserveGuard = 0;
volatile uint8_t preserveOverflow = 0;

// 대체 경로 계수기
volatile uint32_t etcErrorCount = 0;
volatile uint64_t etcErrorSpec = 0;
volatile uint64_t fbMissSession = 0;
volatile uint64_t fbScheduleMisses = 0;

// USB 이진 스트림
volatile uint64_t usbCursor = 0;
volatile uint64_t usbSent = 0;
volatile uint64_t usbDroppedTotal = 0;
volatile uint64_t usbDroppedOverflow = 0;
volatile uint64_t usbDropWrite = 0;
uint32_t usbMaxBacklog = 0;
uint32_t usbPacketSeq = 0;
uint32_t usbLastHeaderMs = 0;
uint8_t usbHeaderPending = 0;
uint8_t usbDrainPending = 0;

// SD
File fullFile;
File fiveFile;
char fullName[32] = "";
char fiveName[32] = "";
uint32_t sdFileSession = 0;
uint64_t fullBytes = 0;
uint64_t fiveBytes = 0;
uint32_t sdToFallbackCount = 0;
uint32_t sdToFallbackCount2 = 0;
uint32_t fallbackToSdCount = 0;
uint32_t safeCommitCount = 0;
uint32_t safeCommitDeferred = 0;
uint32_t lastFullWriteUs = 0, maxFullWriteUs = 0, lastFullWriteBytes = 0;
uint32_t lastFiveWriteUs = 0, maxFiveWriteUs = 0, lastFiveWriteBytes = 0;
uint32_t lastSeenFullDrops = 0, lastSeenFiveDrops = 0;

// SD 대기 상태와 복구
uint8_t sdIdleKnown = 0;
uint8_t sdIdlePresent = 0;
uint8_t sdIdleRawMisses = 0;
uint32_t sdIdleProbeMs = 0;
uint8_t sdPresentDuringFallback = 0;
uint8_t sdRecoveredFlag = 0;
uint8_t sdRecoveryBusy = 0;
uint8_t sdRecoveryAttempts = 0;
uint32_t sdRecoveryPollMs = 0;
uint32_t sdRecoveryAttemptMs = 0;
uint32_t sdLastRecoveryUs = 0;
uint32_t sdMaxRecoveryUs = 0;

// 점검 모드와 전원 재개
uint8_t checkDeleteFiles = 0;
uint8_t checkMode = 0;
uint8_t checkActive = 0;
uint8_t resumeAtBoot = 0;
uint8_t resumeArmed = 0;

// MAX_SPEED 자동 조정
uint8_t maxTuning = 0;
uint8_t maxTuneDone = 0;
uint8_t maxBackedOff = 0;
uint8_t maxStableCount = 0;
uint8_t maxStorageBackoffs = 0;
uint8_t maxUnstable = 0;
uint32_t maxTuneMs = 0;
uint32_t maxSeedHz = 0;
uint32_t maxProbeHz = 0;
uint32_t maxStableHz = 0;
uint32_t maxLastEtcCount = 0;
uint32_t maxLastFrameSeq = 0;

// 주기 타이머
uint32_t lastIdlePollMs = 0;
uint32_t lastMonitorSampleMs = 0;
uint32_t lastLiveMs = 0;
uint32_t lastDaqMs = 0;
uint32_t lastResumeTryMs = 0;
uint32_t monitorStartMs = 0;

// @DAQ 비율 계산용 직전 값
uint32_t daqLastSamples = 0;
uint32_t daqLastQueued = 0;
uint64_t daqLastPsramTotal = 0;
uint64_t daqLastFullBytes = 0;
uint32_t etcErrorAtStart = 0;

// 점검 기록 전에 보관하는 PSRAM 상태
struct PreserveBackup {
  uint32_t wr, valid;
  uint64_t total, overwritten, fbMisses, fbSession, usbDropped, usbOverflow, usbWrite, usbSentCount;
  uint32_t usbBacklog, sessionUs, frameSeq, fileSession;
  uint8_t hasData;
};
PreserveBackup preserveBackup;

char cmdBuf[CMD_BUFFER_MAX + 1];
uint8_t cmdLen = 0;

extern "C" volatile uint8_t usb_configuration;

inline uint8_t usbOpen() {
  return Serial ? 1 : 0;
}

// ---------------------------------------------------------------------------
// ADC 보정
// ---------------------------------------------------------------------------
static bool calibrateAdc(volatile uint32_t& cfg, volatile uint32_t& gc, volatile uint32_t& gs, volatile uint32_t& hc0) {
  hc0 = 0x1F;
  cfg = ADC_CFG_CALIBRATE;
  gc = 0x80;
  const uint32_t startUs = micros();
  do {
    if ((gc & 0x80U) == 0) {
      if (gs & 2U) {
        return false;
      }
      gc = 0;
      return true;
    }
  } while ((uint32_t)(micros() - startUs) < 0x4E21U);
  gc = 0;
  return false;
}

static void printBootInfo(bool psramOk, bool armedAtBoot) {
  Serial.println("# Teensy 4.1 LG07 TMS Logger v6.15 SD-fast/no-runtime-flush + 5K USB failover");
  Serial.println("@HELLO,device=TEENSY_DAQ,proto=1,fw=LG07_TMS_GS6_16_SPEC975K_MAX_SD_FAST_FAILOVER");
  Serial.print("# ADC_BITS=");
  Serial.println(12);
  Serial.print("# ENABLED_CHANNELS=");
  Serial.println(3);
  Serial.println("# ADC_ENGINE=DUAL_ADC1_ADC2_ADC_ETC_EDMA");
  Serial.print("# ADC_DIRECT_CLOCK_HZ=");
  Serial.println(ADC_DIRECT_CLOCK_HZ);
  Serial.print("# ADC_PROFILE_DEFAULT=");
  Serial.println(adcProfileName());
  Serial.print("# ADC_SPEC_FRAME_HZ=");
  Serial.println(SPEC_FRAME_HZ);
  Serial.print("# ADC_EDMA_FRAMES_PER_HALF=");
  Serial.println(DMA_HALF_FRAMES);
  Serial.println("# ADC_DMA_SOURCE=ADC_ETC_RESULT_32BIT_3CH_LINKED_ONE_REQUEST_PER_FRAME");
  Serial.print("# ADC_SPEC_RATED_SPS=");
  Serial.println(SPEC_RATED_SPS);
  Serial.print("# ADC_SPEC_TARGET_SPS=");
  Serial.println(SPEC_TARGET_SPS);
  Serial.println("# EXPECTED_SPEC_FRAME_HZ=243750; MAX_INITIAL_PROBE_HZ~=669642@ADCK37.5MHz; MAX_TARGET_HZ=0");
  Serial.print("# ADC_PROFILE_SPEC_SAFE=PACED,DUMMY_ON,FRAME_HZ=");
  Serial.println(SPEC_FRAME_HZ);
  Serial.print("# ADC_PROFILE_MAX_SPEED=AUTO_HW_LIMIT,DUMMY_OFF,ADC_ETC_RESULT_DMA,TARGET_HZ=0,INITIAL_PROBE_HZ=");
  Serial.println(MAX_INITIAL_HZ);
  Serial.println("# PINMAP=LOADCELL:A0/PIN14,PRESSURE1:A1/PIN15,PRESSURE2:A2/PIN16");
  Serial.print("# SD_RECORD_BYTES=");
  Serial.println(SD_RECORD_BYTES);
  Serial.print("# SD_5K_TARGET_HZ=");
  Serial.println(LOG_5K_HZ);
  Serial.println("# SD_RUNTIME_PERIODIC_FLUSH=OFF");
  Serial.print("# SD_WRITE_CHUNK_RECORDS=");
  Serial.println(SD_WRITE_CHUNK_RECORDS);
  Serial.print("# SD_FULL_DRAIN_TRIGGER_RECORDS=");
  Serial.println(SD_WRITE_CHUNK_RECORDS);
  Serial.print("# SD_5K_WRITE_CHUNK_RECORDS=");
  Serial.println(SD_5K_WRITE_CHUNK_RECORDS);
  Serial.print("# SD_5K_DRAIN_TRIGGER_RECORDS=");
  Serial.println(SD_5K_WRITE_CHUNK_RECORDS);
  Serial.print("# SD_FULL_SPILL_CAP_RECORDS=");
  Serial.println(0UL);
  Serial.print("# SD_5K_SPILL_CAP_RECORDS=");
  Serial.println(0UL);
  Serial.print("# FALLBACK_USB_LIVE_STORAGE=RAM1_DTCM,records=");
  Serial.println(psramCap);
  Serial.println("# SDIO_MODE=DMA_SDIO");
  Serial.println("# SD_NORMAL_PATH=ADC_EDMA_TO_INTERNAL_RAM2_TO_SD_ONLY; PSRAM=FALLBACK_ONLY");
  Serial.println("# SD_FAILURE_POLICY=SHORT_WRITE_TO_FALLBACK_20K_FIR5K_USB_SERIAL");
  Serial.println("# MAX_STORAGE_GOVERNOR=OFF");
  Serial.print("# PS_RECORD_BYTES=");
  Serial.println(PS_RECORD_BYTES);
  Serial.print("# FALLBACK_HZ=");
  Serial.println(LOG_5K_HZ);
  Serial.print("# FALLBACK_ADC_FRAME_HZ=");
  Serial.println(FALLBACK_ADC_HZ);
  Serial.print("# FALLBACK_DECIMATION_FACTOR=");
  Serial.println(4UL);
  Serial.println("# FALLBACK_FILTER=FIR97_EQUIRIPPLE_LINEAR_PHASE_Q15");
  Serial.println("# FALLBACK_FIR_PASSBAND_HZ=0..2000,RIPPLE_DB_PP~0.27");
  Serial.println("# FALLBACK_FIR_STOPBAND_HZ=2500..10000,ATTEN_DB>=~55");
  Serial.print("# FALLBACK_FIR_GROUP_DELAY_US=");
  Serial.println(FIR_GROUP_DELAY_US);
  Serial.print("# PSRAM_DETECTED_MIB=");
  Serial.println((unsigned long)external_psram_size);
  Serial.print("# PSRAM_OK=");
  Serial.println(psramOk ? 1 : 0);
  Serial.println("# Commands: START STOP MONITOR ADC_PROFILE_SPEC ADC_PROFILE_MAX ADC_PROFILE SD_RETRY PING STATUS CHECK_SELFTEST CHECK_SNAPSHOT CHECK_SD_START CHECK_FB_START CHECK_RESTORE_PSRAM CHECK_CLEAR_PSRAM DUMP PINIT");
  Serial.print("# ADC_PROFILE=");
  Serial.print(adcProfileName());
  Serial.print(",DUMMY=");
  Serial.println((int)dummyOn);
  Serial.print("# AUTO_SD_RECOVERY_ENABLE=");
  Serial.println(1UL);
  Serial.print("# SD_AUTO_REPROBE_COOLDOWN_MS=");
  Serial.println(2000UL);
  Serial.print("# IDLE_SD_POLL_INTERVAL_MS=");
  Serial.println(500UL);
  Serial.println("# IDLE_SD_REMOVAL_DETECT=RAW_SECTOR0_2X_CONFIRM");
  Serial.print("# IDLE_SD_STATUS_INTERVAL_MS=");
  Serial.println(500UL);
  Serial.print("# IDLE_SD_FRESH_PROBE_INTERVAL_MS=");
  Serial.println(2000UL);
  Serial.print("# POWERLOSS_AUTO_RESUME_ENABLE=");
  Serial.println(1);
  Serial.print("# RESUME_ARMED_AT_BOOT=");
  Serial.println(armedAtBoot ? 1 : 0);
  Serial.println("# SD_AUTO_REPROBE_POLICY=NO_BLIND_PROBE_WHILE_FALLBACK_STRONG_EVIDENCE_ONLY");
  Serial.println("# TIMELINE_META_VERSION=1");
  Serial.print("# FALLBACK_USB=LG5K_BINARY,RECORD_HZ=");
  Serial.println(LOG_5K_HZ);
  Serial.print("# FALLBACK_USB_PACKET_RECORDS=");
  Serial.println(USB_PACKET_RECORDS);
  Serial.print("# FALLBACK_USB_PACKET_BUDGET=");
  Serial.println(USB_PACKET_BUDGET);
  Serial.print("# FALLBACK_USB_TEXT_RESERVE_BYTES=");
  Serial.println(USB_TEXT_RESERVE);
  Serial.print("# FALLBACK_USB_SPOOL_RECORDS=");
  Serial.println(psramCap);
  Serial.println("# FALLBACK_USB_POLICY=3P28S_LIVE_BUFFER_2S_POST_PROBE_CATCHUP_DRAIN_ON_RECOVERY_NO_PSRAM_REPLAY");
  Serial.println("# FALLBACK_USB_QUEUE=DEDICATED_RAM1_DTCM_LIVE_RING");
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
  const uint32_t waitStartMs = millis();
  while (!Serial) {
    const uint32_t limitMs = usb_configuration ? 2000 : 750;
    if ((uint32_t)(millis() - waitStartMs) > limitMs) {
      break;
    }
    yield();
  }
  delay(300);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  pinMode(PIN_LOADCELL, INPUT_DISABLE);
  pinMode(PIN_PRESSURE1, INPUT_DISABLE);
  pinMode(PIN_PRESSURE2, INPUT_DISABLE);

  CCM_REG_CCGR1 |= 0x30300U;
  if (!calibrateAdc(ADC1_REG_CFG, ADC1_REG_GC, ADC1_REG_GS, ADC1_REG_HC0)) {
    Serial.println("# WARNING: ADC1_CALIBRATION_FAILED");
  }
  if (!calibrateAdc(ADC2_REG_CFG, ADC2_REG_GC, ADC2_REG_GS, ADC2_REG_HC0)) {
    Serial.println("# WARNING: ADC2_CALIBRATION_FAILED");
  }
  asm("dsb");
  delayMicroseconds(10);
  ETC_CTRL &= 0x3FFFFFFFU;
  asm("dsb");

  attachInterruptVector((IRQ_NUMBER_t)ADC_ETC_ERROR_IRQ, adcEtcErrorIsr);
  NVIC_SET_PRIORITY(ADC_ETC_ERROR_IRQ, 8);
  NVIC_ENABLE_IRQ(ADC_ETC_ERROR_IRQ);

  ADC1_REG_CFG = ADC_CFG_SOFTWARE;
  ADC1_REG_GC = 0;
  ADC1_REG_HC0 = 0x1F;
  dummyOn = (adcProfile == PROFILE_SPEC_SAFE) ? 1 : 0;
  dummyOverride = 0;

  const bool psramOk = psramReady();

  uint8_t armed = 0;
  const bool armedAtBoot = readResumeFlag(&armed) && armed == 1;
  resumeAtBoot = armedAtBoot ? 1 : 0;
  resumeArmed = resumeAtBoot;

  printBootInfo(psramOk, armedAtBoot);

  lastIdlePollMs = millis() - 500;
  lastDaqMs = millis() - 500;
  if (resumeAtBoot) {
    lastResumeTryMs = millis() - 1000;
    if (!recording && !monitoring) {
      autoResumeTry();
    }
  }
}

// MAX_SPEED 프로파일에서 끊김이 없는 최고 프레임 주파수를 찾는다.
static void serviceMaxTuning(uint32_t etcAtLoop, uint32_t previousEtc, uint32_t previousSeq, uint32_t hzAtLoop) {
  if (!maxTuning || maxTuneDone || !recording || adcProfile != PROFILE_MAX_SPEED || !adcRunning || actualFrameHz == 0) {
    return;
  }
  const uint32_t dtMs = millis() - maxTuneMs;
  if (dtMs <= 0x77) {
    return;
  }
  maxTuneMs = millis();
  const uint64_t expected = (uint64_t)actualFrameHz * dtMs;
  maxLastEtcCount = etcErrorCount;
  maxLastFrameSeq = frameSeq;

  auto backOff = [&]() {
    maxBackedOff = 1;
    maxStableCount = 0;
    uint32_t next = (uint32_t)(((uint64_t)hzAtLoop * 980U) / 1000U);
    if (next == 0) {
      next = 1;
    }
    if (!retuneAdc(next)) {
      maxTuning = 0;
      maxTuneDone = 1;
    } else {
      maxTuneMs = millis();
      maxLastEtcCount = etcErrorCount;
      maxLastFrameSeq = frameSeq;
      maxProbeHz = actualFrameHz;
    }
  };

  auto lockIn = [&]() {
    maxTuneDone = 1;
    maxTuning = 0;
    maxProbeHz = hzAtLoop;
  };

  auto stepUp = [&]() {
    maxStableCount = 0;
    uint32_t next = (uint32_t)(((uint64_t)hzAtLoop * 1010U + 999U) / 1000U);
    if (next <= hzAtLoop) {
      next = hzAtLoop + 1;
    }
    if (retuneAdc(next)) {
      maxProbeHz = actualFrameHz;
      if (hzAtLoop < actualFrameHz) {
        maxTuneMs = millis();
        maxLastEtcCount = etcErrorCount;
        maxLastFrameSeq = frameSeq;
        return;
      }
    }
    lockIn();
  };

  auto evaluate = [&]() {
    if (etcAtLoop != previousEtc) {
      backOff();
      return;
    }
    maxStableHz = hzAtLoop;
    const uint8_t count = maxStableCount;
    if (count == 0xFF) {
      if (!maxBackedOff) {
        stepUp();
      } else {
        lockIn();
      }
      return;
    }
    maxStableCount = count + 1;
    if (!maxBackedOff) {
      if ((uint8_t)(count + 1) != 1) {
        stepUp();
      }
    } else if ((uint8_t)(count + 1) > 2) {
      lockIn();
    }
  };

  if (expected <= 999) {
    evaluate();
    return;
  }

  const uint64_t ratio = ((uint64_t)(uint32_t)(frameSeq - previousSeq) * 1000U) / (expected / 1000U);
  if (ratio <= 0x4B0 && (ratio > 499 || etcAtLoop != previousEtc)) {
    if (ratio >= 0x3DE && ratio <= 0x3DE + 0x3C) {
      evaluate();
    } else {
      backOff();
    }
    return;
  }
  maxTuning = 0;
  maxTuneDone = 0;
  maxProbeHz = hzAtLoop;
  maxUnstable = 1;
}

static uint32_t daqPeriodMs() {
  if (recording) {
    return 1000;
  }
  return monitoring ? 1000 : 500;
}

void loop() {
  pollCommands();

  if (resumeAtBoot && !recording && !monitoring) {
    autoResumeTry();
  }

  const uint32_t previousSeq = maxLastFrameSeq;
  const uint32_t previousEtc = maxLastEtcCount;
  const uint32_t hzAtLoop = actualFrameHz;
  const uint32_t etcAtLoop = etcErrorCount;

  if (!recording) {
    usbStreamService();
    if (!monitoring && (uint32_t)(millis() - lastIdlePollMs) > 499) {
      lastIdlePollMs = millis();
      sdIdleProbe(false);
    }
    if (monitoring && (uint32_t)(millis() - lastMonitorSampleMs) > 99) {
      lastMonitorSampleMs = millis();
      sampleMonitor();
    }
    printLive();
    if ((uint32_t)(millis() - lastDaqMs) >= daqPeriodMs()) {
      printDaq(true);
    }
    return;
  }

  bool writeFailed = false;
  if (storageMode == STORAGE_SD) {
    serviceMaxTuning(etcAtLoop, previousEtc, previousSeq, hzAtLoop);

    if (lastSeenFullDrops != fullDrops || lastSeenFiveDrops != fiveDrops) {
      lastSeenFullDrops = fullDrops;
      lastSeenFiveDrops = fiveDrops;
      enterFallback("SD_QUEUE_OVERFLOW");
    }

    if (storageMode == STORAGE_SD) {
      for (uint8_t i = 12; i != 0; i--) {
        if ((uint32_t)(fullHead - fullTail) < SD_WRITE_CHUNK_RECORDS) {
          break;
        }
        if (!writeFullChunk()) {
          writeFailed = true;
          break;
        }
        if ((uint32_t)(fiveHead - fiveTail) >= SD_5K_WRITE_CHUNK_RECORDS && !writeFiveChunk()) {
          writeFailed = true;
          break;
        }
      }
      for (uint8_t i = 0; i < 2 && !writeFailed; i++) {
        if ((uint32_t)(fiveHead - fiveTail) < SD_5K_WRITE_CHUNK_RECORDS) {
          break;
        }
        if (!writeFiveChunk()) {
          writeFailed = true;
        }
      }
    }
    if (writeFailed) {
      enterFallback("SD_QUEUE_WRITE_FAILED");
    }
  }

  if (!checkDeleteFiles && !checkMode) {
    printLive();
    if ((uint32_t)(millis() - lastDaqMs) >= daqPeriodMs()) {
      printDaq(true);
    }
  }

  usbStreamService();
  if (recording && storageMode == STORAGE_FALLBACK && !checkMode) {
    sdRecoveryService(false);
  }
  if (storageHalted) {
    stopRecording(false);
  }
  usbStreamService();
}

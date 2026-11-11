// ADC, ADC_ETC, eDMA 설정과 프레임 처리.

// ADC_ETC 오류 인터럽트. 대체 경로에서는 FIR 상태를 다시 채우도록 표시한다.
FASTRUN void adcEtcErrorIsr() {
  etcErrorCount = etcErrorCount + 1;
  if (storageMode != STORAGE_FALLBACK) {
    if (adcProfile == PROFILE_SPEC_SAFE) {
      etcErrorSpec = etcErrorSpec + 1;
    }
  } else {
    fbMissSession = fbMissSession + 1;
    fbScheduleMisses = fbScheduleMisses + 1;
    firResetRequest = 1;
  }
  ETC_DONE2_ERR_IRQ = 0x110000;
  asm("dsb");
}

// 5K 예약 시각이 지난 표본의 타임라인 값을 계산하고 누락 슬롯을 센다.
FASTRUN static inline void noteFiveTimeline(uint32_t timelineUs) {
  if (fiveTimelineValid) {
    const uint32_t delta = timelineUs - lastFiveTimelineUs;
    if (delta - 0xFBU < 0x3938605U) {
      const uint32_t rounded = delta + 100U;
      if (rounded > 399U) {
        fiveTimelineMissing = fiveTimelineMissing + (rounded / 200U - 1U);
      }
    }
  }
  fiveTimelineValid = 1;
  lastFiveTimelineUs = timelineUs;
}

FASTRUN static inline uint32_t advanceFiveSchedule(uint32_t sampleCycle) {
  uint32_t missed = 0;
  next5kCycle = next5kCycle + CYCLES_PER_5K;
  while ((int32_t)(sampleCycle - next5kCycle) >= 0) {
    next5kCycle = next5kCycle + CYCLES_PER_5K;
    missed++;
  }
  fiveScheduleMisses = fiveScheduleMisses + missed;

  const uint64_t cycleNow = ((uint64_t)cyc64NowHi << 32) | cyc64NowLo;
  const uint64_t cycleAtSample = cycleNow - (uint32_t)(isrCycle - sampleCycle);
  return (sessionUs - startMicros) + (uint32_t)(cycleAtSample / 600U);
}

FASTRUN static inline void pushFive(const LogRecord& record) {
  uint32_t depth = fiveHead - fiveTail;
  if (depth < FIVEK_RING_RECORDS) {
    fiveRing[fiveWr] = record;
    uint32_t next = fiveWr + 1;
    if (next >= FIVEK_RING_RECORDS) {
      next = 0;
    }
    fiveWr = next;
    asm("dmb");
    fiveHead = fiveHead + 1;
    depth++;
    if (fiveMaxDepth < depth) {
      fiveMaxDepth = depth;
    }
    fiveQueued = fiveQueued + 1;
  } else {
    fiveDrops = fiveDrops + 1;
  }
}

FASTRUN static inline uint16_t consumeFlags() {
  uint16_t flags = 0;
  if (gapFlag) {
    gapFlag = 0;
    flags = 1;
  }
  if (sdStartFlag) {
    flags |= 2;
    sdStartFlag = 0;
  }
  return flags;
}

FASTRUN static inline uint16_t firFilter(const uint16_t* line, uint32_t oldest, uint32_t newest) {
  int32_t sum = 0;
  uint32_t a = oldest;
  uint32_t b = newest;
  for (uint32_t i = 0; i < 48; i++) {
    sum += (int32_t)FIR_HALF[i] * ((int32_t)line[a] + (int32_t)line[b]);
    a = (a + 1 == FIR_TAPS) ? 0 : a + 1;
    b = (b == 0) ? FIR_TAPS - 1 : b - 1;
  }
  sum += (int32_t)line[a] * FIR_CENTER;
  int32_t value = (sum + ((sum < 0) ? -0x4000 : 0x4000)) >> 15;
  if (value < 0) {
    value = 0;
  } else if (value > 4095) {
    value = 4095;
  }
  return (uint16_t)value;
}

FASTRUN static inline void storeFallbackSample(uint16_t lc, uint16_t p1, uint16_t p2) {
  if (firSettle) {
    firSettle = 0;
    return;
  }
  if (firResetRequest) {
    firFill = 0;
    firDecim = 0;
    firResetRequest = 0;
    firIndex = 0;
    return;
  }

  const uint32_t newest = firIndex;
  firLc[newest] = lc;
  firP1[newest] = p1;
  firP2[newest] = p2;
  uint32_t oldest = newest + 1;
  if (oldest >= FIR_TAPS) {
    oldest = 0;
  }
  firIndex = (uint8_t)oldest;

  if (firFill < FIR_TAPS) {
    firFill = firFill + 1;
    if (firFill < FIR_TAPS) {
      return;
    }
    firDecim = 0;
  }
  if (firDecim != 0) {
    firDecim = firDecim + 1;
    if (firDecim > 3) {
      firDecim = 0;
    }
    return;
  }

  const uint16_t outLc = firFilter(firLc, oldest, newest);
  const uint16_t outP1 = firFilter(firP1, oldest, newest);
  const uint16_t outP2 = firFilter(firP2, oldest, newest);
  filtLc = outLc;
  filtP1 = outP1;
  filtP2 = outP2;
  firDecim = 1;

  const uint32_t sampleUs = fbSampleUs - FIR_GROUP_DELAY_US;
  noteFiveTimeline(sampleUs - startMicros);

  if (preserveGuard && psramValid >= psramCap) {
    preserveOverflow = 1;
    storageHalted = 1;
    return;
  }

  uint8_t* slot = psramBuf + psramWr * PS_RECORD_BYTES;
  const uint32_t packed = (outLc & 0xFFFU) | ((uint32_t)(outP1 & 0xFFFU) << 12) | ((uint32_t)outP2 << 24);
  memcpy(slot, &sampleUs, 4);
  memcpy(slot + 4, &packed, 4);
  slot[8] = (uint8_t)((outP2 >> 8) & 0x0F);

  uint32_t next = psramWr + 1;
  if (next >= psramCap) {
    next = 0;
  }
  psramWr = next;
  if (psramValid < psramCap) {
    psramValid = psramValid + 1;
  } else {
    psramOverwritten = psramOverwritten + 1;
  }
  psramHasData = 1;
  asm("dmb");
  psramTotal = psramTotal + 1;
}

// 프레임 하나를 처리한다. 진입 시 SD 모드였으면 ISR 배치에 쌓고, 아니면 현재 모드대로 바로 처리한다.
FASTRUN static inline void processFrame(uint16_t lc, uint16_t p1, uint16_t p2, uint8_t entryMode, uint32_t& batchCount) {
  const uint32_t sampleCycle = nextCycleTs;
  const uint32_t followingCycle = sampleCycle + cyclesPerFrame;

  if (entryMode == STORAGE_SD) {
    LogRecord& r = isrBatch[batchCount];
    r.cycles = sampleCycle;
    nextCycleTs = followingCycle;
    r.seq = frameSeq;
    frameSeq = frameSeq + 1;
    r.lc = lc;
    r.p1 = p1;
    r.p2 = p2;
    r.flags = consumeFlags();
    batchCount++;
    if ((int32_t)(sampleCycle - next5kCycle) >= 0) {
      noteFiveTimeline(advanceFiveSchedule(sampleCycle));
      pushFive(r);
    }
    return;
  }

  liveLc = lc;
  liveP1 = p1;
  liveP2 = p2;
  nextCycleTs = followingCycle;
  if (storageMode == STORAGE_FALLBACK) {
    fbSampleUs = fbNextUs;
    fbNextUs = fbNextUs + FALLBACK_US_STEP;
  }

  if (storageMode == STORAGE_SD) {
    const uint32_t seq = frameSeq;
    const uint16_t flags = consumeFlags();
    uint32_t depth = fullHead - fullTail;
    if (depth < FULL_RING_RECORDS) {
      LogRecord& r = fullRing[fullWr];
      r.cycles = sampleCycle;
      r.seq = seq;
      r.lc = lc;
      r.p1 = p1;
      r.p2 = p2;
      r.flags = flags;
      uint32_t next = fullWr + 1;
      if (next >= FULL_RING_RECORDS) {
        next = 0;
      }
      fullWr = next;
      asm("dmb");
      fullHead = fullHead + 1;
      depth++;
      if (fullMaxDepth < depth) {
        fullMaxDepth = depth;
      }
      fullQueued = fullQueued + 1;
    } else {
      fullDrops = fullDrops + 1;
      gapFlag = 1;
    }
    frameSeq = seq + 1;
    if ((int32_t)(sampleCycle - next5kCycle) >= 0) {
      noteFiveTimeline(advanceFiveSchedule(sampleCycle));
      LogRecord five;
      five.cycles = sampleCycle;
      five.seq = seq;
      five.lc = lc;
      five.p1 = p1;
      five.p2 = p2;
      five.flags = flags;
      pushFive(five);
    }
    return;
  }

  frameSeq = frameSeq + 1;
  if (storageMode == STORAGE_FALLBACK) {
    storeFallbackSample(lc, p1, p2);
  }
}

// ISR 배치를 FULL 링으로 옮긴다.
FASTRUN static inline void flushIsrBatch(uint32_t count) {
  if (count == 0) {
    return;
  }
  const uint32_t wr = fullWr;
  const uint32_t tail = fullTail;
  const uint32_t head = fullHead;
  uint32_t copied = 0;
  if ((uint32_t)(head - tail) < FULL_RING_RECORDS) {
    const uint32_t space = (tail + FULL_RING_RECORDS) - head;
    copied = (count <= space) ? count : space;
    uint32_t first = FULL_RING_RECORDS - wr;
    if (copied <= first) {
      first = copied;
    }
    memcpy(&fullRing[wr], isrBatch, first * sizeof(LogRecord));
    if (copied - first != 0) {
      memcpy(&fullRing[0], &isrBatch[first], (copied - first) * sizeof(LogRecord));
    }
    uint32_t next = wr + copied;
    if (next >= FULL_RING_RECORDS) {
      next -= FULL_RING_RECORDS;
    }
    fullWr = next;
    asm("dmb");
    fullHead = head + copied;
    if (fullMaxDepth < (uint32_t)(fullHead - tail)) {
      fullMaxDepth = fullHead - tail;
    }
    if (space < count) {
      fullDrops = (fullDrops + count) - copied;
      gapFlag = 1;
    }
  } else {
    fullDrops = fullDrops + count;
    gapFlag = 1;
  }
  fullQueued = fullQueued + copied;
}

// DMA 버퍼의 프레임을 처리한다. 더미 변환 사용 여부에 따라 값의 위치가 다르다.
FASTRUN static void processBlock(const uint32_t* block, uint32_t frames, uint8_t entryMode) {
  uint32_t batch = 0;
  uint16_t lc = 0, p1 = 0, p2 = 0;
  for (uint32_t i = 0; i < frames; i++) {
    const uint32_t* f = block + i * 3;
    if (!dummyOn) {
      lc = (uint16_t)(f[0] & 0xFFFU);
      p1 = (uint16_t)((f[0] >> 16) & 0xFFFU);
      p2 = (uint16_t)(f[1] & 0xFFFU);
    } else {
      lc = (uint16_t)((f[0] >> 16) & 0xFFFU);
      p1 = (uint16_t)((f[1] >> 16) & 0xFFFU);
      p2 = (uint16_t)((f[2] >> 16) & 0xFFFU);
    }
    processFrame(lc, p1, p2, entryMode, batch);
  }
  if (entryMode == STORAGE_SD) {
    liveLc = lc;
    liveP1 = p1;
    liveP2 = p2;
    flushIsrBatch(batch);
  }
}

// 64비트 사이클 값을 ISR 시작 시점으로 갱신한다.
FASTRUN static inline void updateCycle64() {
  const uint32_t cyc = ARM_DWT_CYCCNT;
  isrCycle = cyc;
  if (!cyc64Valid) {
    cycHi = 0;
    cyc64Valid = 1;
  } else if (cyc < lastIsrCycle) {
    cycHi = cycHi + 1;
  }
  cyc64NowLo = cyc;
  cyc64NowHi = cycHi;
  lastIsrCycle = cyc;
}

// 반 버퍼 완료 인터럽트
FASTRUN void adcDmaIsr() {
  const uint8_t entryMode = storageMode;
  updateCycle64();
  DMA_CINT = adcDma2.channel;
  const uint8_t half = dmaHalf ? 1 : 0;
  if (adcRunning && recording && !storageHalted) {
    const uint32_t* block = adcDmaBuf + (half ? (DMA_HALF_FRAMES * DMA_FRAME_BYTES) / 4 : 0);
    arm_dcache_delete((void*)block, DMA_HALF_FRAMES * DMA_FRAME_BYTES);
    dmaHalf = half ^ 1;
    processBlock(block, DMA_HALF_FRAMES, entryMode);
  } else {
    dmaHalf = half ^ 1;
  }
}

// 중지 직전에 아직 처리하지 않은 프레임을 DMA 위치로 찾아 처리한다.
static void drainPartialDma() {
  const uint8_t entryMode = storageMode;
  updateCycle64();
  const uintptr_t daddr = (uintptr_t)adcDma2.TCD->DADDR;
  const uintptr_t base = (uintptr_t)adcDmaBuf;
  const uintptr_t limit = base + DMA_TOTAL_FRAMES * DMA_FRAME_BYTES;
  if (daddr < base || daddr > limit || daddr < base + 8) {
    return;
  }
  const uint32_t offset = (uint32_t)(daddr - (base + 8));
  if (offset >= 0xC0C) {
    return;
  }
  uint32_t frames = offset / DMA_FRAME_BYTES;
  const uint32_t* halfB = adcDmaBuf + (DMA_HALF_FRAMES * DMA_FRAME_BYTES) / 4;

  auto runFrames = [&](const uint32_t* start, uint32_t count) {
    arm_dcache_delete((void*)start, count * DMA_FRAME_BYTES);
    uint32_t done = 0;
    while (done < count) {
      const uint32_t batch = (count - done > DMA_HALF_FRAMES) ? DMA_HALF_FRAMES : count - done;
      processBlock(start + done * 3, batch, entryMode);
      done += batch;
    }
  };

  if (dmaHalf == 0) {
    if (offset < DMA_FRAME_BYTES) {
      return;
    }
    runFrames(adcDmaBuf, frames);
  } else if (offset < DMA_HALF_FRAMES * DMA_FRAME_BYTES) {
    runFrames(halfB, DMA_HALF_FRAMES);
    if (offset >= DMA_FRAME_BYTES) {
      runFrames(adcDmaBuf, frames);
    }
  } else {
    frames -= DMA_HALF_FRAMES;
    if (frames != 0) {
      runFrames(halfB, frames);
    }
  }
}

static void xbarConnect(uint32_t input, uint32_t output) {
  if (output < 132 && input < 88) {
    volatile uint16_t& reg = REG16(0x403BC000U + (output & ~1U));
    uint16_t value = reg;
    if (output & 1U) {
      value = (value & 0x00FFU) | (uint16_t)(input << 8);
    } else {
      value = (uint16_t)input | (value & 0xFF00U);
    }
    reg = value;
  }
}

// FlexPWM 트리거 시작. 프레임 주기와 타임스탬프 간격을 함께 계산한다.
static bool startAdcTrigger(uint32_t hz) {
  CCM_REG_CCGR4 |= 0xC00000U;
  CCM_REG_CCGR2 |= 0xC00000U;
  xbarConnect(54, 103);
  PWM4_MCTRL &= (uint16_t)~PWM4_MCTRL_RUN_SM2;
  PWM4_SM2_TCTRL = 0;
  asm("dsb");

  uint32_t halfPeriod = (uint32_t)(((uint64_t)F_BUS_ACTUAL + hz) / ((uint64_t)hz * 2U));
  if (halfPeriod > 0x7FFE) {
    halfPeriod = 0x7FFF;
  }
  if (halfPeriod < 4) {
    halfPeriod = 4;
  }
  PWM4_SM2_CTRL2 = 0xE000;
  PWM4_SM2_CTRL = 0;
  PWM4_SM2_INIT = (uint16_t)(-(int16_t)halfPeriod);
  PWM4_SM2_VAL1 = (uint16_t)(halfPeriod - 1);
  PWM4_SM2_VAL0 = 0;
  PWM4_SM2_TCTRL = 2;
  PWM4_MCTRL |= 0x44;

  actualFrameHz = F_BUS_ACTUAL / (halfPeriod * 2U);
  cyclesPerFrame = (uint32_t)((((uint64_t)halfPeriod * 2U) * CPU_HZ_NOMINAL + (F_BUS_ACTUAL >> 1)) / F_BUS_ACTUAL);
  if (cyclesPerFrame == 0) {
    cyclesPerFrame = 1;
  }
  nextCycleTs = ARM_DWT_CYCCNT + cyclesPerFrame;
  PWM4_MCTRL |= PWM4_MCTRL_RUN_SM2;
  return true;
}

// 트리거를 멈추고 진행 중인 변환이 끝나기를 기다린다.
static void stopTriggerHard() {
  PWM4_MCTRL &= (uint16_t)~PWM4_MCTRL_RUN_SM2;
  PWM4_SM2_TCTRL = 0;
  asm("dsb");
  DMA_CERQ = adcDma2.channel;
  ETC_DMA_CTRL = (ETC_DMA_CTRL & 0xFF00FF00U) | 0x00FF0000U;
  asm("dsb");
  ETC_DONE0_1_IRQ = 0xFFFFFFFFU;
  ETC_DONE2_ERR_IRQ = 0xFFFFFFFFU;
  asm("dsb");
  delayMicroseconds(1);
  ETC_CTRL &= 0x3FFFFF00U;
  asm("dsb");

  maxTuning = 0;
  dmaHalf = 0;
  maxTuneDone = 0;
  adcActive = 0;
  maxBackedOff = 0;
  maxStableCount = 0;
  maxUnstable = 0;
  maxTuneMs = 0;
  maxLastEtcCount = 0;
  maxLastFrameSeq = 0;
  maxSeedHz = 0;
  maxStableHz = 0;
  maxProbeHz = 0;
}

// 3개의 eDMA 채널을 연결해 프레임마다 결과 레지스터 3개를 버퍼로 옮긴다.
static void setupAdcDma() {
  const uint32_t resultB = dummyOn ? ETC_TRIG0_RES32_ADDR : ETC_TRIG4_RES10_ADDR;
  const uint32_t resultC = dummyOn ? ETC_TRIG4_RES10_ADDR : ETC_TRIG0_RES32_ADDR;

  auto program = [](DMAChannel& ch, uint32_t source, uint32_t* dest) {
    ch.TCD->CSR = 0;
    ch.TCD->SADDR = (volatile const void*)(uintptr_t)source;
    ch.TCD->SOFF = 0;
    ch.TCD->ATTR = 0x0202;
    ch.TCD->NBYTES = 4;
    ch.TCD->SLAST = 0;
    ch.TCD->DADDR = (volatile void*)dest;
    ch.TCD->DOFF = DMA_FRAME_BYTES;
    ch.TCD->CITER = DMA_TOTAL_FRAMES;
    ch.TCD->BITER = DMA_TOTAL_FRAMES;
    ch.TCD->DLASTSGA = -(int32_t)(DMA_TOTAL_FRAMES * DMA_FRAME_BYTES);
  };

  program(adcDma0, ETC_TRIG0_RES10_ADDR, adcDmaBuf);
  program(adcDma1, resultB, adcDmaBuf + 1);

  DMA_CERQ = adcDma2.channel;
  DMA_CINT = adcDma2.channel;
  DMA_CDNE = adcDma2.channel;
  DMA_CERR = adcDma2.channel;
  program(adcDma2, resultC, adcDmaBuf + 2);
  adcDma2.TCD->CSR |= DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
  adcDma2.attachInterrupt(adcDmaIsr, 16);

  volatile uint32_t* mux = &DMAMUX_CHCFG0;
  mux[adcDma1.channel] = 0;
  mux[adcDma2.channel] = 0;

  // 첫 채널이 다음 채널을 호출하도록 소규모, 주 루프 링크를 설정한다.
  adcDma0.TCD->CITER = (uint16_t)((adcDma0.TCD->CITER & 0xC1FFU) | ((adcDma1.channel & 0x1FU) << 9) | 0x8000U);
  adcDma0.TCD->BITER = adcDma0.TCD->CITER;
  adcDma0.TCD->CSR = (uint16_t)(((adcDma1.channel & 0x1FU) << 8) | (adcDma0.TCD->CSR & 0xE07FU) | 0x20U);
  adcDma1.TCD->CITER = (uint16_t)((adcDma1.TCD->CITER & 0xC1FFU) | ((adcDma2.channel & 0x1FU) << 9) | 0x8000U);
  adcDma1.TCD->BITER = adcDma1.TCD->CITER;
  adcDma1.TCD->CSR = (uint16_t)(((adcDma2.channel & 0x1FU) << 8) | (adcDma1.TCD->CSR & 0xE07FU) | 0x20U);

  mux[adcDma0.channel] = 0;
  mux[adcDma0.channel] = 0x80000000U | DMAMUX_SRC_ADC_ETC;

  dmaHalf = 0;
  asm("dsb");
  arm_dcache_delete(adcDmaBuf, sizeof(adcDmaBuf));
  asm("dsb");
  asm("isb");
  DMA_SERQ = adcDma0.channel;
}

// 수집을 시작한다. 프레임 주파수, 더미 변환 사용, 대체 경로 시작 여부를 받는다.
static void startAdc(uint32_t hz, bool dummy, bool fallback) {
  stopTriggerHard();
  ADC1_REG_CFG = ADC_CFG_HARDWARE;
  ADC1_REG_GC = 0;
  ADC1_REG_HC0 = 0x10;
  ADC2_REG_CFG = ADC_CFG_HARDWARE;
  ADC2_REG_GC = 0;
  ADC2_REG_HC0 = 0x10;
  dummyOn = dummy ? 1 : 0;
  dummyOverride = 0;

  PWM4_MCTRL &= (uint16_t)~PWM4_MCTRL_RUN_SM2;
  PWM4_SM2_TCTRL = 0;
  asm("dsb");
  asm("dsb");
  ETC_DONE0_1_IRQ = 0xFFFFFFFFU;
  ETC_DONE2_ERR_IRQ = 0xFFFFFFFFU;

  const uint32_t chA = (0x1010U | ADC_CH_LOADCELL) & 0xFFFFU;
  const uint32_t chB = (0x1010U | ADC_CH_PRESSURE1) & 0xFFFFU;
  const uint32_t chC = (0x1010U | ADC_CH_PRESSURE2) & 0xFFFFU;
  if (!dummy) {
    ETC_TRIG0_CTRL = 0x10100;
    ETC_TRIG4_CTRL = 0x10000;
    ETC_TRIG0_CHAIN10 = (chB << 16) | chA;
    ETC_TRIG4_CHAIN10 = chC;
    ETC_TRIG0_CHAIN32 = 0;
  } else {
    ETC_TRIG0_CTRL = 0x10300;
    ETC_TRIG4_CTRL = 0x10100;
    ETC_TRIG0_CHAIN10 = (chA << 16) | chA;
    ETC_TRIG0_CHAIN32 = (chB << 16) | chB;
    ETC_TRIG4_CHAIN10 = (chC << 16) | chC;
  }
  ETC_TRIG4_CHAIN76 = 0;
  ETC_DMA_CTRL = (ETC_DMA_CTRL & 0xFF00FF00U) | 1U;
  asm("dsb");
  ETC_CTRL = (ETC_CTRL & 0xFFFFFF00U) | 0x20000001U;
  asm("dsb");
  ETC_TRIG0_CHAIN54 = 0;
  ETC_TRIG0_CHAIN76 = 0;
  ETC_TRIG4_CHAIN32 = 0;
  ETC_TRIG4_CHAIN54 = 0;

  setupAdcDma();
  adcFallbackRun = fallback ? 1 : 0;
  adcActive = 1;
  adcRunning = 1;
  startAdcTrigger(hz);
  if (fallback) {
    fbNextUs = micros() + FALLBACK_US_STEP;
  }
}

// 수집 중 주파수만 바꾼다. MAX_SPEED 자동 조정에서 사용한다.
static bool retuneAdc(uint32_t hz) {
  if (hz == 0) {
    return false;
  }
  PWM4_MCTRL &= (uint16_t)~PWM4_MCTRL_RUN_SM2;
  PWM4_SM2_TCTRL = 0;
  asm("dsb");
  while (ADC1_REG_GS & 1U) {
  }
  while (ADC2_REG_GS & 1U) {
  }
  noInterrupts();
  DMA_CERQ = adcDma2.channel;
  DMA_CINT = adcDma2.channel;
  interrupts();
  if (adcRunning) {
    drainPartialDma();
  }
  setupAdcDma();
  return startAdcTrigger(hz);
}

static void startFallbackAdc() {
  noInterrupts();
  firIndex = 0;
  firFill = 0;
  firDecim = 0;
  filtLc = liveLc;
  filtP1 = liveP1;
  filtP2 = liveP2;
  interrupts();
  fbMissSession = 0;
  firResetRequest = 0;
  firSettle = 0;
  startAdc(FALLBACK_ADC_HZ, true, true);
}

// 현재 프로파일로 수집을 시작한다.
static bool startProfileAdc() {
  const uint8_t profile = adcProfile;
  dummyOn = (profile == PROFILE_SPEC_SAFE) ? 1 : 0;
  dummyOverride = 0;
  if (profile != PROFILE_MAX_SPEED) {
    startAdc(SPEC_FRAME_HZ, dummyOn != 0, false);
    return true;
  }

  startAdc(MAX_INITIAL_HZ, false, false);
  maxSeedHz = actualFrameHz;
  maxProbeHz = actualFrameHz;
  maxStableHz = 0;
  maxStableCount = 0;
  maxBackedOff = 0;
  maxUnstable = 0;
  maxStorageBackoffs = 0;
  maxTuneMs = millis();
  maxLastEtcCount = etcErrorCount;
  maxLastFrameSeq = frameSeq;
  maxTuning = profile;
  maxTuneDone = 0;
  return true;
}

// 수집을 멈추고 남은 프레임을 처리한다.
static void stopAdc() {
  PWM4_MCTRL &= (uint16_t)~PWM4_MCTRL_RUN_SM2;
  PWM4_SM2_TCTRL = 0;
  asm("dsb");
  if (adcRunning) {
    while (ADC1_REG_GS & 1U) {
    }
    while (ADC2_REG_GS & 1U) {
    }
    delayMicroseconds(1);
    DMA_CINT = adcDma2.channel;
    DMA_CERQ = adcDma2.channel;
    if (adcRunning) {
      drainPartialDma();
    }
  }
  adcRunning = 0;
  ADC1_REG_GC = 0;
  ADC1_REG_HC0 = 0x1F;
  ADC2_REG_GC = 0;
  ADC2_REG_HC0 = 0x1F;
  adcActive = 0;
  DMA_CERQ = adcDma2.channel;
  stopTriggerHard();
}

// 모니터 모드에서 3개 채널을 소프트웨어 변환으로 읽는다.
static uint16_t readAdc1Blocking(uint32_t channel) {
  ADC1_REG_HC0 = channel;
  while ((ADC1_REG_HS & 1U) == 0) {
  }
  return (uint16_t)(ADC1_REG_R0 & 0xFFFU);
}

static void sampleMonitor() {
  if (dummyOn) {
    (void)readAdc1Blocking(ADC_CH_LOADCELL);
  }
  liveLc = readAdc1Blocking(ADC_CH_LOADCELL);
  if (dummyOn) {
    (void)readAdc1Blocking(ADC_CH_PRESSURE1);
  }
  liveP1 = readAdc1Blocking(ADC_CH_PRESSURE1);
  if (dummyOn) {
    (void)readAdc1Blocking(ADC_CH_PRESSURE2);
  }
  liveP2 = readAdc1Blocking(ADC_CH_PRESSURE2);
  monitorSamples = monitorSamples + 1;
}

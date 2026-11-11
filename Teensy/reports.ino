// @LIVE, @DAQ, @CHECK 출력.

static inline float rawToEngineering(uint16_t raw, float factor) {
  return (((float)raw * ADC_VREF) / 4095.0f) * factor;
}

// 10 Hz 실시간 값.
static void printLive() {
  const uint32_t nowMs = millis();
  if (!recording && !monitoring) {
    return;
  }
  if ((uint32_t)(nowMs - lastLiveMs) < 100) {
    return;
  }
  if (!usbOpen() || (uint32_t)Serial.availableForWrite() < 0x200) {
    return;
  }
  lastLiveMs = nowMs;

  const uint8_t mode = storageMode;
  noInterrupts();
  const bool isFallback = (storageMode == STORAGE_FALLBACK);
  uint16_t lc = liveLc, p1 = liveP1, p2 = liveP2;
  uint64_t total = 0, drop = 0, drop5k = 0;
  uint32_t valid = 0, cap = 0, fiveCount = 0;
  if (recording && isFallback) {
    if (firFill >= FIR_TAPS) {
      lc = filtLc;
      p1 = filtP1;
      p2 = filtP2;
    }
    total = psramTotal;
    drop = (uint64_t)fullDrops + fullDiscarded + fullExtraDrops;
    drop5k = (uint64_t)fiveDrops + fiveDiscarded + fiveTimelineMissing;
    valid = psramValid;
    cap = psramCap;
    fiveCount = fiveQueued;
  }
  interrupts();

  uint32_t elapsedMs = 0;
  if (recording) {
    elapsedMs = nowMs - sessionStartMs;
  } else if (monitoring) {
    elapsedMs = nowMs - monitorStartMs;
  }

  Serial.print("@LIVE,t=");
  Serial.print((float)elapsedMs / 1000.0f, 3);
  Serial.print(",LC=");
  Serial.print(rawToEngineering(lc, LC_K), 3);
  Serial.print(",P1=");
  Serial.print(rawToEngineering(p1, P1_K), 3);
  Serial.print(",P2=");
  Serial.print(rawToEngineering(p2, P2_K), 3);
  Serial.print(",mode=");
  if (!recording) {
    Serial.print(monitoring ? "MONITOR" : "IDLE");
  } else {
    Serial.print(storageModeName(mode));
  }
  Serial.print(",recording=");
  Serial.print((int)recording);
  Serial.print(",monitor=");
  Serial.print((int)monitoring);
  Serial.print(",resume_armed=");
  Serial.print((int)resumeArmed);
  Serial.print(",auto_resume_pending=");
  Serial.print((int)resumeAtBoot);
  Serial.print(",dummy=");
  Serial.print((int)dummyOn);
  Serial.print(",adc_profile=");
  Serial.print(adcProfileName());

  if (recording && isFallback) {
    Serial.print(",acq_hz=");
    Serial.print(0.0f, 1);
    Serial.print(",log_hz=");
    Serial.print(0.0f, 1);
    Serial.print(",psram_records=");
    Serial.print(valid);
    Serial.print(",psram_cap=");
    Serial.print(cap);
    Serial.print(",records=");
    Serial.print((uint64_t)fullQueued + total);
    Serial.print(",5k_records=");
    Serial.print((uint64_t)fiveCount + total);
    Serial.print(",drop=");
    Serial.print(drop);
    Serial.print(",drop5k=");
    Serial.print(drop5k);
    Serial.print(",ring=0,ring5k=0");
    Serial.print(",usb5k_queued=");
    noInterrupts();
    uint32_t queued = 0;
    if (psramTotal > usbCursor && psramValid != 0) {
      const uint64_t backlog = psramTotal - usbCursor;
      queued = (backlog > psramValid) ? psramValid : (uint32_t)backlog;
    }
    interrupts();
    Serial.print(queued);
    Serial.print(",usb5k_cap=");
    Serial.print(psramCap);
    Serial.print(",usb5k_drop=");
    Serial.print((uint64_t)usbDroppedTotal);
    Serial.print(",usb5k_drop_overflow=");
    Serial.print((uint64_t)usbDroppedOverflow);
    Serial.print(",usb5k_drop_write=");
    Serial.print((uint64_t)usbDropWrite);
  }
  Serial.println();
}

// 주기 상태 줄. includeSd가 false면 ",sd=" 항목을 뺀다.
static void printDaq(bool includeSd) {
  if (!usbOpen() || (uint32_t)Serial.availableForWrite() < 0x300) {
    return;
  }

  const uint8_t mode = storageMode;
  const uint32_t nowMs = millis();
  const uint32_t fullDepth = fullHead - fullTail;
  const uint32_t fiveDepth = fiveHead - fiveTail;
  const uint32_t valid = psramValid;
  const uint32_t cap = psramCap;
  const uint32_t queued = fullQueued;
  const uint32_t fiveCount = fiveQueued;
  const uint64_t bytesNow = fullBytes;
  const uint64_t totalNow = psramTotal;

  noInterrupts();
  const bool isFallback = (storageMode == STORAGE_FALLBACK);
  uint16_t lc = liveLc, p1 = liveP1, p2 = liveP2;
  if (recording && isFallback && firFill > 0x60) {
    lc = filtLc;
    p1 = filtP1;
    p2 = filtP2;
  }
  const uint32_t samples = monitoring ? monitorSamples : frameSeq;
  const uint64_t drop = (uint64_t)fullDrops + fullDiscarded + fullExtraDrops;
  const uint64_t drop5k = (uint64_t)fiveDrops + fiveDiscarded + fiveTimelineMissing;
  interrupts();

  uint32_t elapsedMs = 0;
  if (recording) {
    elapsedMs = millis() - sessionStartMs;
  } else if (monitoring) {
    elapsedMs = millis() - monitorStartMs;
  }

  const uint32_t dtMs = millis() - lastDaqMs;
  float acqHz = 0.0f, logHz = 0.0f, enqueueHz = 0.0f;
  if (dtMs != 0 && (recording || monitoring)) {
    acqHz = ((float)(samples - daqLastSamples) * 1000.0f) / (float)dtMs;
  }
  if (dtMs != 0 && recording) {
    if (storageMode == STORAGE_SD) {
      enqueueHz = ((float)(queued - daqLastQueued) * 1000.0f) / (float)dtMs;
      logHz = ((float)((bytesNow - daqLastFullBytes) >> 4) * 1000.0f) / (float)dtMs;
    } else if (storageMode == STORAGE_FALLBACK) {
      enqueueHz = ((float)(totalNow - daqLastPsramTotal) * 1000.0f) / (float)dtMs;
      logHz = enqueueHz;
    }
  }

  int sdState = -1;
  if (includeSd && !monitoring) {
    if (!recording) {
      sdState = sdIdleKnown ? sdIdlePresent : (sdIdleProbe(true) ? 1 : 0);
    } else if (storageMode == STORAGE_FALLBACK) {
      sdState = sdPresentDuringFallback;
    } else {
      sdState = SD.mediaPresent() ? 1 : 0;
    }
  }

  Serial.print("@DAQ,device=TEENSY_DAQ");
  Serial.print(",recording=");
  Serial.print((int)recording);
  Serial.print(",monitor=");
  Serial.print((int)monitoring);
  Serial.print(",mode=");
  if (!recording) {
    Serial.print(monitoring ? "MONITOR" : "IDLE");
  } else {
    Serial.print(storageModeName(mode));
  }
  if (includeSd) {
    Serial.print(",sd=");
    Serial.print(sdState);
  }
  Serial.print(",t=");
  Serial.print((float)elapsedMs / 1000.0f, 3);
  Serial.print(",LC=");
  Serial.print(rawToEngineering(lc, LC_K), 3);
  Serial.print(",P1=");
  Serial.print(rawToEngineering(p1, P1_K), 3);
  Serial.print(",P2=");
  Serial.print(rawToEngineering(p2, P2_K), 3);
  Serial.print(",acq_hz=");
  Serial.print(acqHz, 1);
  Serial.print(",log_hz=");
  Serial.print(logHz, 1);
  Serial.print(",enqueue_hz=");
  Serial.print(enqueueHz, 1);
  Serial.print(",adc_profile=");
  Serial.print(adcProfileName());
  Serial.print(",dummy=");
  Serial.print((int)dummyOn);
  Serial.print(",adc_engine=DUAL_ADC_EDMA");
  Serial.print(",cpu_temp_c=");
  Serial.print(tempmonGetTemp(), 1);
  Serial.print(",drop=");
  Serial.print(drop);
  Serial.print(",adc_miss=");
  Serial.print((uint64_t)etcErrorSpec);
  Serial.print(",etc_err=");
  Serial.print(etcErrorCount - etcErrorAtStart);
  Serial.print(",etc_err_total=");
  Serial.print(etcErrorCount);
  Serial.print(",dma_es=");
  Serial.print(DMA_ES);
  Serial.print(",dma_req_flags=");
  Serial.print((ETC_DMA_CTRL & 0xFFFFFFU) >> 16);
  Serial.print(",dma0_citer=");
  Serial.print((uint32_t)(adcDma0.TCD->CITER & 0x7FFFU));
  Serial.print(",dma1_citer=");
  Serial.print((uint32_t)(adcDma1.TCD->CITER & 0x7FFFU));
  Serial.print(",dma2_citer=");
  Serial.print((uint32_t)(adcDma2.TCD->CITER & 0x7FFFU));
  Serial.print(",adc_run=");
  Serial.print((int)adcRunning);
  Serial.print(",trigger=FLEXPWM_XBAR");
  Serial.print(",dma_source=ADC_ETC_RESULT32_LINKED3");
  Serial.print(",trigger_hz=");
  Serial.print(actualFrameHz);
  Serial.print(",adc_clk_hz=");
  Serial.print(ADC_DIRECT_CLOCK_HZ);
  Serial.print(",adc_conv_cap_sps=");
  Serial.print(1339285UL);
  Serial.print(",profile_target_hz=");
  Serial.print(adcProfile ? 0UL : (unsigned long)SPEC_FRAME_HZ);

  if (adcProfile == PROFILE_MAX_SPEED) {
    Serial.print(",max_tune=");
    if (maxUnstable) {
      Serial.print("DMA_FAULT");
    } else if (maxTuneDone) {
      Serial.print("LOCKED");
    } else if (maxTuning) {
      Serial.print("SEARCH");
    } else {
      Serial.print("OFF");
    }
    Serial.print(",max_seed_hz=");
    Serial.print(maxSeedHz);
    Serial.print(",max_probe_hz=");
    Serial.print(maxProbeHz);
    Serial.print(",max_stable_hz=");
    Serial.print(maxStableHz);
    Serial.print(",max_oper_hz=");
    Serial.print(actualFrameHz);
    Serial.print(",max_storage_backoffs=");
    Serial.print((unsigned long)maxStorageBackoffs);
  }

  Serial.print(",useful_sps=");
  Serial.print(acqHz * 3.0f, 1);
  Serial.print(",busy_adc_sps=");
  Serial.print(acqHz * (dummyOn ? 4.0f : 2.0f), 1);

  if (actualFrameHz != 0 && recording && mode == STORAGE_SD) {
    const float capturePct = (acqHz * 100.0f) / (float)actualFrameHz;
    Serial.print(",capture_pct=");
    Serial.print(capturePct, 2);
    Serial.print(",dma_rate_fault=");
    Serial.print(capturePct > 120.0f ? 1 : 0);
  }
  if (adcProfile == PROFILE_MAX_SPEED && mode == STORAGE_SD && recording) {
    Serial.print(",max_hw_hz=");
    Serial.print(acqHz, 1);
  }

  Serial.print(",records=");
  Serial.print((uint64_t)queued + totalNow);
  const uint32_t ringNow = (mode == STORAGE_FALLBACK) ? 0 : fullDepth;
  Serial.print(",ring=");
  Serial.print(ringNow);
  Serial.print(",ring_cap=");
  Serial.print(FULL_RING_RECORDS);
  Serial.print(",full_spill=");
  Serial.print(0UL);
  Serial.print(",full_spill_cap=");
  Serial.print(0UL);
  Serial.print(",full_q_total=");
  Serial.print(ringNow);
  Serial.print(",full_q_cap=");
  Serial.print(FULL_RING_RECORDS);
  Serial.print(",5k_records=");
  Serial.print((uint64_t)fiveCount + totalNow);
  Serial.print(",drop5k=");
  Serial.print(drop5k);
  const uint32_t ring5kNow = (mode == STORAGE_FALLBACK) ? 0 : fiveDepth;
  Serial.print(",ring5k=");
  Serial.print(ring5kNow);
  Serial.print(",ring5k_cap=");
  Serial.print(FIVEK_RING_RECORDS);
  Serial.print(",5k_spill=");
  Serial.print(0UL);
  Serial.print(",5k_spill_cap=");
  Serial.print(0UL);
  Serial.print(",5k_q_total=");
  Serial.print(ring5kNow);
  Serial.print(",5k_q_cap=");
  Serial.print(FIVEK_RING_RECORDS);
  Serial.print(",psram_records=");
  Serial.print(valid);
  Serial.print(",psram_cap=");
  Serial.print(cap);
  Serial.print(",psram_total=");
  Serial.print(totalNow);
  if (fullName[0] != '\0') {
    Serial.print(",full=");
    Serial.print(fullName);
  }
  if (fiveName[0] != '\0') {
    Serial.print(",fivek=");
    Serial.print(fiveName);
  }
  if (isFallback && recording) {
    Serial.print(",fb_adc_target_hz=");
    Serial.print(FALLBACK_ADC_HZ);
    Serial.print(",fb_adc_miss=");
    Serial.print((uint64_t)fbMissSession);
    Serial.print(",fb_decim=");
    Serial.print(4UL);
    Serial.print(",usb5k_drop=");
    Serial.print((uint64_t)usbDroppedTotal);
  }
  Serial.println();

  daqLastPsramTotal = totalNow;
  lastDaqMs = nowMs;
  daqLastSamples = samples;
  daqLastQueued = queued;
  daqLastFullBytes = bytesNow;
}

// CHECK_SNAPSHOT 출력.
static void printSnapshot() {
  noInterrupts();
  const uint32_t sampleCount = frameSeq;
  const uint16_t lc = liveLc, p1 = liveP1, p2 = liveP2;
  const uint32_t sdRecords = fullQueued;
  const uint32_t fiveRecords = fiveQueued;
  const uint32_t valid = psramValid;
  const uint64_t total = psramTotal, over = psramOverwritten;
  const uint32_t fDrops = fullDrops, vDrops = fiveDrops, fDisc = fullDiscarded, vDisc = fiveDiscarded, fExtra = fullExtraDrops;
  const uint64_t timelineMissing = fiveTimelineMissing;
  const uint64_t fullPhys = fullBytes, fivePhys = fiveBytes, schedMiss = fiveScheduleMisses;
  const uint64_t fbMiss = fbScheduleMisses, usbSentCount = usbSent, usbDrop = usbDroppedTotal;
  const uint32_t fullDepth = fullHead - fullTail;
  const uint32_t fiveDepth = fiveHead - fiveTail;
  const uint8_t mode = storageMode;
  interrupts();

  uint32_t fullFileSize = 0, fiveFileSize = 0;
  if (mode == STORAGE_SD && recording) {
    if (fullFile) {
      fullFileSize = (uint32_t)fullFile.size();
    }
    if (fiveFile) {
      fiveFileSize = (uint32_t)fiveFile.size();
    }
  }

  Serial.print("@CHECK,device=TEENSY_DAQ,kind=SNAPSHOT");
  Serial.print(",recording=");
  Serial.print((int)recording);
  Serial.print(",mode=");
  Serial.print(recording ? storageModeName(mode) : "IDLE");
  Serial.print(",sample_count=");
  Serial.print(sampleCount);
  Serial.print(",lc_raw=");
  Serial.print((int)lc);
  Serial.print(",p1_raw=");
  Serial.print((int)p1);
  Serial.print(",p2_raw=");
  Serial.print((int)p2);
  Serial.print(",sd_records=");
  Serial.print(sdRecords);
  Serial.print(",fivek_records=");
  Serial.print(fiveRecords);
  Serial.print(",full_bytes=");
  Serial.print(fullFileSize);
  Serial.print(",fivek_bytes=");
  Serial.print(fiveFileSize);
  Serial.print(",full_drop=");
  Serial.print((uint64_t)fDrops + fDisc + fExtra);
  Serial.print(",fivek_drop=");
  Serial.print((uint64_t)vDrops + vDisc + timelineMissing);
  Serial.print(",full_ring_drop=");
  Serial.print(fDrops);
  Serial.print(",fivek_ring_drop=");
  Serial.print(vDrops);
  Serial.print(",transition_full_discard=");
  Serial.print(fDisc);
  Serial.print(",transition_5k_discard=");
  Serial.print(vDisc);
  Serial.print(",inflight_full_discard=");
  Serial.print(fExtra);
  Serial.print(",timeline_5k_missing=");
  Serial.print(timelineMissing);
  Serial.print(",full_q=");
  Serial.print(fullDepth);
  Serial.print(",fivek_q=");
  Serial.print(fiveDepth);
  Serial.print(",full_spill_q=");
  Serial.print(0UL);
  Serial.print(",fivek_spill_q=");
  Serial.print(0UL);
  Serial.print(",full_q_total=");
  Serial.print(fullDepth);
  Serial.print(",fivek_q_total=");
  Serial.print(fiveDepth);
  Serial.print(",full_phys_bytes=");
  Serial.print(fullPhys);
  Serial.print(",fivek_phys_bytes=");
  Serial.print(fivePhys);
  Serial.print(",fivek_miss=");
  Serial.print(schedMiss);
  Serial.print(",spec_timer_miss=");
  Serial.print((uint64_t)etcErrorSpec);
  Serial.print(",etc_err=");
  Serial.print(etcErrorCount);
  Serial.print(",trigger_hz=");
  Serial.print(actualFrameHz);
  Serial.print(",psram_valid=");
  Serial.print(valid);
  Serial.print(",psram_total=");
  Serial.print(total);
  Serial.print(",psram_overwrite=");
  Serial.print(over);
  Serial.print(",fb_miss=");
  Serial.print(fbMiss);
  Serial.print(",fb_adc_target_hz=");
  Serial.print(FALLBACK_ADC_HZ);
  Serial.print(",fb_decim=");
  Serial.print(4UL);
  Serial.print(",fb_adc_miss=");
  Serial.print((uint64_t)fbMissSession);
  Serial.print(",fb_fir_valid=");
  Serial.print((unsigned long)firFill);
  Serial.print(",adc_profile=");
  Serial.print(adcProfileName());
  Serial.print(",dummy=");
  Serial.print((int)dummyOn);
  Serial.print(",adc_engine=DUAL_ADC_EDMA");
  Serial.print(",cpu_temp_c=");
  Serial.print(tempmonGetTemp(), 1);
  Serial.print(",usb_sent=");
  Serial.print(usbSentCount);
  Serial.print(",usb_drop=");
  Serial.print(usbDrop);
  Serial.print(",preserve_pending=");
  Serial.print((int)preserveGuard);
  Serial.print(",preserve_guard=");
  Serial.print((int)preserveOverflow);
  Serial.print(",psram_free=");
  Serial.print(valid < psramCap ? psramCap - valid : 0UL);
  Serial.print(",sd_write_fail=");
  Serial.print(sdToFallbackCount2);
  Serial.print(",safe_commit=");
  Serial.println(safeCommitCount);
  Serial.println("@ACK,device=TEENSY_DAQ,cmd=CHECK_SNAPSHOT,ok=1");
}

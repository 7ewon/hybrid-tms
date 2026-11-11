// SD 장애 대체 경로, PSRAM 상태, USB 이진 스트림, SD 복구.

static void printPsramStatus() {
  const uint64_t capacityBytes = (uint64_t)psramCap * PS_RECORD_BYTES;
  const uint64_t usedBytes = (uint64_t)psramValid * PS_RECORD_BYTES;
  const uint32_t remainingBytes = (usedBytes < capacityBytes) ? (uint32_t)(capacityBytes - usedBytes) : 0;
  const double usage = psramCap ? ((double)usedBytes * 100.0) / (double)capacityBytes : 0.0;

  Serial.println("#PSRAM_STATUS");
  Serial.print("# PS_RECORD_BYTES=");
  Serial.println(PS_RECORD_BYTES);
  Serial.print("# DETECTED_MIB=");
  Serial.println((unsigned long)external_psram_size);
  Serial.print("# CAPACITY_BYTES=");
  Serial.println((uint32_t)capacityBytes);
  Serial.print("# USED_BYTES=");
  Serial.println((uint32_t)usedBytes);
  Serial.print("# USAGE_PERCENT=");
  Serial.println((float)usage, 2);
  Serial.print("# REMAINING_BYTES=");
  Serial.println(remainingBytes);
  Serial.print("# VALID_RECORDS=");
  Serial.println(psramValid);
  Serial.print("# CAPACITY_RECORDS=");
  Serial.println(psramCap);
  Serial.print("# TOTAL_STORED_RECORDS=");
  Serial.println((uint64_t)psramTotal);
  Serial.print("# OVERWRITTEN_RECORDS=");
  Serial.println((uint64_t)psramOverwritten);
  Serial.print("# WRITE_INDEX=");
  Serial.println(psramWr);
  Serial.print("# OLDEST_INDEX=");
  Serial.println((psramValid != 0 && psramCap != 0 && psramValid >= psramCap) ? psramWr : 0UL);
  Serial.print("# FALLBACK_HZ=");
  Serial.println(LOG_5K_HZ);
  Serial.print("# RETAINED_SECONDS=");
  Serial.println((float)((double)psramValid / 5000.0), 3);
  Serial.print("# FULL_WINDOW_SECONDS=");
  Serial.println((float)((double)psramCap / 5000.0), 3);
  Serial.print("# FALLBACK_SCHEDULE_MISSES=");
  Serial.println((uint64_t)fbScheduleMisses);
  Serial.print("# SERIAL_DROPPED_LINES=");
  Serial.println((uint64_t)usbDroppedTotal);
  Serial.print("# ESTIMATED_DUMP_BYTES=");
  Serial.println(psramValid * PS_RECORD_BYTES + LOG_HEADER_BYTES);
}

// 대체 경로 헤더에서 달라지는 항목. 원본은 +0xA0을 5000으로 다시 쓴다.
static void patchFallbackHeader(uint8_t* h) {
  h[0x08] = 2;
  h[0x09] = (uint8_t)PS_RECORD_BYTES;
  h[0x0C] = 2;
  headerPut32(h, 0x24, sdFileSession);
  headerPut32(h, 0x28, 0);
  headerPut32(h, 0x98, 1);
  headerPut32(h, 0x9C, FALLBACK_ADC_HZ);
  headerPut32(h, 0xA0, LOG_5K_HZ);
  headerPut32(h, 0xA4, (uint32_t)fbMissSession);
  headerPut32(h, 0xA8, 2);
  headerPut32(h, 0xAC, LOG_5K_HZ);
}

static bool sendUsbHeader() {
  uint8_t header[LOG_HEADER_BYTES];
  fillLogHeader(header);
  patchFallbackHeader(header);

  uint8_t packet[16];
  memcpy(packet, "LG5K", 4);
  packet[4] = 1;
  packet[5] = 2;
  packet[6] = (uint8_t)PS_RECORD_BYTES;
  packet[7] = 0;
  memcpy(packet + 8, "\0\0", 2);
  const uint16_t headerBytes = LOG_HEADER_BYTES;
  memcpy(packet + 10, &headerBytes, 2);
  memcpy(packet + 12, &usbPacketSeq, 4);

  if ((uint32_t)Serial.availableForWrite() < 0x250) {
    return false;
  }
  if (Serial.write(packet, 16) != 16 || Serial.write(header, LOG_HEADER_BYTES) != LOG_HEADER_BYTES) {
    return false;
  }
  usbPacketSeq++;
  usbHeaderPending = 0;
  usbLastHeaderMs = millis();
  return true;
}

// PSRAM에 쌓인 5 kHz 레코드를 USB로 보낸다.
static void usbStreamService() {
  if (recording && storageMode == STORAGE_FALLBACK && !usbHeaderPending && (uint32_t)(millis() - usbLastHeaderMs) > 1999) {
    usbHeaderPending = 1;
  }
  if (usbHeaderPending && !sendUsbHeader()) {
    return;
  }

  uint32_t packets = 0;
  for (;;) {
    noInterrupts();
    const uint64_t cursor = usbCursor;
    const uint64_t total = psramTotal;
    const uint32_t valid = psramValid;
    const uint32_t wr = psramWr;
    const uint32_t cap = psramCap;
    interrupts();

    if (psramBuf == nullptr || valid == 0 || cap == 0 || total <= cursor) {
      break;
    }
    const uint64_t oldest = total - valid;
    if (cursor < oldest) {
      const uint64_t dropped = oldest - cursor;
      noInterrupts();
      usbCursor = oldest;
      usbDroppedTotal = usbDroppedTotal + dropped;
      usbDroppedOverflow = usbDroppedOverflow + dropped;
      interrupts();
      continue;
    }

    const uint64_t backlog = total - cursor;
    const uint32_t index = (uint32_t)((wr + cap - (uint32_t)(backlog % cap)) % cap);
    uint32_t count = (backlog > USB_PACKET_RECORDS) ? USB_PACKET_RECORDS : (uint32_t)backlog;
    if (count > cap - index) {
      count = cap - index;
    }
    if (count == 0) {
      break;
    }
    const uint32_t bytes = (count * PS_RECORD_BYTES) & 0xFFFFU;
    if ((uint32_t)Serial.availableForWrite() < bytes + 0x50U) {
      break;
    }
    memcpy(usbPacketBuf, psramBuf + index * PS_RECORD_BYTES, bytes);

    noInterrupts();
    const uint64_t oldestNow = psramTotal - psramValid;
    interrupts();
    if (cursor < oldestNow) {
      continue;
    }

    uint8_t packet[16];
    memcpy(packet, "LG5K", 4);
    packet[4] = 1;
    packet[5] = 1;
    packet[6] = (uint8_t)PS_RECORD_BYTES;
    packet[7] = 0;
    const uint16_t count16 = (uint16_t)count;
    const uint16_t bytes16 = (uint16_t)bytes;
    memcpy(packet + 8, &count16, 2);
    memcpy(packet + 10, &bytes16, 2);
    memcpy(packet + 12, &usbPacketSeq, 4);
    arm_dcache_flush(usbPacketBuf, bytes);

    if (Serial.write(packet, 16) != 16 || Serial.write(usbPacketBuf, bytes) != bytes) {
      usbDropWrite = usbDropWrite + 1;
      break;
    }

    noInterrupts();
    if (usbCursor == cursor) {
      usbCursor = cursor + count;
    }
    if (usbCursor < psramTotal) {
      const uint64_t remain = psramTotal - usbCursor;
      if (remain > usbMaxBacklog) {
        usbMaxBacklog = (remain >> 32) ? 0xFFFFFFFFUL : (uint32_t)remain;
      }
    }
    interrupts();

    usbPacketSeq++;
    usbSent = usbSent + count;
    packets++;
    if (packets == USB_PACKET_BUDGET) {
      break;
    }
  }

  if (usbDrainPending && !usbHeaderPending) {
    noInterrupts();
    const bool drained = (psramTotal <= usbCursor) || psramValid == 0;
    interrupts();
    if (drained) {
      Serial.print("@EVENT,device=TEENSY_DAQ,type=FALLBACK_STREAM_END");
      Serial.print(",sent_records=");
      Serial.print((uint64_t)usbSent);
      Serial.print(",usb_dropped_records=");
      Serial.println((uint64_t)usbDroppedTotal);
      usbDrainPending = 0;
    }
  }
}

// SD 쓰기에 실패하면 20 kHz 수집과 PSRAM, USB 경로로 전환한다.
static void enterFallback(const char* reason) {
  if (storageMode == STORAGE_FALLBACK) {
    return;
  }
  if (!psramReady()) {
    Serial.println("# CRITICAL: SD_FAILED_AND_PSRAM_UNAVAILABLE");
    Serial.println("@EVENT,device=TEENSY_DAQ,type=CRITICAL_STORAGE_FAILURE,reason=SD_FAILED_AND_PSRAM_UNAVAILABLE");
    storageHalted = 1;
    return;
  }

  stopAdc();
  noInterrupts();
  firIndex = 0;
  firFill = 0;
  firDecim = 0;
  firResetRequest = 0;
  filtLc = liveLc;
  filtP1 = liveP1;
  filtP2 = liveP2;
  interrupts();

  const uint32_t nowUs = micros();
  noInterrupts();
  if (storageMode == STORAGE_FALLBACK) {
    interrupts();
    return;
  }
  fullDiscarded = (fullHead + fullDiscarded) - fullTail;
  fiveDiscarded = (fiveHead + fiveDiscarded) - fiveTail;
  fullTail = fullHead;
  fiveTail = fiveHead;
  sdToFallbackCount++;
  sdToFallbackCount2++;
  usbLastHeaderMs = 0;
  fbMissSession = 0;
  usbSent = 0;
  firResetRequest = 0;
  firSettle = 0;
  adcFallbackRun = 1;
  dummyOn = 1;
  dummyOverride = 0;
  usbPacketSeq = 0;
  usbHeaderPending = 1;
  usbDrainPending = 0;
  usbCursor = psramTotal;
  usbDroppedOverflow = 0;
  usbDropWrite = 0;
  usbMaxBacklog = 0;
  storageMode = STORAGE_FALLBACK;
  interrupts();

  fbSampleUs = nowUs;
  startFallbackAdc();

  fullFile = File();
  fiveFile = File();

  sdRecoveryPollMs = millis();
  sdPresentDuringFallback = 0;
  sdRecoveryAttemptMs = millis();
  sdLastRecoveryUs = 0;
  sdRecoveryBusy = 0;
  sdRecoveryAttempts = 0;
  sdRecoveredFlag = 0;
  pinMode(PIN_SD_DETECT, INPUT_PULLDOWN);
  delayMicroseconds(5);

  Serial.print("# WARNING: ENTER_FALLBACK_ADC20K_FIR5K,REASON=");
  Serial.println(reason);
  Serial.print("# DISCARDED_QUEUED_SD_RECORDS_TOTAL=");
  Serial.println(fullDiscarded);
  Serial.print("# DISCARDED_QUEUED_5K_RECORDS_TOTAL=");
  Serial.println(fiveDiscarded);
  Serial.print("@EVENT,device=TEENSY_DAQ,type=FALLBACK_ENTERED,reason=");
  Serial.print(reason);
  printFallbackEnteredFields(nowUs);

  lastDaqMs = millis();
  daqLastSamples = frameSeq;
  daqLastQueued = fullQueued;
  daqLastPsramTotal = psramTotal;
  daqLastFullBytes = fullBytes;
  printPsramStatus();
}

// 기록 중이 아닐 때 SD 카드 상태를 확인한다.
static bool sdIdleProbe(bool force) {
  const uint8_t wasKnown = sdIdleKnown;
  const uint32_t nowMs = millis();
  const uint8_t previous = sdIdlePresent;
  uint8_t present = 0;
  bool haveResult = false;

  if (force || !wasKnown) {
    const bool media = SD.mediaPresent();
    if (wasKnown) {
      if (!media) {
        if (!force) {
          present = ((uint32_t)(nowMs - sdIdleProbeMs) < 2000) ? 0 : ((previous ^ 1) & 1);
          if (present == 0) {
            haveResult = true;
          }
        }
      } else if (previous && !force) {
        sdIdlePresent = 1;
        sdIdleRawMisses = 0;
        sdIdleKnown = 1;
        return true;
      }
    }
  } else if (previous) {
    SdCard* card = SD.sdfs.card();
    if (card == nullptr || !card->readSector(0, sdProbeSector)) {
      if (sdIdleRawMisses != 0xFF) {
        sdIdleRawMisses++;
      }
      const bool media = SD.mediaPresent();
      if (!media || sdIdleRawMisses > 1) {
        sdIdlePresent = 0;
        sdIdleRawMisses = 0;
        SD.sdfs.end();
        sdIdleKnown = 1;
        sdIdleProbeMs = nowMs;
        Serial.println("@EVENT,device=TEENSY_DAQ,type=SD_IDLE_CHANGED,sd=0,detector=RAW_SECTOR");
        return false;
      }
    } else {
      sdIdleRawMisses = 0;
    }
    return true;
  } else {
    if (!SD.mediaPresent() && (uint32_t)(nowMs - sdIdleProbeMs) < 2000) {
      sdIdleKnown = 1;
      sdIdleRawMisses = 0;
      sdIdlePresent = 0;
      return false;
    }
  }

  if (!haveResult) {
    present = sdBeginFresh() ? 1 : 0;
    sdIdleProbeMs = millis();
  }

  sdIdleKnown = 1;
  sdIdlePresent = present;
  sdIdleRawMisses = 0;
  if (((previous ^ present) & 0xFF) != 0 && wasKnown) {
    Serial.print("@EVENT,device=TEENSY_DAQ,type=SD_IDLE_CHANGED,sd=");
    Serial.print((int)present);
    Serial.println(",detector=FRESH_PROBE");
  }
  return present != 0;
}

// 대체 경로로 기록 중일 때 SD 복구를 시도한다.
static void sdRecoveryService(bool manual) {
  if (!recording || storageMode != STORAGE_FALLBACK) {
    return;
  }
  if (!manual) {
    if ((uint32_t)(millis() - sdRecoveryPollMs) < 500) {
      return;
    }
    sdRecoveryPollMs = millis();
    if ((GPIO8_REG_PSR & 0x20000U) == 0) {
      sdPresentDuringFallback = 0;
      return;
    }
    if ((uint32_t)(millis() - sdRecoveryAttemptMs) < 2000) {
      return;
    }
  }

  sdRecoveryBusy = 1;
  if (sdRecoveryAttempts != 0xFF) {
    sdRecoveryAttempts++;
  }
  sdRecoveryAttemptMs = millis();

  const uint32_t startUs = micros();
  bool ok = sdBeginFresh();
  if (ok) {
    ok = openSdLogs();
  }
  sdLastRecoveryUs = micros() - startUs;
  if (sdMaxRecoveryUs < sdLastRecoveryUs) {
    sdMaxRecoveryUs = sdLastRecoveryUs;
  }
  sdRecoveryAttemptMs = millis();

  if (ok) {
    sdRecoveryBusy = 0;
    sdPresentDuringFallback = 1;
    sdRecoveryAttempts = 0;
    sdRecoveredFlag = 1;
    stopAdc();

    noInterrupts();
    fullHead = 0;
    fullTail = 0;
    fullWr = 0;
    fiveHead = 0;
    fiveTail = 0;
    fiveWr = 0;
    next5kCycle = ARM_DWT_CYCCNT;
    sdStartFlag = 1;
    storageMode = STORAGE_SD;
    adcFallbackRun = 0;
    dummyOn = (adcProfile == PROFILE_SPEC_SAFE) ? 1 : 0;
    dummyOverride = 0;
    adcActive = 0;
    fallbackToSdCount++;
    interrupts();

    startProfileAdc();
    usbDrainPending = 1;

    Serial.print("@EVENT,device=TEENSY_DAQ,type=SD_RECOVERED,recovery=");
    Serial.print(manual ? "MANUAL" : "AUTO");
    Serial.print(",sd=1,usable=1,full=");
    Serial.print(fullName);
    Serial.print(",fivek=");
    Serial.println(fiveName);
    if (manual) {
      Serial.print("@ACK,device=TEENSY_DAQ,cmd=SD_RETRY,ok=1,mode=SD,full=");
      Serial.print(fullName);
      Serial.print(",fivek=");
      Serial.println(fiveName);
    }

    lastDaqMs = millis();
    daqLastSamples = frameSeq;
    daqLastQueued = fullQueued;
    daqLastPsramTotal = psramTotal;
    daqLastFullBytes = fullBytes;
    return;
  }

  sdPresentDuringFallback = 0;
  sdRecoveredFlag = 0;
  sdRecoveryBusy = 0;
  sdRecoveryAttemptMs = millis();
  pinMode(PIN_SD_DETECT, INPUT_PULLDOWN);
  delayMicroseconds(5);
  if (manual) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=SD_RETRY,ok=0,error=SD_REINIT_FAILED");
  }
}

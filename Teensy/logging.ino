// SD 파일, 512바이트 헤더, 기록 시작과 중지, 전원 재개.

static const char* storageModeName(uint8_t mode) {
  if (mode == STORAGE_SD) {
    return "SD";
  }
  if (mode == STORAGE_FALLBACK) {
    return "PSRAM";
  }
  return "NONE";
}

static const char* adcProfileName() {
  return (adcProfile == PROFILE_MAX_SPEED) ? "MAX_SPEED" : "SPEC_SAFE";
}

// ---------------------------------------------------------------------------
// PSRAM
// ---------------------------------------------------------------------------
static bool psramReady() {
  if (psramBuf != nullptr) {
    return true;
  }
  if (external_psram_size == 0) {
    return false;
  }
  uint32_t bytes = (uint32_t)external_psram_size * 0x100000U - 0x10000U;
  psramBuf = (uint8_t*)extmem_malloc(bytes);
  while (psramBuf == nullptr) {
    if (bytes < 0x100001U) {
      psramBytes = 0;
      psramCap = 0;
      return false;
    }
    bytes -= 0x100000U;
    psramBuf = (uint8_t*)extmem_malloc(bytes);
  }
  psramCap = bytes / PS_RECORD_BYTES;
  psramBytes = bytes;
  return true;
}

static void psramClearLogical() {
  noInterrupts();
  psramWr = 0;
  psramValid = 0;
  psramTotal = 0;
  psramOverwritten = 0;
  fbScheduleMisses = 0;
  usbCursor = 0;
  usbDroppedTotal = 0;
  usbDroppedOverflow = 0;
  usbDropWrite = 0;
  usbMaxBacklog = 0;
  interrupts();
  psramHasData = 0;
}

// 점검 기록 전에 PSRAM 상태를 보관하고, 끝나면 되돌린다.
static void preserveSave() {
  preserveBackup.wr = psramWr;
  preserveBackup.valid = psramValid;
  preserveBackup.sessionUs = sessionUs;
  preserveBackup.fileSession = sdFileSession;
  preserveBackup.hasData = psramHasData;
  preserveBackup.total = psramTotal;
  preserveBackup.overwritten = psramOverwritten;
  preserveBackup.fbMisses = fbScheduleMisses;
  preserveBackup.fbSession = fbMissSession;
  preserveBackup.usbDropped = usbDroppedTotal;
  preserveBackup.usbOverflow = usbDroppedOverflow;
  preserveBackup.usbWrite = usbDropWrite;
  preserveBackup.usbBacklog = usbMaxBacklog;
  preserveBackup.frameSeq = frameSeq;
  preserveBackup.usbSentCount = usbSent;
}

static void preserveRestore() {
  noInterrupts();
  psramWr = preserveBackup.wr;
  psramValid = preserveBackup.valid;
  psramTotal = preserveBackup.total;
  psramOverwritten = preserveBackup.overwritten;
  fbScheduleMisses = preserveBackup.fbMisses;
  fbMissSession = preserveBackup.fbSession;
  usbDroppedTotal = preserveBackup.usbDropped;
  usbDroppedOverflow = preserveBackup.usbOverflow;
  usbDropWrite = preserveBackup.usbWrite;
  usbMaxBacklog = preserveBackup.usbBacklog;
  frameSeq = preserveBackup.frameSeq;
  usbSent = preserveBackup.usbSentCount;
  sessionUs = preserveBackup.sessionUs;
  psramHasData = preserveBackup.hasData;
  sdFileSession = preserveBackup.fileSession;
  usbCursor = preserveBackup.total;
  usbHeaderPending = 0;
  usbDrainPending = 0;
  preserveGuard = 0;
  preserveOverflow = 0;
  interrupts();
}

// ---------------------------------------------------------------------------
// 512바이트 파일 헤더
// ---------------------------------------------------------------------------
static inline void headerPut32(uint8_t* h, uint32_t offset, uint32_t value) {
  memcpy(h + offset, &value, 4);
}
static inline void headerPut16(uint8_t* h, uint32_t offset, uint16_t value) {
  memcpy(h + offset, &value, 2);
}
static inline void headerPutFloat(uint8_t* h, uint32_t offset, float value) {
  memcpy(h + offset, &value, 4);
}

static void fillLogHeader(uint8_t* h) {
  memset(h, 0, LOG_HEADER_BYTES);
  const uint32_t mib = external_psram_size;
  memcpy(h, "LG07", 4);
  headerPut16(h, 0x04, 7);
  headerPut16(h, 0x06, LOG_HEADER_BYTES);
  headerPut16(h, 0x0A, 0x030C);
  headerPut32(h, 0x10, mib ? 0x7F : 0x77);
  headerPut32(h, 0x14, CPU_HZ_NOMINAL);
  headerPut32(h, 0x18, LOG_5K_HZ);
  headerPut32(h, 0x1C, mib);
  headerPut32(h, 0x20, psramCap);
  headerPutFloat(h, 0x2C, ADC_VREF);
  headerPutFloat(h, 0x34, LC_K);
  headerPutFloat(h, 0x3C, P1_K);
  headerPutFloat(h, 0x44, P2_K);
  headerPut32(h, 0x48, 1);
  headerPut32(h, 0x4C, psramValid);
  headerPut32(h, 0x50, psramWr);
  const uint64_t total = psramTotal;
  const uint64_t over = psramOverwritten;
  headerPut32(h, 0x54, (uint32_t)total);
  headerPut32(h, 0x58, (uint32_t)(total >> 32));
  headerPut32(h, 0x5C, (uint32_t)over);
  headerPut32(h, 0x60, (uint32_t)(over >> 32));
  uint32_t oldest = 0;
  if (psramCap != 0 && psramValid != 0 && psramValid >= psramCap) {
    oldest = psramWr;
  }
  headerPut32(h, 0x64, oldest);
  const uint64_t fbMisses = fbScheduleMisses;
  const uint64_t usbDrop = usbDroppedTotal;
  headerPut32(h, 0x68, (uint32_t)fbMisses);
  headerPut32(h, 0x6C, (uint32_t)(fbMisses >> 32));
  headerPut32(h, 0x70, (uint32_t)usbDrop);
  headerPut32(h, 0x74, (uint32_t)(usbDrop >> 32));
  headerPut32(h, 0x78, sdToFallbackCount);
  headerPut32(h, 0x7C, fallbackToSdCount);
  headerPut32(h, 0x80, 0);
  headerPut32(h, 0x84, sdToFallbackCount2);
  headerPut32(h, 0x88, fullDrops);
  headerPut32(h, 0x8C, fullDiscarded);
  headerPut32(h, 0x90, frameSeq);
  headerPut32(h, 0x94, sdFileSession);
  headerPut32(h, 0x98, adcProfile == PROFILE_SPEC_SAFE ? 1 : 0);
  headerPut32(h, 0x9C, adcProfile == PROFILE_SPEC_SAFE ? SPEC_FRAME_HZ : 0);
  headerPut32(h, 0xA0, LOG_5K_HZ);
  headerPut32(h, 0xA4, (uint32_t)etcErrorSpec);
  headerPut32(h, 0xB0, 1);
  headerPut32(h, 0xB4, sessionUs);
  headerPut32(h, 0xB8, micros());
}

// SD 파일 헤더에서 파일별로 달라지는 항목을 넣는다. 원본 코드에서 FULL과 5K의 차이는 +0xA8, +0xAC뿐이다.
static void patchSdHeader(uint8_t* h, uint32_t session, uint32_t startSeq, bool fiveK) {
  h[0x08] = 1;
  h[0x09] = (uint8_t)SD_RECORD_BYTES;
  h[0x0C] = 1;
  headerPut32(h, 0x24, session);
  headerPut32(h, 0x28, startSeq);
  headerPut32(h, 0xA8, fiveK ? 1 : 0);
  headerPut32(h, 0xAC, fiveK ? LOG_5K_HZ : 0);
}

// ---------------------------------------------------------------------------
// SD
// ---------------------------------------------------------------------------
static bool sdBeginFresh() {
  if (!SD.sdfs.begin(SdioConfig(DMA_SDIO))) {
    return false;
  }
  return SD.mediaPresent();
}

static bool openSdLogs() {
  uint32_t index = 0;
  for (;;) {
    snprintf(fullName, sizeof(fullName), "TLOG%05lu_FULL.BIN", (unsigned long)index);
    snprintf(fiveName, sizeof(fiveName), "TLOG%05lu_5K.BIN", (unsigned long)index);
    if (!SD.exists(fullName) && !SD.exists(fiveName)) {
      break;
    }
    index++;
    if (index == 100000) {
      strcpy(fullName, "TLOG99999_FULL.BIN");
      strcpy(fiveName, "TLOG99999_5K.BIN");
      break;
    }
  }

  File newFull = SD.open(fullName, FILE_WRITE);
  if (!newFull) {
    return false;
  }
  File newFive = SD.open(fiveName, FILE_WRITE);
  if (!newFive) {
    newFull.close();
    if (SD.mediaPresent()) {
      SD.remove(fullName);
    }
    return false;
  }

  const uint32_t session = sdFileSession;
  const uint32_t startSeq = frameSeq;
  static uint8_t fullHeader[LOG_HEADER_BYTES];
  static uint8_t fiveHeader[LOG_HEADER_BYTES];
  fillLogHeader(fullHeader);
  patchSdHeader(fullHeader, session, startSeq, false);
  fillLogHeader(fiveHeader);
  patchSdHeader(fiveHeader, session, startSeq, true);

  bool ok = newFull.write(fullHeader, LOG_HEADER_BYTES) == LOG_HEADER_BYTES;
  ok = ok && newFive.write(fiveHeader, LOG_HEADER_BYTES) == LOG_HEADER_BYTES;
  if (ok) {
    newFull.flush();
    newFive.flush();
    ok = SD.mediaPresent();
  }
  if (!ok) {
    newFull.close();
    newFive.close();
    if (SD.mediaPresent()) {
      SD.remove(fullName);
      SD.remove(fiveName);
    }
    return false;
  }

  fullFile = newFull;
  fiveFile = newFive;
  sdFileSession = session + 1;
  sdStartFlag = 1;
  return true;
}

// FULL 링에서 최대 4096개 레코드를 쓴다. 실패하면 false.
static bool writeFullChunk() {
  if (storageMode != STORAGE_SD || !fullFile) {
    return true;
  }
  const uint32_t tail = fullTail;
  uint32_t pending = fullHead - tail;
  if (pending == 0) {
    return true;
  }
  if (pending >= SD_WRITE_CHUNK_RECORDS) {
    pending = SD_WRITE_CHUNK_RECORDS;
  }
  const uint32_t index = tail % FULL_RING_RECORDS;
  LogRecord* src = &fullRing[index];
  uint32_t count = FULL_RING_RECORDS - index;
  if (pending <= count) {
    count = pending;
  }
  const uint32_t bytes = count * SD_RECORD_BYTES;
  arm_dcache_flush(src, bytes);

  const uint32_t startUs = micros();
  const uint32_t written = fullFile.write((const uint8_t*)src, bytes);
  lastFullWriteUs = micros() - startUs;
  if (maxFullWriteUs < lastFullWriteUs) {
    maxFullWriteUs = lastFullWriteUs;
  }
  fullBytes += written;
  lastFullWriteBytes = bytes;
  if (written != bytes) {
    return false;
  }
  fullTail = tail + count;
  return true;
}

// 5K 링에서 최대 1024개 레코드를 쓴다. 실패하면 false.
static bool writeFiveChunk() {
  if (storageMode != STORAGE_SD || !fiveFile) {
    return true;
  }
  const uint32_t tail = fiveTail;
  uint32_t pending = fiveHead - tail;
  if (pending == 0) {
    return true;
  }
  if (pending >= SD_5K_WRITE_CHUNK_RECORDS) {
    pending = SD_5K_WRITE_CHUNK_RECORDS;
  }
  const uint32_t index = tail & (FIVEK_RING_RECORDS - 1);
  LogRecord* src = &fiveRing[index];
  uint32_t count = FIVEK_RING_RECORDS - index;
  if (pending <= count) {
    count = pending;
  }
  const uint32_t bytes = count * SD_RECORD_BYTES;
  arm_dcache_flush(src, bytes);

  const uint32_t startUs = micros();
  const uint32_t written = fiveFile.write((const uint8_t*)src, bytes);
  lastFiveWriteUs = micros() - startUs;
  if (maxFiveWriteUs < lastFiveWriteUs) {
    maxFiveWriteUs = lastFiveWriteUs;
  }
  fiveBytes += written;
  lastFiveWriteBytes = bytes;
  if (written != bytes) {
    return false;
  }
  fiveTail = tail + count;
  return true;
}

// ---------------------------------------------------------------------------
// 전원 재개 EEPROM
// ---------------------------------------------------------------------------
const uint32_t RESUME_MAGIC = 0x4C534D54UL;

static bool readResumeFlag(uint8_t* armed) {
  uint8_t raw[8];
  for (uint32_t i = 0; i < 8; i++) {
    raw[i] = EEPROM.read(i);
  }
  uint32_t magic;
  memcpy(&magic, raw, 4);
  if (magic != RESUME_MAGIC || raw[4] != 1) {
    return false;
  }
  if ((uint8_t)(raw[5] ^ raw[6]) != 0xFF) {
    return false;
  }
  *armed = raw[5];
  return true;
}

static void setResumeArmed(uint8_t armed) {
  uint8_t current = 0;
  if (readResumeFlag(&current) && current < 2 && current == armed) {
    resumeArmed = armed;
    return;
  }
  uint8_t raw[8];
  memcpy(raw, &RESUME_MAGIC, 4);
  raw[4] = 1;
  raw[5] = armed;
  raw[6] = (uint8_t)~armed;
  raw[7] = 0;
  for (uint32_t i = 0; i < 8; i++) {
    EEPROM.update(i, raw[i]);
  }
  resumeArmed = armed;
}

// ---------------------------------------------------------------------------
// 기록 시작
// ---------------------------------------------------------------------------
static void ackPrefix(const char* command) {
  Serial.print("@ACK,device=TEENSY_DAQ,cmd=");
  Serial.print(command);
}

static void resetSessionCounters() {
  noInterrupts();
  fullHead = 0;
  fullTail = 0;
  fullWr = 0;
  fullDrops = 0;
  fullDiscarded = 0;
  fullExtraDrops = 0;
  gapFlag = 0;
  sdStartFlag = 0;
  fiveHead = 0;
  fiveTail = 0;
  fiveWr = 0;
  fullMaxDepth = 0;
  fiveMaxDepth = 0;
  fullName[0] = '\0';
  fiveName[0] = '\0';
  lastSeenFiveDrops = 0;
  fiveBytes = 0;
  lastSeenFullDrops = 0;
  fullBytes = 0;
  etcErrorAtStart = etcErrorCount;
  lastFullWriteUs = 0;
  daqLastFullBytes = 0;
  maxFullWriteUs = 0;
  lastFiveWriteUs = 0;
  maxFiveWriteUs = 0;
  lastFullWriteBytes = 0;
  fiveDrops = 0;
  lastFiveWriteBytes = 0;
  fiveDiscarded = 0;
  fiveTimelineMissing = 0;
  lastFiveTimelineUs = 0;
  fiveTimelineValid = 0;
  fiveQueued = 0;
  fiveScheduleMisses = 0;
  next5kCycle = 0;
  frameSeq = 0;
  fullQueued = 0;
  etcErrorSpec = 0;
  lastIsrCycle = 0;
  cyc64Valid = 0;
  isrCycle = 0;
  cyc64NowLo = 0;
  adcActive = 0;
  adcFallbackRun = 0;
  fbNextUs = 0;
  fbSampleUs = 0;
  fbMissSession = 0;
  firResetRequest = 0;
  firSettle = 0;
  sdRecoveryBusy = 0;
  firIndex = 0;
  firFill = 0;
  sdRecoveryAttempts = 0;
  firDecim = 0;
  filtLc = 0;
  filtP1 = 0;
  filtP2 = 0;
  sdToFallbackCount = 0;
  fallbackToSdCount = 0;
  sdToFallbackCount2 = 0;
  sdFileSession = 0;
  psramWr = 0;
  psramValid = 0;
  psramTotal = 0;
  psramOverwritten = 0;
  fbScheduleMisses = 0;
  usbSent = 0;
  usbLastHeaderMs = 0;
  sdPresentDuringFallback = 0;
  sdRecoveredFlag = 0;
  usbCursor = 0;
  usbDroppedTotal = 0;
  usbDroppedOverflow = 0;
  usbDropWrite = 0;
  usbMaxBacklog = 0;
  usbPacketSeq = 0;
  usbHeaderPending = 0;
  usbDrainPending = 0;
  storageHalted = 0;
  interrupts();
  safeCommitCount = 0;
  safeCommitDeferred = 0;
  psramHasData = 0;
}

static void resetCycleCounter() {
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  ARM_DWT_CYCCNT = 0;
  sessionUs = micros();
}

static void printFallbackEnteredFields(uint32_t nowUs) {
  Serial.print(",sd=");
  Serial.print((int)sdPresentDuringFallback);
  Serial.print(",session_us=");
  Serial.print(nowUs - startMicros);
  Serial.print(",source_us=");
  Serial.print(nowUs);
  Serial.print(",acq_target_hz=");
  Serial.print(FALLBACK_ADC_HZ);
  Serial.print(",log_target_hz=");
  Serial.print(LOG_5K_HZ);
  Serial.print(",psram_records=");
  Serial.print(psramValid);
  Serial.print(",psram_cap=");
  Serial.print(psramCap);
  Serial.print(",drop=");
  Serial.print((uint64_t)fullDrops + fullDiscarded + fullExtraDrops);
  Serial.print(",drop5k=");
  Serial.print((uint64_t)fiveDrops + fiveDiscarded + fiveTimelineMissing);
  Serial.println();
}

// 대체 경로로 기록을 시작할 때의 공통 준비.
static void prepareFallbackStart(int startKind) {
  usbLastHeaderMs = 0;
  sdRecoveredFlag = 0;
  sdPresentDuringFallback = 0;
  storageMode = STORAGE_FALLBACK;
  sdRecoveryPollMs = millis();
  usbPacketSeq = 0;
  usbHeaderPending = 1;
  usbDrainPending = 0;
  sdRecoveryAttemptMs = millis();
  sdLastRecoveryUs = 0;
  usbCursor = psramTotal;
  usbDroppedOverflow = 0;
  usbSent = 0;
  usbDropWrite = 0;
  usbMaxBacklog = 0;
  pinMode(PIN_SD_DETECT, INPUT_PULLDOWN);
  delayMicroseconds(5);

  if (startKind == START_CHECK_FALLBACK) {
    Serial.print("# SYSTEM_CHECK,MODE=FORCED_PSRAM_");
    Serial.print(LOG_5K_HZ);
    Serial.println("HZ");
  } else {
    Serial.println("# WARNING: SD_NOT_AVAILABLE_AT_START");
    Serial.print("# START,MODE=PSRAM_CIRCULAR_");
    Serial.print(LOG_5K_HZ);
    Serial.println("HZ_PLUS_USB_BINARY");
    printPsramStatus();
  }
  checkDeleteFiles = 0;
}

static void printStartSd() {
  storageMode = STORAGE_SD;
  Serial.print("# START,MODE=SD_CONTINUOUS,ADC_PROFILE=");
  Serial.print(adcProfileName());
  Serial.print(",FULL=");
  Serial.print(fullName);
  Serial.print(",FIVEK=");
  Serial.println(fiveName);
}

static void startRecording(int startKind, const char* command) {
  if (monitoring) {
    monitoring = 0;
    adcActive = 0;
    stopAdc();
    Serial.println("@EVENT,device=TEENSY_DAQ,type=MONITOR_STOPPED");
  }

  if (recording) {
    if (startKind == START_NORMAL) {
      Serial.println("# ALREADY_RECORDING");
    }
    ackPrefix(command);
    Serial.print(startKind == START_NORMAL ? ",ok=1,state=ALREADY_RECORDING,mode=" : ",ok=0,error=ALREADY_RECORDING,mode=");
    Serial.println(storageModeName(storageMode));
    return;
  }

  if (startKind == START_NORMAL) {
    if (psramHasData && psramValid != 0) {
      Serial.println("# WARNING: NEW_START_CLEARS_PREVIOUS_PSRAM_DATA");
    }
  } else {
    // 점검 기록은 기존 PSRAM 데이터를 덮어쓰지 않도록 상태를 보관한 뒤 시작한다.
    if (!preserveGuard && psramReady()) {
      noInterrupts();
      if (psramCap != 0 && psramValid < psramCap && psramCap - psramValid > 19999U) {
        preserveSave();
        preserveGuard = 1;
        interrupts();
        preserveOverflow = 0;
      } else {
        const uint32_t freeRecords = (psramCap > psramValid) ? psramCap - psramValid : 0;
        interrupts();
        ackPrefix(command);
        Serial.print(",ok=0,error=INSUFFICIENT_FREE_PSRAM_FOR_PRESERVE_CHECK");
        Serial.print(",free_records=");
        Serial.println(freeRecords);
        return;
      }
    } else if (preserveGuard) {
      ackPrefix(command);
      Serial.print(",ok=0,error=CHECK_RESTORE_REQUIRED");
      Serial.print(",free_records=");
      Serial.println(psramCap > psramValid ? psramCap - psramValid : 0UL);
      return;
    } else {
      ackPrefix(command);
      Serial.print(",ok=0,error=INSUFFICIENT_FREE_PSRAM_FOR_PRESERVE_CHECK");
      Serial.print(",free_records=");
      Serial.println(0UL);
      return;
    }
  }

  resetSessionCounters();

  bool sdStarted = false;
  int forcedFallback = 0;

  if (startKind == START_NORMAL) {
    if (psramReady()) {
      psramClearLogical();
    }
    resetCycleCounter();
    if (sdBeginFresh() && openSdLogs()) {
      sdStarted = true;
      checkDeleteFiles = 0;
    } else if (psramReady()) {
      prepareFallbackStart(START_NORMAL);
    } else {
      storageMode = STORAGE_NONE;
      Serial.println("# ERROR: NO_SD_AND_NO_PSRAM");
      ackPrefix(command);
      Serial.print(",ok=0,error=");
      Serial.println("NO_SD_AND_NO_PSRAM");
      return;
    }
  } else {
    if (preserveGuard) {
      noInterrupts();
      psramHasData = preserveBackup.hasData | (preserveBackup.valid != 0 ? 1 : 0);
      psramWr = preserveBackup.wr;
      psramValid = preserveBackup.valid;
      psramTotal = preserveBackup.total;
      psramOverwritten = preserveBackup.overwritten;
      interrupts();
    }
    resetCycleCounter();
    if (startKind == START_CHECK_FALLBACK) {
      if (psramReady()) {
        prepareFallbackStart(START_CHECK_FALLBACK);
        forcedFallback = 1;
      } else {
        storageMode = STORAGE_NONE;
        if (preserveGuard) {
          preserveRestore();
        }
        ackPrefix(command);
        Serial.print(",ok=0,error=");
        Serial.println("NO_PSRAM");
        return;
      }
    } else if (sdBeginFresh() && openSdLogs()) {
      sdStarted = true;
      checkDeleteFiles = 1;
    } else {
      if (fullFile) {
        fullFile = File();
      }
      if (fiveFile) {
        fiveFile = File();
      }
      storageMode = STORAGE_NONE;
      if (preserveGuard) {
        preserveRestore();
      }
      ackPrefix(command);
      Serial.println(",ok=0,error=SD_REQUIRED_SELFTEST_FAILED");
      return;
    }
  }

  if (sdStarted) {
    printStartSd();
  }

  checkMode = (startKind == START_CHECK_FALLBACK) ? 1 : 0;
  recording = 1;
  sessionStartMs = millis();
  startMicros = micros();
  lastLiveMs = sessionStartMs - 100;
  daqLastSamples = frameSeq;
  lastDaqMs = sessionStartMs;
  daqLastQueued = fullQueued;
  daqLastPsramTotal = psramTotal;
  daqLastFullBytes = fullBytes;
  next5kCycle = ARM_DWT_CYCCNT;

  if (storageMode == STORAGE_FALLBACK) {
    startFallbackAdc();
    const uint32_t nowUs = micros();
    Serial.print("@EVENT,device=TEENSY_DAQ,type=FALLBACK_ENTERED,reason=");
    Serial.print(forcedFallback ? "SYSTEM_CHECK_FORCED_FALLBACK" : "SD_NOT_AVAILABLE_AT_START");
    printFallbackEnteredFields(nowUs);
  } else {
    startProfileAdc();
  }

  checkActive = (startKind == START_NORMAL) ? 1 : 0;
  if (checkActive) {
    setResumeArmed(1);
    resumeAtBoot = 0;
  }

  ackPrefix(command);
  Serial.print(",ok=1,mode=");
  Serial.print(storageModeName(storageMode));
  if (storageMode == STORAGE_SD) {
    Serial.print(",full=");
    Serial.print(fullName);
    Serial.print(",fivek=");
    Serial.print(fiveName);
  }
  Serial.println();
}

// 전원 재개: 부팅 뒤 1초 간격으로 기록 시작을 시도한다.
static void autoResumeTry() {
  if (millis() - lastResumeTryMs <= 999) {
    return;
  }
  lastResumeTryMs = millis();
  if (usbOpen() && Serial.availableForWrite() > 0xFF) {
    Serial.println("@EVENT,device=TEENSY_DAQ,type=POWERLOSS_AUTO_RESUME_ATTEMPT");
  }
  startRecording(START_NORMAL, "AUTO_RESUME");
  if (recording) {
    resumeAtBoot = 0;
    if (usbOpen() && Serial.availableForWrite() > 0xFF) {
      Serial.print("@EVENT,device=TEENSY_DAQ,type=POWERLOSS_AUTO_RESUME_OK,mode=");
      Serial.println(storageModeName(storageMode));
    }
  }
}

// ---------------------------------------------------------------------------
// 기록 중지
// ---------------------------------------------------------------------------
static void stopRecording(bool fromCommand) {
  if (fromCommand) {
    setResumeArmed(0);
    resumeAtBoot = 0;
  }
  if (!recording && !storageHalted) {
    Serial.println("# NOT_RECORDING");
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=STOP,ok=1,state=ALREADY_STOPPED");
    return;
  }

  recording = 0;
  adcFallbackRun = 0;
  adcActive = 0;
  stopAdc();
  dummyOn = (adcProfile == PROFILE_SPEC_SAFE) ? 1 : 0;
  dummyOverride = 0;

  if (storageMode == STORAGE_SD) {
    while (fullTail != fullHead) {
      if (!writeFullChunk()) {
        Serial.println("# WARNING: FINAL_FULL_SD_DRAIN_FAILED");
        break;
      }
    }
    while (fiveTail != fiveHead) {
      if (!writeFiveChunk()) {
        Serial.println("# WARNING: FINAL_5K_SD_DRAIN_FAILED");
        break;
      }
    }
    if (fullFile) {
      fullFile.flush();
      fullFile.close();
    }
    fullFile = File();
    if (fiveFile) {
      fiveFile.flush();
      fiveFile.close();
    }
    fiveFile = File();

    if (checkDeleteFiles) {
      if (fullName[0] != '\0' && SD.exists(fullName)) {
        SD.remove(fullName);
      }
      if (fiveName[0] != '\0' && SD.exists(fiveName)) {
        SD.remove(fiveName);
      }
      Serial.println("@EVENT,device=TEENSY_DAQ,type=SYSTEM_CHECK_SD_FILES_REMOVED");
    }
    Serial.println("# RECORDING_STOPPED,MODE=SD_DUAL");
  } else if (storageMode == STORAGE_FALLBACK) {
    usbDrainPending = 1;
    Serial.print("# RECORDING_STOPPED,MODE=PSRAM_");
    Serial.print(LOG_5K_HZ);
    Serial.println("HZ");
  }

  if (psramValid != 0) {
    psramHasData = 1;
    Serial.println("# PSRAM_FALLBACK_DATA_AVAILABLE");
    Serial.println("# WARNING: PSRAM_IS_VOLATILE");
    Serial.println("# TYPE DUMP TO TRANSFER RETAINED_FALLBACK_DATA");
    printPsramStatus();
  }

  Serial.print("# GLOBAL_SAMPLE_COUNT=");
  Serial.println(frameSeq);
  Serial.print("# SD_FULL_LOGGED_RECORDS=");
  Serial.println(fullQueued);
  Serial.print("# SD_5K_LOGGED_RECORDS=");
  Serial.println(fiveQueued);
  Serial.print("# SD_FULL_DROPS=");
  Serial.println((uint64_t)fullDrops + fullDiscarded + fullExtraDrops);
  Serial.print("# SD_5K_DROPS=");
  Serial.println((uint64_t)fiveDrops + fiveDiscarded + fiveTimelineMissing);
  Serial.print("# SD_5K_SCHEDULE_MISSES=");
  Serial.println((uint64_t)fiveScheduleMisses);
  Serial.print("# SD_SAFE_COMMIT_COUNT=");
  Serial.println(safeCommitCount);
  Serial.print("# SD_SAFE_COMMIT_DEFERRED=");
  Serial.println(safeCommitDeferred);
  Serial.print("# PSRAM_TOTAL_FALLBACK_RECORDS=");
  Serial.println((uint64_t)psramTotal);
  Serial.print("# PSRAM_RETAINED_RECORDS=");
  Serial.println(psramValid);
  Serial.print("# PSRAM_OVERWRITTEN_RECORDS=");
  Serial.println((uint64_t)psramOverwritten);
  Serial.print("# SD_TO_FALLBACK_COUNT=");
  Serial.println(sdToFallbackCount);
  Serial.print("# FALLBACK_TO_SD_COUNT=");
  Serial.println(fallbackToSdCount);
  Serial.print("# SERIAL_DROPPED_LINES=");
  Serial.println((uint64_t)usbDroppedTotal);

  storageHalted = 0;
  checkActive = 0;
  checkMode = 0;
  checkDeleteFiles = 0;
  Serial.println("@ACK,device=TEENSY_DAQ,cmd=STOP,ok=1,state=STOPPED");
}

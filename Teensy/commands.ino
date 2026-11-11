// USB 명령 처리. 명령어는 대소문자를 구분하지 않는다.

static bool cmdIs(const char* command, const char* name) {
  return strcasecmp(command, name) == 0;
}

static void stopMonitorAndAck() {
  if (monitoring) {
    monitoring = 0;
    adcActive = 0;
    stopAdc();
    Serial.println("@EVENT,device=TEENSY_DAQ,type=MONITOR_STOPPED");
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=MONITOR_OFF,ok=1,state=STOPPED");
  } else {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=MONITOR_OFF,ok=1,state=ALREADY_STOPPED");
  }
}

static void startMonitor() {
  if (recording) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=MONITOR,ok=0,error=RECORDING_ACTIVE");
    return;
  }
  noInterrupts();
  const bool streamPending = (psramTotal > usbCursor) && (psramValid != 0);
  interrupts();
  if (usbDrainPending || streamPending) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=MONITOR,ok=0,error=FALLBACK_STREAM_DRAINING");
    return;
  }
  if (monitoring) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=MONITOR,ok=1,state=ALREADY_MONITORING,mode=MONITOR,no_save=1");
    return;
  }

  adcActive = 0;
  stopAdc();
  ADC1_REG_CFG = ADC_CFG_SOFTWARE;
  ADC1_REG_GC = 0;
  ADC1_REG_HC0 = 0x1F;
  monitoring = 1;
  monitorStartMs = millis();
  lastLiveMs = millis() - 100;
  lastMonitorSampleMs = millis();
  monitorSamples = 0;
  daqLastSamples = 0;
  lastDaqMs = millis();
  sampleMonitor();
  daqLastSamples = monitorSamples;
  Serial.println("@EVENT,device=TEENSY_DAQ,type=MONITOR_STARTED,no_save=1,rate_hz=10");
  Serial.println("@ACK,device=TEENSY_DAQ,cmd=MONITOR,ok=1,mode=MONITOR,no_save=1");
  printDaq(false);
}

// PSRAM 앞뒤 32바이트에 패턴을 쓰고 읽어 비교한 뒤 원래 값으로 되돌린다.
static bool selfTestPsram() {
  if (!psramReady() || psramBuf == nullptr || psramBytes < 0x80) {
    return false;
  }
  uint8_t patternA[32], patternB[32], saveA[32], saveB[32], readA[32], readB[32];
  for (uint32_t i = 0; i < 32; i++) {
    patternA[i] = (uint8_t)((i * 13U) ^ 0xA5U);
    patternB[i] = (uint8_t)((i * 29U) ^ 0x5AU);
  }
  uint8_t* first = psramBuf;
  uint8_t* last = psramBuf + psramBytes - 0x20U;

  memcpy(saveA, first, 32);
  memcpy(saveB, last, 32);
  memcpy(first, patternA, 32);
  memcpy(last, patternB, 32);
  arm_dcache_flush_delete(first, 32);
  arm_dcache_flush_delete(last, 32);
  memcpy(readA, first, 32);
  memcpy(readB, last, 32);
  const bool ok = memcmp(readA, patternA, 32) == 0 && memcmp(readB, patternB, 32) == 0;
  memcpy(first, saveA, 32);
  memcpy(last, saveB, 32);
  arm_dcache_flush(first, 32);
  arm_dcache_flush(last, 32);
  return ok;
}

// 128 KiB를 쓰고 읽어 비교한 뒤 파일을 지운다.
static bool selfTestSd(uint8_t* present, uint32_t* bytesWritten, uint32_t* writeMs, uint32_t* readMs, uint32_t* kibPerSec) {
  static const char* const TEST_NAME = "GCSCHK.TMP";
  const uint32_t TEST_BYTES = 0x20000;
  *present = 0;
  *bytesWritten = 0;
  *writeMs = 0;
  *readMs = 0;
  *kibPerSec = 0;

  const bool started = sdBeginFresh();
  *present = started ? 1 : 0;
  if (!started) {
    return false;
  }
  if (SD.exists(TEST_NAME)) {
    SD.remove(TEST_NAME);
  }
  File file = SD.open(TEST_NAME, FILE_WRITE);
  if (!file) {
    return false;
  }

  uint8_t block[512];
  uint32_t t0 = millis();
  bool ok = true;
  uint32_t offset = 0;
  while (offset != TEST_BYTES) {
    uint8_t value = ']';
    for (uint32_t i = 0; i < 512; i++) {
      block[i] = value;
      value = (uint8_t)(value + '%');
    }
    if (file.write(block, 512) != 512) {
      ok = false;
      break;
    }
    offset += 512;
    *bytesWritten += 512;
  }
  file.flush();
  *writeMs = millis() - t0;
  file.close();

  if (!ok || *bytesWritten != TEST_BYTES || !SD.mediaPresent()) {
    if (SD.exists(TEST_NAME)) {
      SD.remove(TEST_NAME);
    }
    return false;
  }

  file = SD.open(TEST_NAME, FILE_READ);
  if (!file) {
    SD.remove(TEST_NAME);
    return false;
  }
  t0 = millis();
  uint8_t readBack[512];
  offset = 0;
  while (offset != TEST_BYTES) {
    uint8_t value = ']';
    for (uint32_t i = 0; i < 512; i++) {
      block[i] = value;
      value = (uint8_t)(value + '%');
    }
    if (file.read(readBack, 512) != 512 || memcmp(block, readBack, 512) != 0) {
      ok = false;
      break;
    }
    offset += 512;
  }
  *readMs = millis() - t0;
  file.close();
  SD.remove(TEST_NAME);

  if (*writeMs != 0) {
    *kibPerSec = (uint32_t)(((uint64_t)*bytesWritten * 1000U) / ((uint64_t)*writeMs << 10));
  }
  if (*present) {
    *present = SD.mediaPresent() ? 1 : 0;
  }
  return ok && *present;
}

static void runSelfTest() {
  if (recording || monitoring) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=CHECK_SELFTEST,ok=0,error=STOP_ACQUISITION_FIRST");
    return;
  }
  const bool psramOk = selfTestPsram();
  const uint32_t valid = psramValid;
  const uint64_t total = psramTotal;

  uint8_t sdPresent;
  uint32_t sdBytes, writeMs, readMs, kib;
  const bool sdOk = selfTestSd(&sdPresent, &sdBytes, &writeMs, &readMs, &kib);

  Serial.print("@CHECK,device=TEENSY_DAQ,kind=SELFTEST");
  Serial.print(",psram_ok=");
  Serial.print(psramOk ? 1 : 0);
  Serial.print(",psram_mib=");
  Serial.print((unsigned long)external_psram_size);
  Serial.print(",psram_capacity_records=");
  Serial.print(psramCap);
  Serial.print(",psram_valid=");
  Serial.print(valid);
  Serial.print(",psram_total=");
  Serial.print(total);
  Serial.print(",check_preserve=1");
  Serial.print(",preserve_pending=");
  Serial.print((int)preserveGuard);
  Serial.print(",sd_present=");
  Serial.print((int)sdPresent);
  Serial.print(",sd_rw_ok=");
  Serial.print(sdOk ? 1 : 0);
  Serial.print(",sd_bytes=");
  Serial.print(sdBytes);
  Serial.print(",sd_write_ms=");
  Serial.print(writeMs);
  Serial.print(",sd_read_ms=");
  Serial.print(readMs);
  Serial.print(",sd_write_kib_s=");
  Serial.println(kib);
  Serial.print("@ACK,device=TEENSY_DAQ,cmd=CHECK_SELFTEST,ok=1,psram_ok=");
  Serial.print(psramOk ? 1 : 0);
  Serial.print(",sd_rw_ok=");
  Serial.println(sdOk ? 1 : 0);
}

static void clearPsramCheck() {
  if (recording || monitoring) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=CHECK_CLEAR_PSRAM,ok=0,error=STOP_ACQUISITION_FIRST");
    return;
  }
  if (!psramReady()) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=CHECK_CLEAR_PSRAM,ok=0,error=PSRAM_UNAVAILABLE");
    return;
  }
  psramClearLogical();
  Serial.println("@ACK,device=TEENSY_DAQ,cmd=CHECK_CLEAR_PSRAM,ok=1");
}

static void restorePsramCheck() {
  if (recording || monitoring) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=CHECK_RESTORE_PSRAM,ok=0,error=STOP_ACQUISITION_FIRST");
    return;
  }
  if (!preserveGuard) {
    Serial.print("@ACK,device=TEENSY_DAQ,cmd=CHECK_RESTORE_PSRAM,ok=1,state=NO_BACKUP,valid=");
    Serial.println(psramValid);
    return;
  }
  const uint32_t savedValid = preserveBackup.valid;
  const uint64_t savedTotal = preserveBackup.total;
  noInterrupts();
  const bool streamPending = (psramTotal > usbCursor) && (psramValid != 0);
  uint32_t queued = 0;
  if (streamPending) {
    const uint64_t backlog = psramTotal - usbCursor;
    queued = (backlog > psramValid) ? psramValid : (uint32_t)backlog;
  }
  interrupts();
  if (streamPending || usbDrainPending) {
    Serial.print("@ACK,device=TEENSY_DAQ,cmd=CHECK_RESTORE_PSRAM,ok=0,error=USB_DRAIN_PENDING,queued=");
    Serial.println(queued);
    return;
  }
  preserveRestore();
  Serial.print("@ACK,device=TEENSY_DAQ,cmd=CHECK_RESTORE_PSRAM,ok=1,state=RESTORED,valid=");
  Serial.print(savedValid);
  Serial.print(",total=");
  Serial.println(savedTotal);
}

static void changeAdcProfile(uint8_t profile) {
  if (recording) {
    Serial.println("@ACK,device=TEENSY_DAQ,cmd=ADC_PROFILE,ok=0,error=RECORDING_ACTIVE");
    return;
  }
  stopAdc();
  dummyOn = profile ? 0 : 1;
  dummyOverride = 0;
  ADC1_REG_CFG = ADC_CFG_SOFTWARE;
  ADC1_REG_HC0 = 0x1F;
  adcProfile = profile;
  ADC1_REG_GC = 0;

  Serial.print("@ACK,device=TEENSY_DAQ,cmd=ADC_PROFILE,ok=1,profile=");
  Serial.print(adcProfileName());
  Serial.print(",dummy=");
  Serial.print((int)dummyOn);
  Serial.print(",target_hz=");
  if (profile) {
    Serial.print(0UL);
    Serial.print(",clock_hz=");
    Serial.print(ADC_DIRECT_CLOCK_HZ);
    Serial.print(",max_mode=");
    Serial.println("AUTO_HW_LIMIT");
  } else {
    Serial.print(SPEC_FRAME_HZ);
    Serial.print(",clock_hz=");
    Serial.print(ADC_DIRECT_CLOCK_HZ);
    Serial.print(",max_mode=");
    Serial.println("PACED");
  }
  Serial.print("@EVENT,device=TEENSY_DAQ,type=ADC_PROFILE_CHANGED,profile=");
  Serial.print(adcProfileName());
  Serial.print(",dummy=");
  Serial.println((int)dummyOn);
}

static void sdRetryCommand() {
  if (recording && storageMode == STORAGE_FALLBACK) {
    sdRecoveryService(true);
    return;
  }
  if (recording && storageMode == STORAGE_SD) {
    const int present = SD.mediaPresent() ? 1 : 0;
    Serial.print("@ACK,device=TEENSY_DAQ,cmd=SD_RETRY,ok=1,state=ALREADY_SD,sd=");
    Serial.println(present);
    return;
  }
  int present;
  if (!recording) {
    present = sdIdleProbe(true) ? 1 : 0;
  } else if (storageMode == STORAGE_FALLBACK) {
    present = sdPresentDuringFallback;
  } else {
    present = SD.mediaPresent() ? 1 : 0;
  }
  Serial.print("@ACK,device=TEENSY_DAQ,cmd=SD_RETRY,ok=1,state=PROBE_ONLY,sd=");
  Serial.println(present);
}

// 블록 쓰기가 다 나갈 때까지 조금씩 보낸다.
static void writeAllUsb(const uint8_t* data, uint32_t length) {
  while (length != 0) {
    const uint32_t chunk = (length > 0x4000U) ? 0x4000U : length;
    const uint32_t sent = Serial.write(data, chunk);
    if (sent == 0) {
      yield();
      continue;
    }
    length -= sent;
    data += sent;
  }
}

static void dumpPsram() {
  if (recording) {
    Serial.println("# ERROR: STOP_RECORDING_BEFORE_DUMP");
    return;
  }
  if (!psramHasData || psramBuf == nullptr || psramValid == 0) {
    Serial.println("# ERROR: NO_PSRAM_DATA");
    return;
  }

  printPsramStatus();
  const uint32_t valid = psramValid;
  const uint32_t oldest = (valid != 0 && psramCap != 0 && valid >= psramCap) ? psramWr : 0;
  const uint32_t totalBytes = valid * PS_RECORD_BYTES;

  uint8_t header[LOG_HEADER_BYTES];
  fillLogHeader(header);
  patchFallbackHeader(header);

  Serial.print("#DUMP_BEGIN,TOTAL_BYTES=");
  Serial.print(totalBytes + LOG_HEADER_BYTES);
  Serial.print(",VALID_RECORDS=");
  Serial.print(valid);
  Serial.print(",RECORD_BYTES=");
  Serial.print(PS_RECORD_BYTES);
  Serial.print(",TOTAL_STORED_RECORDS=");
  Serial.print((uint64_t)psramTotal);
  Serial.print(",OVERWRITTEN_RECORDS=");
  Serial.print((uint64_t)psramOverwritten);
  Serial.print(",OLDEST_INDEX=");
  Serial.println(oldest);
  Serial.flush();
  delay(20);

  writeAllUsb(header, LOG_HEADER_BYTES);
  if (valid < psramCap) {
    writeAllUsb(psramBuf, totalBytes);
  } else {
    if (psramCap != oldest) {
      writeAllUsb(psramBuf + oldest * PS_RECORD_BYTES, (psramCap - oldest) * PS_RECORD_BYTES);
    }
    if (oldest != 0) {
      writeAllUsb(psramBuf, oldest * PS_RECORD_BYTES);
    }
  }
  Serial.flush();
  Serial.println();
  Serial.println("#DUMP_END");
}

static void psramInitCommand() {
  if (recording) {
    Serial.println("# ERROR: STOP_RECORDING_BEFORE_PSRAM_INIT");
    return;
  }
  if (!psramReady()) {
    Serial.println("# ERROR: PSRAM_INIT_FAILED");
    Serial.print("# PSRAM_DETECTED_MIB=");
    Serial.println((unsigned long)external_psram_size);
    return;
  }
  psramClearLogical();
  Serial.println("# PSRAM_INIT_OK");
  Serial.println("# PSRAM_LOGICAL_CLEAR_DONE");
  Serial.print("# PSRAM_DETECTED_MIB=");
  Serial.println((unsigned long)external_psram_size);
  Serial.print("# PS_RECORD_BYTES=");
  Serial.println(PS_RECORD_BYTES);
  Serial.print("# PSRAM_CAPACITY_RECORDS=");
  Serial.println(psramCap);
}

static void handleCommand(const char* command) {
  if (command[0] == '\0') {
    return;
  }
  if (command[0] == '1' && command[1] == '\0') {
    startRecording(START_NORMAL, "START");
    return;
  }
  if (cmdIs(command, "START")) {
    startRecording(START_NORMAL, "START");
    return;
  }
  if ((command[0] == '0' && command[1] == '\0') || cmdIs(command, "STOP")) {
    if (monitoring && !recording) {
      stopMonitorAndAck();
    } else {
      stopRecording(true);
    }
    return;
  }
  if (cmdIs(command, "MONITOR") || cmdIs(command, "VIEW")) {
    startMonitor();
    return;
  }
  if (cmdIs(command, "MONITOR_OFF") || cmdIs(command, "VIEW_OFF") || cmdIs(command, "MONITOR_STOP")) {
    stopMonitorAndAck();
    return;
  }
  if (cmdIs(command, "PING") || cmdIs(command, "ID")) {
    Serial.println("@PONG,device=TEENSY_DAQ,proto=1,fw=LG07_TMS_GS6_16_SPEC975K_MAX_SD_FAST_FAILOVER");
    return;
  }
  if (cmdIs(command, "CHECK_SELFTEST")) {
    runSelfTest();
    return;
  }
  if (cmdIs(command, "CHECK_SNAPSHOT")) {
    printSnapshot();
    return;
  }
  if (cmdIs(command, "CHECK_SD_START")) {
    startRecording(START_CHECK_SD, "CHECK_SD_START");
    return;
  }
  if (cmdIs(command, "CHECK_FB_START")) {
    startRecording(START_CHECK_FALLBACK, "CHECK_FB_START");
    return;
  }
  if (cmdIs(command, "CHECK_CLEAR_PSRAM")) {
    clearPsramCheck();
    return;
  }
  if (cmdIs(command, "CHECK_RESTORE_PSRAM")) {
    restorePsramCheck();
    return;
  }
  if (cmdIs(command, "STATUS")) {
    if (!recording && !monitoring) {
      sdIdleProbe(true);
    }
    printDaq(true);
    return;
  }
  if (cmdIs(command, "ADC_PROFILE_SPEC") || cmdIs(command, "ADC SPEC") || cmdIs(command, "ADC_SPEED_SPEC")) {
    changeAdcProfile(PROFILE_SPEC_SAFE);
    return;
  }
  if (cmdIs(command, "ADC_PROFILE_MAX") || cmdIs(command, "ADC MAX") || cmdIs(command, "ADC_SPEED_MAX")) {
    changeAdcProfile(PROFILE_MAX_SPEED);
    return;
  }
  if (cmdIs(command, "ADC_PROFILE") || cmdIs(command, "ADC_SPEED")) {
    Serial.print("@ACK,device=TEENSY_DAQ,cmd=ADC_PROFILE,ok=1,profile=");
    Serial.print(adcProfileName());
    Serial.print(",dummy=");
    Serial.println((int)dummyOn);
    return;
  }
  if (cmdIs(command, "DUMMY_ON") || cmdIs(command, "DUMMY_OFF") || cmdIs(command, "DUMMY ON") || cmdIs(command, "DUMMY OFF")) {
    Serial.print("@ACK,device=TEENSY_DAQ,cmd=DUMMY,ok=0,error=PROFILE_CONTROLLED,profile=");
    Serial.println(adcProfileName());
    return;
  }
  if (cmdIs(command, "DUMMY")) {
    Serial.print("@ACK,device=TEENSY_DAQ,cmd=DUMMY,ok=1,enabled=");
    Serial.print((int)dummyOn);
    Serial.print(",profile=");
    Serial.println(adcProfileName());
    return;
  }
  if (cmdIs(command, "SD_RETRY") || cmdIs(command, "RETRY_SD") || cmdIs(command, "SD_CHECK") || cmdIs(command, "CHECK_SD")) {
    sdRetryCommand();
    return;
  }
  if (cmdIs(command, "DUMP")) {
    dumpPsram();
    return;
  }
  if (cmdIs(command, "PINIT") || cmdIs(command, "PSRAM_INIT") || cmdIs(command, "CLEARPSRAM")) {
    psramInitCommand();
    return;
  }

  Serial.print("# UNKNOWN_COMMAND=");
  Serial.println(command);
  Serial.print("@ACK,device=TEENSY_DAQ,cmd=");
  Serial.print(command);
  Serial.println(",ok=0,error=UNKNOWN_COMMAND");
}

// USB에서 명령을 읽는다. 버퍼가 비어 있을 때 온 0, 1은 줄바꿈 없이 바로 실행한다.
static void pollCommands() {
  while (Serial.available()) {
    const uint8_t c = (uint8_t)Serial.read();
    const uint8_t length = cmdLen;

    if (length == 0) {
      if (c == '0' || c == '1') {
        char single[2] = {(char)c, '\0'};
        handleCommand(single);
        continue;
      }
      if (c == '\r' || c == '\n') {
        continue;
      }
    } else {
      if (c == '\r' || c == '\n') {
        cmdBuf[length] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
        continue;
      }
      if (length > CMD_BUFFER_MAX - 1) {
        cmdLen = (c == '\n') ? 1 : 0;
        Serial.println("# ERROR: COMMAND_TOO_LONG");
        continue;
      }
    }
    cmdLen = length + 1;
    cmdBuf[length] = (char)c;
  }
}

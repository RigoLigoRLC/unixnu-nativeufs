// SPDX-License-Identifier: BSD-2-Clause-Patent
#pragma once

extern "C" IOReturn armcpu_spi_attach (uint32_t, bool, void (*) (void *), void *,
                                       uint32_t *, uint64_t *) __attribute__((weak_import));

static constexpr unsigned kRuntimeSlots = 32;
static constexpr size_t kRuntimeStride = 4096 + kDataMax;

enum class UfsPowerState : uint32_t {
  Active,
  Suspending,
  Hibern8,
  Resuming,
  ShuttingDown,
  Shutdown,
  Failed
};

static IOReturn runtimeEnsureActive ();

// UfsRequest owns one command, response and data area until its caller finishes
// copying the completed response. Failed hardware ownership keeps the whole
// allocation quarantined even after software callers have returned.
struct UfsRequest {
  uint8_t *command = nullptr;
  uint64_t physical = 0;
  IOReturn result = kIOReturnNotReady;
  uint32_t ocs = 15;
  bool submitted = false;
  bool complete = false;
  uint64_t submittedAt = 0;
  uint32_t bytes = 0;
  unsigned direction = 0;
  bool aborting = false;
};

// UfsRuntimeQueue separates the noncached UTRL from cached per-slot DMA.
// The spin lock covers admission, descriptors and completion state. Callers
// copy and authenticate data outside it, retaining their slot until finished.
struct UfsRuntimeQueue {
  IOBufferMemoryDescriptor *allocation = nullptr;
  volatile uint32_t *list = nullptr;
  IOSimpleLock *lock = nullptr;
  IOLock *powerLock = nullptr;
  UfsPowerState power = UfsPowerState::Active;
  UfsRequest requests[kRuntimeSlots];
  uint32_t leased = 0;
  unsigned count = 0;
  unsigned normalCount = 0;
  bool installed = false;
  bool offline = false;
  bool barrier = false;
  bool recovering = false;
  bool writeError = false;
  bool irq = false;
  bool forcePoll = false;
  uint32_t attaching = 0;
  uint64_t nextAttach = 0;
  uint64_t interrupts = 0;
  uint64_t submitted = 0;
  uint64_t completed = 0;
  uint64_t failed = 0;
  uint64_t admissionTimeouts = 0;
  uint64_t queueTicks = 0;
  uint64_t hardwareTicks = 0;
  uint64_t readBytes = 0;
  uint64_t writeBytes = 0;
  uint64_t timeouts = 0;
  uint64_t aborted = 0;
  uint64_t abortFailures = 0;
  uint64_t lunResets = 0;
  uint32_t failureStatus = 0;
  uint32_t failureDoorbell = 0;
  uint32_t failureHce = 0;
  uint32_t gear[9] = {};
  uint32_t gearValid = 0;
  uint32_t hardware[5] = {};
  uint32_t clocks[15] = {};
  bool clocksValid = false;
  unsigned peak = 0;
};

static UfsRuntimeQueue gQueue;
static unsigned gUfsShutdownComplete = 1;

// The failed allocation remains mapped and retained for the host lifetime.
// No slot is admitted after this transition, including the panic slot.
static void runtimeFailLocked (IOReturn error, bool notify = true)
{
  if (gQueue.offline) {
    return;
  }
  gQueue.offline = true;
  gQueue.power = UfsPowerState::Failed;
  gQueue.failureStatus = read32 (gHost.hc, 0x20);
  gQueue.failureDoorbell = read32 (gHost.hc, 0x58);
  gHost.poisoned = true;
  gHost.ready = false;
  hostWrite (0x24, 0);
  hostWrite (0x60, 0);
  hostWrite (0x80, 0);
  hostWrite (0x34, 0);
  gQueue.failureHce = read32 (gHost.hc, 0x34);
  for (unsigned index = 0; index < gQueue.count; index++) {
    UfsRequest &request = gQueue.requests[index];
    if ((gQueue.leased & (1U << index)) && !request.complete) {
      request.result = error;
      request.complete = true;
      if (notify) {
        thread_wakeup (&request);
      }
    }
  }
  if (notify) {
    thread_wakeup (&gQueue);
  }
}

static void runtimeCollectLocked (bool notify = true)
{
  if (gQueue.offline) {
    return;
  }
  uint32_t status = read32 (gHost.hc, 0x20);
  if (status & kUfsFatalStatus) {
    runtimeFailLocked (kIOReturnIOError, notify);
    return;
  }
  if (status & 1U) {
    hostWrite (0x20, 1U);
  }
  uint32_t pending = read32 (gHost.hc, 0x58);
  __asm__ volatile ("dsb sy" ::: "memory");
  for (unsigned index = 0; index < gQueue.count; index++) {
    UfsRequest &request = gQueue.requests[index];
    if (!(gQueue.leased & (1U << index)) || !request.submitted ||
        request.complete || request.aborting || (pending & (1U << index))) {
      continue;
    }
    uint32_t ocs = gQueue.list[index * 8 + 2] & 255U;
    if (ocs == 15) {
      continue;
    }
    request.ocs = ocs;
    request.result = ocs ? kIOReturnIOError : kIOReturnSuccess;
    request.complete = true;
    gQueue.completed++;
    gQueue.hardwareTicks += mach_absolute_time () - request.submittedAt;
    if (notify) {
      thread_wakeup (&request);
    }
  }
}

static void runtimeInterrupt (void *)
{
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  gQueue.interrupts++;
  runtimeCollectLocked ();
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
}

// These attributes are read only while the device is ready and the transfer
// queue is empty. Status reporting consumes the saved values without UIC I/O.
static void runtimeReadLink ()
{
  const unsigned registers[] = { 0xe4, 0xc0, 0xc4, 0xdc, 0xe0 };
  for (unsigned index = 0; index < 5; index++) {
    gQueue.hardware[index] = read32 (gHost.hc, registers[index]);
  }
  if (gHardware.axi == kSm7325Ufs.axi) {
    OSSharedPtr<IOMemoryMap> gccMap;
    uintptr_t gcc = mapPhys (GCC_BASE + 0x77000, 0x1000, gccMap);
    if (gcc) {
      const unsigned commands[] = { 0x24, 0x84, 0xa0 };
      for (unsigned clock = 0; clock < 3; clock++) {
        for (unsigned word = 0; word < 5; word++) {
          gQueue.clocks[clock * 5 + word] = read32 (gcc, commands[clock] + word * 4);
        }
      }
      gQueue.clocksValid = true;
    }
  }
  const uint16_t attributes[] = { 0x1568, 0x1583, 0x1560, 0x1580, 0x156a, 0x1571,
    0x1569, 0x1584, 0x15a9 };
  for (unsigned index = 0; index < 9; index++) {
    if (!hostWaitRegister (0x30, 8, 8)) {
      break;
    }
    hostWrite (0x20, 0x400);
    hostWrite (0x94, uint32_t (attributes[index]) << 16);
    hostWrite (0x98, 0);
    hostWrite (0x9c, 0);
    hostWrite (0x90, 1);
    if (!hostWaitRegister (0x20, 0x400, 0x400)) {
      break;
    }
    if ((read32 (gHost.hc, 0x98) & 255) == 0) {
      gQueue.gear[index] = read32 (gHost.hc, 0x9c);
      gQueue.gearValid |= 1U << index;
    }
    hostWrite (0x20, 0x400);
  }
}

// ArmCpu registration can sleep. The registration claim is taken under the
// queue lock, but the platform call runs after interrupts have been restored.
static void runtimeTryInterrupt ()
{
  if (gQueue.forcePoll || !armcpu_spi_attach || gUfsInterrupt < 32 || gUfsInterrupt >= 1020) {
    return;
  }
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  if (gQueue.offline || gQueue.irq || gQueue.attaching || mach_absolute_time () < gQueue.nextAttach) {
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    return;
  }
  gQueue.attaching = 1;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  uint32_t configuration = 0;
  uint64_t router = 0;
  IOReturn result = armcpu_spi_attach (gUfsInterrupt, false, runtimeInterrupt, nullptr,
                                     &configuration, &router);
  state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  gQueue.attaching = 0;
  clock_interval_to_deadline (100, kMillisecondScale, &gQueue.nextAttach);
  gQueue.irq = result == kIOReturnSuccess;
  if (gQueue.irq && !gQueue.offline) {
    hostWrite (0x24, 1U | kUfsFatalStatus);
  }
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (result == kIOReturnSuccess) {
    IOLog ("NativeUfs: IRQ intid=%u attached config=%x router=%llx\n",
      gUfsInterrupt, configuration, router);
  }
}

static bool runtimeQueueStart (const char **why)
{
  gQueue.count = (read32 (gHost.hc, 0) & 31U) + 1;
  if (gQueue.count < 2) {
    *why = "UTRL requires an ordinary slot and a panic slot";
    return false;
  }
  unsigned depth = gQueue.count - 1;
  unsigned poll = 0;
  PE_parse_boot_argn ("native-ufs-queue-depth", &depth, sizeof (depth));
  PE_parse_boot_argn ("native-ufs-poll", &poll, sizeof (poll));
  if (!depth || depth >= gQueue.count || poll > 1) {
    *why = "invalid native-ufs-queue-depth or native-ufs-poll";
    return false;
  }
  gQueue.normalCount = depth;
  gQueue.forcePoll = poll == 1;
  const size_t dmaBytes = 4096 + (depth + 1) * kRuntimeStride;
  gQueue.lock = IOSimpleLockAlloc ();
  gQueue.powerLock = IOLockAlloc ();
  gQueue.allocation = IOBufferMemoryDescriptor::inTaskWithOptions (
    kernel_task, static_cast<IOOptionBits> (kIODirectionInOut) |
    static_cast<IOOptionBits> (kIOMemoryPhysicallyContiguous) |
    static_cast<IOOptionBits> (kIOMemoryMapperNone), dmaBytes, 4096);
  if (!gQueue.lock || !gQueue.powerLock || !gQueue.allocation) {
    *why = "runtime queue allocation";
    return false;
  }
  IOByteCount contiguous = 0;
  uint64_t physical = gQueue.allocation->getPhysicalSegment (0, &contiguous, kIOMemoryMapperNone);
  uint8_t *memory = static_cast<uint8_t *> (gQueue.allocation->getBytesNoCopy ());
  if (!memory || !physical || (physical & 4095) || contiguous < dmaBytes ||
      (!(read32 (gHost.hc, 0) & 0x01000000) && physical + dmaBytes > 0x100000000ULL)) {
    *why = "runtime DMA address/contiguity";
    return false;
  }
  UfsSmmuPlan plan = planUfsSmmu (gHardware.stream, physical, dmaBytes,
    [](unsigned offset) { return read32 (gHost.smmu, offset); }, gHardware.smmuSize);
  if (plan.error) {
    *why = plan.error;
    return false;
  }
  bzero (memory, dmaBytes);
  cacheCivac (memory, dmaBytes);
  // The pmap operation updates the managed page and all tracked aliases,
  // including the physical aperture, and synchronizes caches and translations.
  // Adjacent 32-byte UTRDs must never share a dirty CPU cache line.
  ppnum_t listPage = static_cast<ppnum_t> (physical >> PAGE_SHIFT);
  pmap_set_cache_attributes (listPage, VM_WIMG_WCOMB);
  if ((pmap_cache_attributes (listPage) & VM_WIMG_MASK) != VM_WIMG_WCOMB) {
    *why = "runtime UTRL cache attribute";
    return false;
  }
  gQueue.list = reinterpret_cast<volatile uint32_t *> (memory);
  for (unsigned index = 0; index <= gQueue.normalCount; index++) {
    gQueue.requests[index].command = memory + 4096 + index * kRuntimeStride;
    gQueue.requests[index].physical = physical + 4096 + index * kRuntimeStride;
  }
  runtimeReadLink ();
  if (!hostStopQueue (why)) {
    return false;
  }
  hostWrite (0x80, 0);
  if (!hostWaitRegister (0x80, 1, 0)) {
    *why = "task list stop";
    return false;
  }
  hostWrite (0x50, static_cast<uint32_t> (physical));
  hostWrite (0x54, static_cast<uint32_t> (physical >> 32));
  hostWrite (0x70, static_cast<uint32_t> (physical + 1024));
  hostWrite (0x74, static_cast<uint32_t> ((physical + 1024) >> 32));
  hostWrite (0x80, 1);
  hostWrite (0x4c, 0); // Each interrupt-marked UTRD completes without aggregation.
  hostWrite (0x20, read32 (gHost.hc, 0x20));
  hostWrite (0x60, 1);
  if (!hostWaitRegister (0x60, 1, 1) || !hostWaitRegister (0x80, 1, 1) ||
      read32 (gHost.hc, 0x50) != uint32_t (physical) ||
      read32 (gHost.hc, 0x54) != uint32_t (physical >> 32) ||
      read32 (gHost.hc, 0x70) != uint32_t (physical + 1024) ||
      read32 (gHost.hc, 0x74) != uint32_t ((physical + 1024) >> 32)) {
    *why = "runtime request list start";
    runtimeFailLocked (kIOReturnIOError);
    return false;
  }
  __atomic_store_n (&gUfsShutdownComplete, 0U, __ATOMIC_RELEASE);
  gQueue.installed = true;
  runtimeTryInterrupt ();
  IOLog ("NativeUfs: legacy UTRL slots=%u normal=%u panic=0 DMA=%llx bytes=%llx completion=%s\n",
    gQueue.count, gQueue.normalCount, physical, uint64_t (dmaBytes), gQueue.irq ? "IRQ" : "poll");
  return true;
}

// Wait registration occurs under the same lock used by completion, preventing
// an interrupt between checking the predicate and placing the caller to sleep.
static void runtimeWaitLocked (void *event, uint64_t deadline, IOInterruptState state)
{
  if (gQueue.irq) {
    assert_wait_deadline (event, THREAD_UNINT, deadline);
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    thread_block (THREAD_CONTINUE_NULL);
  } else {
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    IODelay (20);
  }
}

// UfsAdmissionFailure records the live queue when a caller could not obtain a
// slot. Its caller prints the snapshot after releasing the queue lock.
struct UfsAdmissionFailure {
  uint64_t elapsed = 0;
  uint64_t submitted = 0;
  uint64_t completed = 0;
  uint32_t leased = 0;
  uint32_t doorbell = 0;
  uint32_t status = 0;
  uint32_t run = 0;
  uint32_t power = 0;
  bool offline = false;
  bool barrier = false;
  bool recovering = false;
};

// UfsRuntimeFailure keeps the last failed runtime operation and its admission
// snapshot separately from the polling commands used during controller startup.
struct UfsRuntimeFailure {
  const char *phase = "none";
  uint32_t slot = 0xffffffff;
  uint32_t opcode = 0;
  uint32_t lun = 0;
  uint32_t result = 0;
  uint32_t ocs = 15;
  uint64_t lba = 0;
  UfsAdmissionFailure admission;
};

static UfsRuntimeFailure gRuntimeFailure;

static uint64_t runtimeCommandLba (const uint8_t *cdb)
{
  if (cdb[0] != 0x28 && cdb[0] != 0x2a) {
    return 0;
  }
  return (uint32_t (cdb[2]) << 24) | (uint32_t (cdb[3]) << 16) |
    (uint32_t (cdb[4]) << 8) | cdb[5];
}

static int runtimeAcquire (bool barrier, uint64_t deadline, bool powerCommand,
                           UfsAdmissionFailure *failure)
{
  uint64_t began = mach_absolute_time ();
  bool ownsBarrier = false;
  for (;;) {
    IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    runtimeCollectLocked ();
    if (!powerCommand && gQueue.power != UfsPowerState::Active && !gQueue.offline) {
      if (ownsBarrier) {
        gQueue.barrier = false;
        thread_wakeup (&gQueue);
      }
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      return -2;
    }
    if (!gQueue.offline && barrier && !ownsBarrier && !gQueue.barrier) {
      gQueue.barrier = true;
      ownsBarrier = true;
    }
    if (!gQueue.offline && !gQueue.recovering && (!gQueue.barrier || ownsBarrier) &&
        (!barrier || gQueue.leased == 0)) {
      for (unsigned index = 1; index <= gQueue.normalCount; index++) {
        if (!(gQueue.leased & (1U << index))) {
          gQueue.leased |= 1U << index;
          UfsRequest &request = gQueue.requests[index];
          request.submitted = false;
          request.complete = false;
          request.aborting = false;
          request.ocs = 15;
          request.result = kIOReturnNotReady;
          gQueue.queueTicks += mach_absolute_time () - began;
          IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
          return static_cast<int> (index);
        }
      }
    }
    // A runnable caller may resume after its deadline with a slot already free.
    // Only a queue that still blocks admission can exhaust the admission budget.
    if (gQueue.offline || mach_absolute_time () >= deadline) {
      failure->elapsed = mach_absolute_time () - began;
      failure->submitted = gQueue.submitted;
      failure->completed = gQueue.completed;
      failure->leased = gQueue.leased;
      failure->doorbell = read32 (gHost.hc, 0x58);
      failure->status = read32 (gHost.hc, 0x20);
      failure->run = read32 (gHost.hc, 0x60);
      failure->power = static_cast<uint32_t> (gQueue.power);
      failure->offline = gQueue.offline;
      failure->barrier = gQueue.barrier;
      failure->recovering = gQueue.recovering;
      gQueue.queueTicks += failure->elapsed;
      gQueue.failed++;
      if (!gQueue.offline) {
        gQueue.admissionTimeouts++;
      }
      if (ownsBarrier) {
        gQueue.barrier = false;
        thread_wakeup (&gQueue);
      }
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      return -1;
    }
    runtimeWaitLocked (&gQueue, deadline, state);
  }
}

static void runtimePrepare (unsigned index, uint8_t lun, const uint8_t *cdb, unsigned direction,
                            uint32_t bytes, const void *source)
{
  UfsRequest &request = gQueue.requests[index];
  uint8_t *command = request.command;
  bzero (command, 4096);
  command[0] = 1;
  command[1] = direction == 2 ? 0x40 : direction == 1 ? 0x20 : 0;
  command[2] = lun;
  command[3] = static_cast<uint8_t> (index);
  for (unsigned byte = 0; byte < 4; byte++) {
    command[12 + byte] = static_cast<uint8_t> (bytes >> (24 - byte * 8));
  }
  memcpy (command + 16, cdb, 10);
  if (direction == 1) {
    memcpy (command + 4096, source, bytes);
  }
  uint32_t *prd = reinterpret_cast<uint32_t *> (command + kPrdtOffset);
  prd[0] = uint32_t (request.physical + 4096);
  prd[1] = uint32_t ((request.physical + 4096) >> 32);
  prd[2] = 0;
  prd[3] = bytes ? bytes - 1 : 0;
  cacheCivac (command, 4096 + bytes);
}

static void runtimeSubmitLocked (unsigned index, unsigned direction, uint32_t bytes)
{
  UfsRequest &request = gQueue.requests[index];
  volatile uint32_t *descriptor = gQueue.list + index * 8;
  descriptor[0] = 0x11000000U | (direction << 25); // Storage command and interrupt bit.
  descriptor[1] = 0;
  descriptor[2] = 15;
  descriptor[3] = 0;
  descriptor[4] = uint32_t (request.physical);
  descriptor[5] = uint32_t (request.physical >> 32);
  descriptor[6] = (512U >> 2) | ((kRspOffset >> 2) << 16);
  descriptor[7] = (bytes ? 1U : 0U) | ((kPrdtOffset >> 2) << 16);
  request.submitted = true;
  request.submittedAt = mach_absolute_time ();
  request.bytes = bytes;
  request.direction = direction;
  gQueue.submitted++;
  unsigned active = 0;
  for (unsigned slot = 0; slot < gQueue.count; slot++) {
    if ((gQueue.leased & (1U << slot)) && gQueue.requests[slot].submitted &&
        !gQueue.requests[slot].complete) {
      active++;
    }
  }
  if (active > gQueue.peak) {
    gQueue.peak = active;
  }
  hostWrite (0x58, 1U << index);
}

static IOReturn runtimeResponse (unsigned index, uint8_t lun, unsigned direction, uint32_t bytes,
                                void *destination, bool allowShortRead = false)
{
  UfsRequest &request = gQueue.requests[index];
  if (request.result != kIOReturnSuccess) {
    return request.result;
  }
  cacheInvalidate (request.command, 4096 + bytes);
  const uint8_t *response = request.command + kRspOffset;
  unsigned length = (unsigned (response[10]) << 8) | response[11];
  if (response[0] != 0x21 || response[2] != lun || response[3] != index || response[8] ||
      length > 480 || (length && (length < 2 ||
      ((unsigned (response[32]) << 8) | response[33]) > length - 2))) {
    return kIOReturnIOError;
  }
  if (response[6] || response[7]) {
    return kIOReturnIOError;
  }
  uint32_t residual = (uint32_t (response[12]) << 24) | (uint32_t (response[13]) << 16) |
    (uint32_t (response[14]) << 8) | response[15];
  bool shortRead = allowShortRead && direction == 2 &&
    (response[1] & 0x60) == 0x20 && residual < bytes;
  if (((response[1] & 0x60) || residual) && !shortRead) {
    return kIOReturnUnderrun;
  }
  if (direction == 2) {
    memcpy (destination, request.command + 4096, bytes - (shortRead ? residual : 0));
  }
  return kIOReturnSuccess;
}

// One recovery owner uses task slot zero. Both task descriptors and transfer
// descriptors reside on the same permanently noncached managed page.
static bool runtimeTask (unsigned index, uint8_t function, uint8_t *response)
{
  volatile uint32_t *task = gQueue.list + 256;
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  if (gQueue.offline || read32 (gHost.hc, 0x78)) {
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    return false;
  }
  uint8_t lun = gQueue.requests[index].command[2];
  for (unsigned word = 0; word < 20; word++) {
    task[word] = 0;
  }
  task[0] = 0x01000000;
  task[2] = 15;
  task[4] = __builtin_bswap32 (0x04000000U | (uint32_t (lun) << 8) | gQueue.count);
  task[5] = __builtin_bswap32 (uint32_t (function) << 16);
  task[7] = __builtin_bswap32 (uint32_t (lun));
  task[8] = __builtin_bswap32 (uint32_t (index));
  hostWrite (0x20, 0x200);
  hostWrite (0x78, 1);
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  uint64_t deadline;
  clock_interval_to_deadline (300, kMillisecondScale, &deadline);
  while (mach_absolute_time () < deadline) {
    state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    runtimeCollectLocked ();
    bool offline = gQueue.offline;
    bool done = !(read32 (gHost.hc, 0x78) & 1) && (task[2] & 255) != 15;
    if (done && !offline) {
      __asm__ volatile ("dsb sy" ::: "memory");
      uint32_t header = __builtin_bswap32 (task[12]);
      bool valid = (task[2] & 255) == 0 && (header >> 24) == 0x24 &&
        ((header >> 8) & 255) == lun && (header & 255) == gQueue.count;
      *response = static_cast<uint8_t> (__builtin_bswap32 (task[15]));
      hostWrite (0x20, 0x200);
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      return valid;
    }
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    if (offline) {
      return false;
    }
    IODelay (20);
  }
  // UFSHCI list-clear uses zero for the selected bit. Another task can enter
  // only after its admission check observes the task doorbell cleared.
  hostWrite (0x7c, ~1U);
  hostWaitRegister (0x78, 1, 0);
  return false;
}

static bool runtimeAbort (unsigned index)
{
  // UFS device-management WLUN requests do not support ABORT TASK.
  if (gQueue.requests[index].command[2] >= 0x80) {
    return false;
  }
  for (unsigned attempt = 0; attempt < 10; attempt++) {
    IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    runtimeCollectLocked ();
    bool complete = gQueue.requests[index].complete;
    bool offline = gQueue.offline;
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    if (complete || offline) {
      return complete && !offline;
    }
    uint8_t response = 0xff;
    if (!runtimeTask (index, 0x80, &response)) {
      return false;
    }
    if (response == 0) {
      IODelay (200);
      continue;
    }
    if (response != 8) {
      return false;
    }
    state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    runtimeCollectLocked ();
    if (gQueue.requests[index].complete || gQueue.offline) {
      bool success = !gQueue.offline;
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      return success;
    }
    gQueue.requests[index].aborting = true;
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    if (!runtimeTask (index, 1, &response) || response != 0) {
      return false;
    }
    hostWrite (0x5c, ~(1U << index));
    if (!hostWaitRegister (0x58, 1U << index, 0)) {
      return false;
    }
    state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    if (!gQueue.offline) {
      UfsRequest &request = gQueue.requests[index];
      request.result = kIOReturnTimeout;
      request.complete = true;
      gQueue.aborted++;
      thread_wakeup (&request);
    }
    bool success = !gQueue.offline;
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    return success;
  }
  return false;
}

// A successful LOGICAL UNIT RESET ends device-side tasks for this LUN. The
// transfer list must also clear before callers can release their DMA leases.
static bool runtimeResetLun (unsigned timedOut)
{
  uint8_t lun = gQueue.requests[timedOut].command[2];
  if (lun >= 0x80) {
    return false;
  }
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  runtimeCollectLocked ();
  uint32_t affected = 0;
  if (!gQueue.offline) {
    for (unsigned index = 1; index <= gQueue.normalCount; index++) {
      UfsRequest &request = gQueue.requests[index];
      if ((gQueue.leased & (1U << index)) && request.submitted &&
          !request.complete && request.command[2] == lun) {
        request.aborting = true;
        affected |= 1U << index;
      }
    }
  }
  bool offline = gQueue.offline;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (offline) {
    return false;
  }
  if (!affected) {
    return true;
  }
  uint8_t response = 0xff;
  if (!runtimeTask (timedOut, 8, &response) || response != 0) {
    return false;
  }
  hostWrite (0x5c, ~affected);
  if (!hostWaitRegister (0x58, affected, 0)) {
    return false;
  }
  state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  if (!gQueue.offline) {
    gQueue.lunResets++;
    for (unsigned index = 1; index <= gQueue.normalCount; index++) {
      if (affected & (1U << index)) {
        UfsRequest &request = gQueue.requests[index];
        request.result = index == timedOut ? kIOReturnTimeout : kIOReturnAborted;
        request.complete = true;
        thread_wakeup (&request);
      }
    }
  }
  bool recovered = !gQueue.offline;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  return recovered;
}

static IOReturn runtimeCommand (uint8_t lun, const uint8_t *cdb, unsigned direction,
                                uint32_t bytes, void *data, uint32_t timeoutUs, bool barrier = false,
                                bool powerCommand = false, bool allowShortRead = false)
{
  runtimeTryInterrupt ();
  uint64_t deadline;
  UfsAdmissionFailure admission = {};
  int acquired;
  do {
    if (!powerCommand) {
      IOReturn active = runtimeEnsureActive ();
      if (active != kIOReturnSuccess) {
        return active;
      }
    }
    // Admission can wait behind a ten-second flush or task recovery. The
    // command's hardware timeout starts only when its doorbell is submitted.
    clock_interval_to_deadline (30, kSecondScale, &deadline);
    acquired = runtimeAcquire (barrier, deadline, powerCommand, &admission);
  } while (acquired == -2);
  if (acquired < 0) {
    IOReturn result = admission.offline ? kIOReturnNotReady : kIOReturnTimeout;
    IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    gRuntimeFailure.phase = "admission";
    gRuntimeFailure.slot = 0xffffffff;
    gRuntimeFailure.opcode = cdb[0];
    gRuntimeFailure.lun = lun;
    gRuntimeFailure.lba = runtimeCommandLba (cdb);
    gRuntimeFailure.result = static_cast<uint32_t> (result);
    gRuntimeFailure.ocs = 15;
    gRuntimeFailure.admission = admission;
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    uint64_t nanoseconds = 0;
    absolutetime_to_nanoseconds (admission.elapsed, &nanoseconds);
    IOLog ("NativeUfs: admission failed lun=%u opcode=%x bytes=%u wait-us=%llu "
           "leased=%x DB=%x IS=%x RSR=%x power=%u offline=%u barrier=%u recovery=%u "
           "submitted=%llu completed=%llu\n", lun, cdb[0], bytes, nanoseconds / 1000,
      admission.leased, admission.doorbell, admission.status, admission.run, admission.power,
      admission.offline ? 1U : 0U, admission.barrier ? 1U : 0U, admission.recovering ? 1U : 0U,
      admission.submitted, admission.completed);
    return result;
  }
  unsigned index = static_cast<unsigned> (acquired);
  UfsRequest &request = gQueue.requests[index];
  runtimePrepare (index, lun, cdb, direction, bytes, data);
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  if (barrier && gQueue.writeError) {
    request.complete = true;
    request.result = kIOReturnIOError;
  } else if (!gQueue.offline) {
    runtimeSubmitLocked (index, direction, bytes);
  }
  clock_interval_to_deadline (timeoutUs, kMicrosecondScale, &deadline);
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  for (;;) {
    state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    runtimeCollectLocked ();
    if (request.complete || gQueue.offline) {
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      break;
    }
    if (mach_absolute_time () >= deadline) {
      if (gQueue.recovering) {
        uint64_t recoveryWait;
        clock_interval_to_deadline (100, kMillisecondScale, &recoveryWait);
        runtimeWaitLocked (&gQueue, recoveryWait, state);
        continue;
      }
      gQueue.timeouts++;
      gQueue.recovering = true;
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      bool recovered = runtimeAbort (index);
      if (!recovered) {
        state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
        gQueue.abortFailures++;
        IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
        recovered = runtimeResetLun (index);
      }
      state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
      if (!recovered) {
        runtimeFailLocked (kIOReturnTimeout);
      }
      gQueue.recovering = false;
      thread_wakeup (&gQueue);
      IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
      IOLog ("NativeUfs: timeout slot=%u opcode=%x task-recovery=%u\n", index, cdb[0], recovered ? 1U : 0U);
      continue;
    }
    runtimeWaitLocked (&request, deadline, state);
  }
  IOReturn result = runtimeResponse (index, lun, direction, bytes, data, allowShortRead);
  state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  if (result != kIOReturnSuccess) {
    gRuntimeFailure.phase = "command";
    gRuntimeFailure.slot = index;
    gRuntimeFailure.opcode = cdb[0];
    gRuntimeFailure.lun = lun;
    gRuntimeFailure.lba = runtimeCommandLba (cdb);
    gRuntimeFailure.result = static_cast<uint32_t> (result);
    gRuntimeFailure.ocs = request.ocs;
    gRuntimeFailure.admission = {};
    gQueue.failed++;
    if (direction == 1) {
      gQueue.writeError = true;
    }
  } else if (direction == 2) {
    gQueue.readBytes += bytes;
  } else if (direction == 1) {
    gQueue.writeBytes += bytes;
  }
  uint32_t ocs = request.ocs;
  gQueue.leased &= ~(1U << index);
  if (barrier) {
    gQueue.barrier = false;
  }
  thread_wakeup (&gQueue);
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (result != kIOReturnSuccess) {
    IOLog ("NativeUfs: slot=%u opcode=%x failed result=%x OCS=%x\n", index, cdb[0], result, ocs);
  }
  return result;
}

static void runtimeFormatFailure (char *buffer, size_t capacity, const char *reason, uint32_t result)
{
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  UfsRuntimeFailure failure = gRuntimeFailure;
  uint32_t leased = gQueue.leased;
  uint32_t doorbell = read32 (gHost.hc, 0x58);
  uint32_t status = read32 (gHost.hc, 0x20);
  uint32_t hcs = read32 (gHost.hc, 0x30);
  uint32_t hce = read32 (gHost.hc, 0x34);
  uint32_t run = read32 (gHost.hc, 0x60);
  uint64_t list = (uint64_t (read32 (gHost.hc, 0x54)) << 32) | read32 (gHost.hc, 0x50);
  uint64_t submitted = gQueue.submitted;
  uint64_t completed = gQueue.completed;
  uint64_t admissions = gQueue.admissionTimeouts;
  uint64_t timeouts = gQueue.timeouts;
  uint32_t power = static_cast<uint32_t> (gQueue.power);
  bool offline = gQueue.offline, barrier = gQueue.barrier, recovering = gQueue.recovering;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  uint64_t admissionNs = 0;
  absolutetime_to_nanoseconds (failure.admission.elapsed, &admissionNs);
  snprintf (buffer, capacity,
    "UFS ROOT FAILURE: %s\nresult=%08x runtime phase=%s\n"
    "LAST slot=%u op=%02x lun=%u lba=%llx result=%08x ocs=%x\n"
    "ADMISSION wait-us=%llu leased=%x DB=%x IS=%x RSR=%x power=%u barrier=%u recovery=%u\n"
    "LIVE UTRL=%llx leased=%x DB=%x IS=%x HCS=%x HCE=%x RSR=%x\n"
    "QUEUE submitted=%llu completed=%llu admission-timeouts=%llu command-timeouts=%llu\n"
    "STATE power=%u offline=%u barrier=%u recovery=%u\n",
    reason, result, failure.phase, failure.slot, failure.opcode, failure.lun, failure.lba,
    failure.result, failure.ocs, admissionNs / 1000, failure.admission.leased,
    failure.admission.doorbell, failure.admission.status, failure.admission.run,
    failure.admission.power, failure.admission.barrier ? 1U : 0U,
    failure.admission.recovering ? 1U : 0U,
    list, leased, doorbell, status, hcs, hce, run, submitted, completed, admissions, timeouts,
    power, offline ? 1U : 0U, barrier ? 1U : 0U, recovering ? 1U : 0U);
}

static bool runtimePanicWrite (uint64_t lba, const void *source)
{
  boolean_t interrupts = ml_set_interrupts_enabled (FALSE);
  if (!IOSimpleLockTryLock (gQueue.lock)) {
    ml_set_interrupts_enabled (interrupts);
    return false;
  }
  if (gQueue.offline || gQueue.power != UfsPowerState::Active || gQueue.leased ||
      gQueue.barrier || gQueue.recovering || read32 (gHost.hc, 0x58)) {
    IOSimpleLockUnlock (gQueue.lock);
    ml_set_interrupts_enabled (interrupts);
    return false;
  }
  gQueue.leased = 1;
  UfsRequest &request = gQueue.requests[0];
  request.complete = false;
  request.submitted = false;
  request.result = kIOReturnNotReady;
  uint8_t cdb[10] = { 0x2a, 8, uint8_t (lba >> 24), uint8_t (lba >> 16),
    uint8_t (lba >> 8), uint8_t (lba), 0, 0, 1, 0 };
  runtimePrepare (0, 0, cdb, 1, 4096, source);
  runtimeSubmitLocked (0, 1, 4096);
  uint64_t deadline;
  clock_interval_to_deadline (100, kMillisecondScale, &deadline);
  while (!request.complete && mach_absolute_time () < deadline) {
    runtimeCollectLocked (false);
    IODelay (2);
  }
  if (!request.complete) {
    runtimeFailLocked (kIOReturnTimeout, false);
  }
  bool ok = runtimeResponse (0, 0, 1, 4096, nullptr) == kIOReturnSuccess;
  gQueue.leased = 0;
  IOSimpleLockUnlock (gQueue.lock);
  ml_set_interrupts_enabled (interrupts);
  return ok;
}

// UfsQueueSnapshot contains counters copied under the queue lock. Printing and
// tick conversion occur after unlocking, in the USB command worker's context.
struct UfsQueueSnapshot {
  uint64_t submitted, completed, failed, admissionTimeouts;
  uint64_t queueTicks, hardwareTicks, readBytes, writeBytes, interrupts;
  uint64_t timeouts, aborted, abortFailures, lunResets;
  uint32_t slots, normalSlots, occupied, active, peak, status, doorbell, hce;
  uint32_t gear[9], gearValid, hardware[5], clocks[15], power;
  bool clocksValid;
  bool offline, barrier, irq, recovering, writeError;
};

extern "C" void native_ufs_dump_stats (void)
{
  if (!gQueue.installed) {
    IOLog ("NativeUfs: runtime queue not installed\n");
    return;
  }
  UfsQueueSnapshot snapshot = {};
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  snapshot.submitted = gQueue.submitted;
  snapshot.completed = gQueue.completed;
  snapshot.failed = gQueue.failed;
  snapshot.admissionTimeouts = gQueue.admissionTimeouts;
  snapshot.queueTicks = gQueue.queueTicks;
  snapshot.hardwareTicks = gQueue.hardwareTicks;
  snapshot.readBytes = gQueue.readBytes;
  snapshot.writeBytes = gQueue.writeBytes;
  snapshot.interrupts = gQueue.interrupts;
  snapshot.timeouts = gQueue.timeouts;
  snapshot.aborted = gQueue.aborted;
  snapshot.abortFailures = gQueue.abortFailures;
  snapshot.lunResets = gQueue.lunResets;
  snapshot.slots = gQueue.count;
  snapshot.normalSlots = gQueue.normalCount;
  snapshot.occupied = __builtin_popcount (gQueue.leased);
  snapshot.peak = gQueue.peak;
  snapshot.status = gQueue.failureStatus;
  snapshot.doorbell = gQueue.failureDoorbell;
  snapshot.hce = gQueue.failureHce;
  snapshot.offline = gQueue.offline;
  snapshot.barrier = gQueue.barrier;
  snapshot.irq = gQueue.irq;
  snapshot.recovering = gQueue.recovering;
  snapshot.writeError = gQueue.writeError;
  snapshot.gearValid = gQueue.gearValid;
  snapshot.power = static_cast<uint32_t> (gQueue.power);
  snapshot.clocksValid = gQueue.clocksValid;
  for (unsigned index = 0; index < 9; index++) {
    snapshot.gear[index] = gQueue.gear[index];
  }
  for (unsigned index = 0; index < 5; index++) {
    snapshot.hardware[index] = gQueue.hardware[index];
  }
  for (unsigned index = 0; index < 15; index++) {
    snapshot.clocks[index] = gQueue.clocks[index];
  }
  for (unsigned index = 0; index < gQueue.count; index++) {
    if ((gQueue.leased & (1U << index)) && gQueue.requests[index].submitted &&
        !gQueue.requests[index].complete) {
      snapshot.active++;
    }
  }
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  uint64_t queueNs = 0, hardwareNs = 0;
  absolutetime_to_nanoseconds (snapshot.queueTicks, &queueNs);
  absolutetime_to_nanoseconds (snapshot.hardwareTicks, &hardwareNs);
  IOLog ("NativeUfs: submitted=%llu completed=%llu failed=%llu read-bytes=%llu write-bytes=%llu "
         "slots=%u normal=%u occupied=%u active=%u peak-active=%u IRQ=%u interrupts=%llu\n",
    snapshot.submitted, snapshot.completed, snapshot.failed, snapshot.readBytes, snapshot.writeBytes,
    snapshot.slots, snapshot.normalSlots, snapshot.occupied, snapshot.active, snapshot.peak,
    snapshot.irq ? 1U : 0U, snapshot.interrupts);
  IOLog ("NativeUfs: queue-us=%llu hardware-us=%llu admission-timeouts=%llu command-timeouts=%llu "
         "aborted=%llu abort-failures=%llu lun-resets=%llu offline=%u barrier=%u recovering=%u write-error=%u "
         "failure-IS=%x failure-DB=%x HCE-after-stop=%x\n",
    queueNs / 1000, hardwareNs / 1000, snapshot.admissionTimeouts, snapshot.timeouts,
    snapshot.aborted, snapshot.abortFailures, snapshot.lunResets,
    snapshot.offline ? 1U : 0U, snapshot.barrier ? 1U : 0U,
    snapshot.recovering ? 1U : 0U, snapshot.writeError ? 1U : 0U,
    snapshot.status, snapshot.doorbell, snapshot.hce);
  IOLog ("NativeUfs: link-valid=%x TXGEAR=%u RXGEAR=%u TXLANES=%u RXLANES=%u HSSERIES=%u PWRMODE=%x\n",
    snapshot.gearValid, snapshot.gear[0], snapshot.gear[1], snapshot.gear[2],
    snapshot.gear[3], snapshot.gear[4], snapshot.gear[5]);
  IOLog ("NativeUfs: power-state=%u TXTERM=%u RXTERM=%u UNIPRO=%x vendor-version=%x "
         "SYS1CLK=%x SYMBOLCLK=%x CFG1=%x CFG2=%x\n", snapshot.power,
    snapshot.gear[6], snapshot.gear[7], snapshot.gear[8], snapshot.hardware[0],
    snapshot.hardware[1], snapshot.hardware[2], snapshot.hardware[3], snapshot.hardware[4]);
  if (snapshot.clocksValid) {
    const char *names[] = { "axi", "unipro", "aux" };
    for (unsigned index = 0; index < 3; index++) {
      const uint32_t *values = snapshot.clocks + index * 5;
      IOLog ("NativeUfs: saved RCG %s cmd=%x cfg=%x m=%x n=%x d=%x\n",
        names[index], values[0], values[1], values[2], values[3], values[4]);
    }
  }
}

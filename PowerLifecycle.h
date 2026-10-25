// SPDX-License-Identifier: BSD-2-Clause-Patent
#pragma once

extern "C" int native_ram_block_shutdown (void) __attribute__((weak_import));
extern "C" int native_boot_log_shutdown (void) __attribute__((weak_import));

// The final platform reset callback reads this after IOKit shutdown, including
// when CPU interrupts are disabled. An uninstalled controller has no pending I/O.
extern "C" int native_ufs_shutdown_complete (void)
{
  return __atomic_load_n (&gUfsShutdownComplete, __ATOMIC_ACQUIRE) ? 1 : 0;
}

// Power operations hold powerLock, block ordinary admission and drain leases
// before UIC access. Firmware-provided rails, reference clocks and PA settings
// remain active across HIBERN8. The platform owns its system sleep callbacks.
static IOReturn runtimePowerUic (uint8_t command, uint32_t completion)
{
  if (read32 (gHost.hc, 0x58) || read32 (gHost.hc, 0x78) ||
      !hostWaitRegister (0x30, 8, 8)) {
    return kIOReturnNotReady;
  }
  hostWrite (0x20, 0x400 | completion);
  hostWrite (0x94, 0);
  hostWrite (0x98, 0);
  hostWrite (0x9c, 0);
  hostWrite (0x90, command);
  uint64_t deadline;
  clock_interval_to_deadline (500, kMillisecondScale, &deadline);
  uint32_t seen = 0;
  do {
    IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
    runtimeCollectLocked ();
    bool offline = gQueue.offline;
    seen |= read32 (gHost.hc, 0x20);
    IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
    if (offline) {
      return kIOReturnIOError;
    }
    if ((seen & (0x400 | completion)) == (0x400 | completion)) {
      uint32_t result = read32 (gHost.hc, 0x98) & 255;
      uint32_t power = (read32 (gHost.hc, 0x30) >> 8) & 7;
      hostWrite (0x20, 0x400 | completion);
      return !result && power == 1 ? kIOReturnSuccess : kIOReturnIOError;
    }
    IODelay (20);
  } while (mach_absolute_time () < deadline);
  return kIOReturnTimeout;
}

static void runtimeSetPower (UfsPowerState power)
{
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  if (!gQueue.offline) {
    gQueue.power = power;
  }
  thread_wakeup (&gQueue);
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
}

static IOReturn runtimePowerFailed (IOReturn error)
{
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  runtimeFailLocked (error);
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  return error;
}

static IOReturn runtimeResumeLocked ()
{
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  UfsPowerState power = gQueue.power;
  bool offline = gQueue.offline;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (offline) {
    return kIOReturnNotReady;
  }
  if (power == UfsPowerState::Active) {
    return kIOReturnSuccess;
  }
  if (power != UfsPowerState::Hibern8) {
    return kIOReturnNotReady;
  }
  runtimeSetPower (UfsPowerState::Resuming);
  IOReturn result = runtimePowerUic (0x18, 0x20);
  if (result != kIOReturnSuccess) {
    return runtimePowerFailed (result);
  }
  uint8_t tur[10] = {};
  result = runtimeCommand (0, tur, 0, 0, nullptr, 1000000, true, true);
  if (result != kIOReturnSuccess) {
    return runtimePowerFailed (result);
  }
  runtimeSetPower (UfsPowerState::Active);
  return kIOReturnSuccess;
}

static IOReturn runtimeEnsureActive ()
{
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  bool active = !gQueue.offline && gQueue.power == UfsPowerState::Active;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (active) {
    return kIOReturnSuccess;
  }
  IOLockLock (gQueue.powerLock);
  IOReturn result = runtimeResumeLocked ();
  IOLockUnlock (gQueue.powerLock);
  return result;
}

// The explicit suspend entry drains issued writes and enters link HIBERN8.
// An ordinary request arriving afterward exits HIBERN8 before taking a slot.
extern "C" int native_ufs_suspend (void)
{
  if (!gQueue.installed) {
    return 0;
  }
  IOLockLock (gQueue.powerLock);
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  UfsPowerState power = gQueue.power;
  bool offline = gQueue.offline;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (!offline && power == UfsPowerState::Hibern8) {
    IOLockUnlock (gQueue.powerLock);
    return 1;
  }
  if (offline || power != UfsPowerState::Active) {
    IOLockUnlock (gQueue.powerLock);
    return 0;
  }
  runtimeSetPower (UfsPowerState::Suspending);
  uint8_t sync[10] = { 0x35 };
  IOReturn result = runtimeCommand (0, sync, 0, 0, nullptr, 10000000, true, true);
  if (result == kIOReturnSuccess) {
    result = runtimePowerUic (0x17, 0x40);
    if (result == kIOReturnSuccess) {
      runtimeSetPower (UfsPowerState::Hibern8);
    } else {
      runtimePowerFailed (result);
    }
  } else {
    runtimeSetPower (UfsPowerState::Active);
  }
  IOLockUnlock (gQueue.powerLock);
  IOLog ("NativeUfs: suspend result=%x\n", result);
  return result == kIOReturnSuccess ? 1 : 0;
}

extern "C" int native_ufs_resume (void)
{
  if (!gQueue.installed) {
    return 0;
  }
  return runtimeEnsureActive () == kIOReturnSuccess ? 1 : 0;
}

// Shutdown stops the Data and log producers while UFS still accepts their final
// writes. The platform calls this before disabling CPU interrupts or clocks.
// Return zero means the producer flush or controller shutdown did not finish.
extern "C" int native_ufs_shutdown (void)
{
  if (!gQueue.installed) {
    return 0;
  }
  if (&native_ram_block_shutdown && !native_ram_block_shutdown ()) {
    IOLog ("NativeUfs: shutdown Data drain failed\n");
    return 0;
  }
  if (&native_boot_log_shutdown && !native_boot_log_shutdown ()) {
    IOLog ("NativeUfs: shutdown BootLog drain failed\n");
    return 0;
  }
  IOLockLock (gQueue.powerLock);
  IOInterruptState state = IOSimpleLockLockDisableInterrupt (gQueue.lock);
  bool stopped = gQueue.power == UfsPowerState::Shutdown;
  IOSimpleLockUnlockEnableInterrupt (gQueue.lock, state);
  if (stopped) {
    IOLockUnlock (gQueue.powerLock);
    return 1;
  }
  IOReturn result = runtimeResumeLocked ();
  if (result != kIOReturnSuccess) {
    IOLockUnlock (gQueue.powerLock);
    return 0;
  }
  runtimeSetPower (UfsPowerState::ShuttingDown);
  uint8_t sync[10] = { 0x35 };
  result = runtimeCommand (0, sync, 0, 0, nullptr, 10000000, true, true);
  if (result == kIOReturnSuccess) {
    uint8_t requestSense[10] = { 0x03, 0, 0, 0, 96 };
    uint8_t sense[96] = {};
    result = runtimeCommand (0xd0, requestSense, 2, sizeof (sense), sense, 1000000, true, true, true);
  }
  if (result == kIOReturnSuccess) {
    uint8_t powerdown[10] = { 0x1b, 0, 0, 0, 0x30 };
    result = runtimeCommand (0xd0, powerdown, 0, 0, nullptr, 10000000, true, true);
  }
  if (result == kIOReturnSuccess) {
    result = runtimePowerUic (0x17, 0x40);
  }
  if (result == kIOReturnSuccess) {
    hostWrite (0x24, 0);
    hostWrite (0x60, 0);
    hostWrite (0x80, 0);
    if (!hostWaitRegister (0x60, 1, 0) || !hostWaitRegister (0x80, 1, 0)) {
      result = kIOReturnTimeout;
    }
  }
  if (result == kIOReturnSuccess) {
    runtimeSetPower (UfsPowerState::Shutdown);
    __atomic_store_n (&gUfsShutdownComplete, 1U, __ATOMIC_RELEASE);
  } else {
    runtimePowerFailed (result);
  }
  IOLockUnlock (gQueue.powerLock);
  IOLog ("NativeUfs: shutdown result=%x\n", result);
  return result == kIOReturnSuccess ? 1 : 0;
}

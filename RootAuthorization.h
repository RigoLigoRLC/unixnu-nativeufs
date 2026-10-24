// SPDX-License-Identifier: BSD-2-Clause-Patent
#pragma once

extern "C" {
int native_verified_block_root_authorized (dev_t device);
int native_verified_block_manifest_sha384 (dev_t device, const char *uuid, uint8_t digest[48]);
int native_verified_block_volume_device (const char *uuid, dev_t *device, char *path, size_t size);
}

// The mounted media must have one verified UFS window among its ancestors.
// UUID comparison is performed by that window against its signed policy.
static bool ufsAuthorizeMedia (IOService *media, const char *uuid, uint8_t *digest)
{
  IORegistryIterator *parents = IORegistryIterator::iterateOver (
    media, gIOServicePlane, kIORegistryIterateRecursively | kIORegistryIterateParents);
  if (!parents) {
    return false;
  }

  bool authenticated = false;
  unsigned windows = 0;
  unsigned depth = 0;
  while (parents->isValid () && depth < 256) {
    OSObject *entry = parents->getNextObject ();
    if (!entry) {
      break;
    }
    depth++;
    IOService *provider = OSDynamicCast (IOService, entry);
    if (!provider) {
      continue;
    }
    int result = native_ufs_block_authenticates (provider, uuid, digest);
    if (result >= 0) {
      windows++;
      authenticated = result == 1;
    }
  }
  bool valid = parents->isValid () && depth < 256;
  parents->release ();
  return valid && authenticated && windows == 1;
}

static int ufsVerifiedVolumeQuery (dev_t device, const char *expectedUuid, uint8_t *digest)
{
  if (device == NODEV) {
    return 0;
  }
  OSDictionary *matching = IOService::serviceMatching ("IOMedia");
  OSDictionary *properties = OSDictionary::withCapacity (2);
  OSNumber *majorNumber = OSNumber::withNumber (major (static_cast<uint32_t> (device)), 32);
  OSNumber *minorNumber = OSNumber::withNumber (minor (static_cast<uint32_t> (device)), 32);
  bool ready = matching && properties && majorNumber && minorNumber &&
    properties->setObject (kIOBSDMajorKey, majorNumber) &&
    properties->setObject (kIOBSDMinorKey, minorNumber) &&
    matching->setObject ("IOPropertyMatch", properties);
  OSSafeReleaseNULL (minorNumber);
  OSSafeReleaseNULL (majorNumber);
  OSSafeReleaseNULL (properties);
  OSIterator *iterator = ready ? IOService::getMatchingServices (matching) : nullptr;
  OSSafeReleaseNULL (matching);
  if (!iterator) {
    return 0;
  }

  unsigned matches = 0;
  bool authenticated = false;
  while (OSObject *object = iterator->getNextObject ()) {
    IOService *media = OSDynamicCast (IOService, object);
    if (!media) {
      continue;
    }
    if (++matches != 1) {
      authenticated = false;
      break;
    }
    OSString *uuid = OSDynamicCast (OSString, media->getProperty ("UUID"));
    if (!uuid || uuid->getLength () != 36 ||
        (expectedUuid && strcmp (expectedUuid, uuid->getCStringNoCopy ()))) {
      continue;
    }
    authenticated = ufsAuthorizeMedia (media, uuid->getCStringNoCopy (), digest);
  }
  iterator->release ();
  if (!authenticated || matches != 1) {
    if (digest) {
      bzero (digest, SHA384_DIGEST_LENGTH);
    }
    return 0;
  }
  return 1;
}

extern "C" int native_verified_block_root_authorized (dev_t device)
{
  return ufsVerifiedVolumeQuery (device, nullptr, nullptr);
}

extern "C" int native_verified_block_manifest_sha384 (dev_t device, const char *uuid, uint8_t digest[48])
{
  if (!digest) {
    return 0;
  }
  bzero (digest, SHA384_DIGEST_LENGTH);
  if (!uuid || strlen (uuid) != 36) {
    return 0;
  }
  return ufsVerifiedVolumeQuery (device, uuid, digest);
}

extern "C" int native_verified_block_volume_device (const char *uuid, dev_t *device, char *path, size_t size)
{
  if (!uuid || strlen (uuid) != 36 || !device || !path || size < 6) {
    return 0;
  }
  *device = NODEV;
  path[0] = '\0';
  OSDictionary *matching = IOService::serviceMatching ("IOMedia");
  OSDictionary *properties = OSDictionary::withCapacity (1);
  OSString *wanted = OSString::withCString (uuid);
  bool ready = matching && properties && wanted && properties->setObject ("UUID", wanted) &&
    matching->setObject ("IOPropertyMatch", properties);
  OSSafeReleaseNULL (wanted);
  OSSafeReleaseNULL (properties);
  OSIterator *iterator = ready ? IOService::getMatchingServices (matching) : nullptr;
  OSSafeReleaseNULL (matching);
  if (!iterator) {
    return 0;
  }

  unsigned matches = 0;
  bool authenticated = false;
  dev_t candidate = NODEV;
  char candidatePath[64] = {};
  while (OSObject *object = iterator->getNextObject ()) {
    IOService *media = OSDynamicCast (IOService, object);
    if (!media) {
      continue;
    }
    if (++matches != 1) {
      authenticated = false;
      break;
    }
    OSString *identity = OSDynamicCast (OSString, media->getProperty ("UUID"));
    OSString *name = OSDynamicCast (OSString, media->getProperty (kIOBSDNameKey));
    OSNumber *majorNumber = OSDynamicCast (OSNumber, media->getProperty (kIOBSDMajorKey));
    OSNumber *minorNumber = OSDynamicCast (OSNumber, media->getProperty (kIOBSDMinorKey));
    if (!identity || strcmp (identity->getCStringNoCopy (), uuid) || !name ||
        !majorNumber || !minorNumber || !name->getLength () ||
        name->getLength () >= sizeof (candidatePath) - 6 || name->getLength () >= size - 6 ||
        majorNumber->unsigned64BitValue () > 255 || minorNumber->unsigned64BitValue () > 0xffffff) {
      continue;
    }
    candidate = static_cast<dev_t> (makedev (majorNumber->unsigned32BitValue (), minorNumber->unsigned32BitValue ()));
    if (!ufsVerifiedVolumeQuery (candidate, uuid, nullptr)) {
      continue;
    }
    snprintf (candidatePath, sizeof (candidatePath), "/dev/%s", name->getCStringNoCopy ());
    authenticated = true;
  }
  iterator->release ();
  if (!authenticated || matches != 1) {
    return 0;
  }
  *device = candidate;
  strlcpy (path, candidatePath, size);
  return 1;
}

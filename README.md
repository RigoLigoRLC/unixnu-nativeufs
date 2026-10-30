# NativeUfs

Native UFS storage driver. Owns the host controller, SMMU, queues and the root read path.

Current state: the native UFS root works on sheng.

SM7325 has controller selection, GCC register diagnostics and configurable SMMU
addressing. Its firmware-to-XNU handoff still requires device execution.

The early root-device personality reads `/arm-io/ufshc` directly from the device
tree before accessing MMIO. SM8550 uses its existing register configuration when
the node contains `qcom,sm8550-ufshc`. SM7325 requires these ADT properties:

| Property | Encoding | SM7325 value |
| --- | --- | --- |
| `compatible` | NUL-terminated string | `qcom,sm7325-ufshc` |
| `reg` | Two little-endian 64-bit integers: bus address, length | `0x01d84000`, `0x3000` with an identity bus mapping |
| `native-ufs-smmu-reg` | Two little-endian 64-bit integers: physical address, length | `0x15000000`, `0x100000` |
| `native-ufs-stream-id` | One little-endian 32-bit integer | `0x80` |

`/arm-io/ranges` contains one little-endian 64-bit triple: child bus address,
physical address, length. NativeUfs translates `reg` through that range. The SMMU
property already contains a physical address. Missing or malformed SM7325
properties stop driver startup before MMIO.

The SM7325 constants come from Samsung's `yupik.dtsi`, `msm-arm-smmu-yupik.dtsi`,
`shima-gdsc.dtsi` and `gcc-yupik.c`. Firmware initializes the UFS clocks, power
rails and PHY. NativeUfs requires an enabled controller, idle queues and a usable
SMMU context bank at handoff. Its root-media implementation sends READ CAPACITY(10)
to LUN 0 after TEST UNIT READY, requires 4096-byte logical blocks, and bounds the
GPT backup header by the returned capacity before reading the root manifest.

The SM-T733 CHN T733ZCS9DYE1 stock DTB 0 with DTBO 1 confirms the controller and
SMMU ranges, stream ID 0x80, coherent DMA and QMP v4 Yupik PHY. Its PIT uses
4096-byte units and places USERDATA at block 2320896 with a capacity-dependent
length. The rooted SM-T733 acquisition on 2026-10-22 maps sda through sdf to SCSI
LUN 0 through 5, each with 4096-byte blocks. LUN 0 contains 31099904 blocks;
userdata occupies blocks 2320896 through 31099898. NativeUfs reads the current GPT
from the device when publishing root media.

`scripts/build-arm-driver.py NativeUfs --tree <device-build-tree>` builds
`build/native-ufs/libNativeUfs.a` using the device's XNU compiler recipe and the
headers prepared by `scripts/prepare-storage-headers.py`.

Linked into XNU by `unixnu build kernel`.

The USB `ufs` command takes a bounded software snapshot with one queue-lock
attempt. Occupied slots report the owning thread, opcode, direction, byte count,
buffer-copy or completion phase, submitted/completed flags and lease age. The
phase stores reuse the existing request lifetime; lease timing reuses admission
accounting. Formatting runs after releasing the queue lock. An optional kernel
diagnostic also reports allocation zones with expansion waiters and zone 50,
including their expansion owners. These snapshots do not change admission or
request retirement.

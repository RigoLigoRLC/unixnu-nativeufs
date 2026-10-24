// SPDX-License-Identifier: BSD-2-Clause-Patent
// UFS clock registers from gcc-sm8550.c and Samsung's gcc-yupik.c.
// NativeUfs reads these registers while firmware owns clock and power setup.
#ifndef NATIVE_UFS_GCC_UFS_H
#define NATIVE_UFS_GCC_UFS_H

// Physical windows from the sheng board DT (clock-controller@100000, ufshc@1d84000).
#define GCC_BASE                      0x00100000ULL
#define UFS_HC_BASE                   0x01d84000ULL
#define UFS_HC_SIZE                   0x3000ULL

#define GCC_UFS_PHY_GDSCR             0x77004
#define GCC_UFS_MEM_PHY_GDSCR         0x9e000

#define GCC_UFS_PHY_AXI_ENABLE        0x77018
#define GCC_UFS_PHY_AHB_ENABLE        0x77024
#define GCC_UFS_PHY_TX_SYMBOL0_ENABLE 0x77028
#define GCC_UFS_PHY_RX_SYMBOL0_ENABLE 0x7702c
#define GCC_UFS_PHY_UNIPRO_CORE_ENABLE 0x77068
#define GCC_UFS_PHY_ICE_CORE_ENABLE   0x77074
#define GCC_UFS_PHY_PHY_AUX_ENABLE    0x770b0
#define GCC_UFS_PHY_RX_SYMBOL1_ENABLE 0x770cc
// bus_aggr_clk in the board DT: gcc_aggre_ufs_phy_axi_clk, halt/enable 0x770e4, HW clock gating bit1
#define GCC_AGGRE_UFS_PHY_AXI_ENABLE  0x770e4

#define GCC_UFS_BRANCH_ENABLE_BIT     0x1U

// UfsHardware describes one controller and its DMA stream. NativeUfs selects the
// clock offsets from the compatible string, then reads the SM7325 controller and
// SMMU ranges and stream ID from the device tree before accessing hardware.
struct UfsHardware {
  uint64_t controllerBase;
  uint64_t controllerSize;
  uint64_t smmuBase;
  uint64_t smmuSize;
  uint16_t stream;
  uint32_t axi;
  uint32_t ahb;
  uint32_t unipro;
  uint32_t txSymbol;
  uint32_t rxSymbol;
  uint32_t phyAux;
  uint32_t memPhyGdsc; // Zero when this SoC has no separate UFS memory-PHY GDSC.
  bool axiHardwareGating;
};

static const UfsHardware kSm8550Ufs = {
  UFS_HC_BASE, UFS_HC_SIZE, 0x15000000ULL, 0xd0000, 0x60,
  GCC_UFS_PHY_AXI_ENABLE, GCC_UFS_PHY_AHB_ENABLE,
  GCC_UFS_PHY_UNIPRO_CORE_ENABLE, GCC_UFS_PHY_TX_SYMBOL0_ENABLE,
  GCC_UFS_PHY_RX_SYMBOL0_ENABLE, GCC_UFS_PHY_PHY_AUX_ENABLE,
  GCC_UFS_MEM_PHY_GDSCR, false
};

// SM7325 ranges and stream ID are filled from ADT. gcc-yupik.c supplies the
// branch offsets; shima-gdsc.dtsi supplies the shared UFS GDSC at GCC + 0x77004.
static const UfsHardware kSm7325Ufs = {
  0, 0, 0, 0, 0,
  0x77010, 0x77018, 0x7705c, 0x7701c, 0x77020, 0x7709c,
  0, true
};

// gcc_ufs_phy_ahb_clk and gcc_aggre_ufs_phy_axi_clk set hwcg_bit=1 in gcc-sm8550.c, so CLK_OFF=1 on
// an idle branch is normal (hardware 2026-10-17 read ahb=80000003).
// The reset and RPMh/TCSR/QMP-PHY programming are not represented here: their
// semantics are not qualified and NativeUfs must not write them speculatively.
#endif

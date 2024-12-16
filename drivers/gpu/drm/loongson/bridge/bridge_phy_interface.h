/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __BRIDGE_PHY_INTERFACE__
#define __BRIDGE_PHY_INTERFACE__

/**
 * @file Bridge phy interface header file
 * @brief All bridge phy chips support with their APIs
 *
 */

int bridge_phy_it66121_init(struct bridge_resource *res);
int bridge_phy_it66121_remove(struct bridge_phy *phy);

int bridge_phy_lt8618_init(struct bridge_resource *res);
int bridge_phy_lt8618_remove(struct bridge_phy *phy);

int bridge_phy_ms7210_init(struct bridge_resource *res);
int bridge_phy_ms7210_remove(struct bridge_phy *phy);

int bridge_phy_ncs8805_init(struct bridge_resource *res);
int bridge_phy_ncs8805_remove(struct bridge_phy *phy);

int bridge_phy_lt8619_init(struct bridge_resource *res);
int bridge_phy_lt8619_remove(struct bridge_phy *phy);

int bridge_phy_lt9721_init(struct bridge_resource *res);
#endif /* __BRIDGE_PHY_INTERFACE__ */

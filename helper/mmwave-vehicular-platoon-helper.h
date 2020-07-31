/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2020 University of Padova, Dep. of Information Engineering,
*   SIGNET lab.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   Author: Umberto Paro <umberto.paro@studenti.unipd.it>
*/

#pragma once

#include "ns3/mmwave-vehicular.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/mmwave-vehicular-propagation-loss-model.h"
#include "ns3/mmwave-vehicular-net-device.h"

namespace ns3 {

namespace millicar {

/**
 * This class is used for the creation of MmWaveVehicularNetDevices, their
 * configuration, position and mobility model and the channel condition
 * among the group
 */
class MmWaveVehicularPlatoonHelper : public Object
{
public:
  /**
   * Constructor
   */
  MmWaveVehicularPlatoonHelper (void);

  /**
   * Destructor
   */
  virtual ~MmWaveVehicularPlatoonHelper (void);

  static TypeId GetTypeId (void);

  /**
   * Set the position and mobility model of the nodes in the platoon
   *
   * \param nodes the nodes to be assigned to a platoon
   */
  void CreatePlatoon (NodeContainer nodes);

  /**
   * Set the channel condition to LOS between consecutive vehicles
   *
   * \param c a NetDeviceContainer with the NetDevices corresponding to vehicles in the platoon
   */
  void SetChannelCondition (NetDeviceContainer c);

  /**
   * \brief Set the routing tables of vehicles in the platoon.
   *
   * Communication is allowed only on adjacent nodes, the platoon leader is the
   * default gateway.
   *
   * \param ipv4Interfaces the container of IPv4 interfaces
   */
  void SetRoutingTables (Ipv4InterfaceContainer ipv4Interfaces);

  /**
   * Set the PropagationLossModel for this helper
   *
   * \param propagationLossModel the PropagationLossModel
   */
  void SetPropagationLossModel (Ptr<PropagationLossModel> propagationLossModel);

  /**
   * Install a MmWaveVehicularNetDevice on each node in the container, set their
   * position, mobility model and setup routing
   */
  // NetDeviceContainer InstallMmWaveVehicularNetDevices (NodeContainer nodes);

private:
  Ptr<MobilityModel> GetMobilityForDevice (Ptr<MmWaveVehicularNetDevice> dev);

  double    m_distance;   //!< Distance between vehicles
  double    m_minX;       //!< X coordinate of the first vehicle
  double    m_minY;       //!< Y coordinate of the first vehicle
  double    m_speed;      //!< Speed of the platoon vehicles
  Ptr<PropagationLossModel> m_propagationLossModel;

}; // class MmWaveVehicularPlatoonHelper

} // namespace millicar

} // namespace ns3

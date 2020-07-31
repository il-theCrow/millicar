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

#include "mmwave-vehicular-platoon-helper.h"
#include "ns3/log.h"
#include "ns3/config.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-static-routing-helper.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveVehicularPlatoonHelper");

namespace millicar {

NS_OBJECT_ENSURE_REGISTERED (MmWaveVehicularPlatoonHelper);

MmWaveVehicularPlatoonHelper::MmWaveVehicularPlatoonHelper ()
  : m_propagationLossModel (nullptr)
{
  NS_LOG_FUNCTION (this);
}

MmWaveVehicularPlatoonHelper::~MmWaveVehicularPlatoonHelper ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
MmWaveVehicularPlatoonHelper::GetTypeId ()
{
  static TypeId tid =
    TypeId ("ns3::MmWaveVehicularPlatoonHelper")
    .SetParent<Object> ()
    .AddConstructor<MmWaveVehicularPlatoonHelper> ()
    .AddAttribute ("Distance",
                   "Distance between vehicles. As non-consecutive vehicles are in NLOSv"
                   "condition, and vehicles are made of a single antenna, this is the"
                   "distance between points. In meters.",
                   DoubleValue (36.0),
                   MakeDoubleAccessor (&MmWaveVehicularPlatoonHelper::m_distance),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinX",
                   "X coordinate of the first vehicle in the platoon",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&MmWaveVehicularPlatoonHelper::m_minX),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinY",
                   "Y coordinate of the first vehicle in the platoon",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&MmWaveVehicularPlatoonHelper::m_minY),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Speed",
                   "Speed of platoon vehicles along the Y axis", //todo direction as an attribute
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&MmWaveVehicularPlatoonHelper::m_speed),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

void
MmWaveVehicularPlatoonHelper::CreatePlatoon (NodeContainer nodes)
{
  NS_LOG_FUNCTION (this);

  uint8_t nVehicles = nodes.GetN ();
  NS_ASSERT_MSG (nVehicles >= 2, "At least 2 vehicles in the platoon");
  NS_ASSERT_MSG (nVehicles <= 5, "At most 5 vehicles in the platoon");

  // assign position and mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (m_minX),
                                 "MinY", DoubleValue (m_minY),
                                 "DeltaX", DoubleValue (0.0),
                                 "DeltaY", DoubleValue (m_distance),
                                 "GridWidth", UintegerValue (nVehicles),
                                 "LayoutType", StringValue ("ColumnFirst"));
  mobility.Install (nodes);
  for (auto n = nodes.Begin (); n != nodes.End (); ++n)
    {
      (*n)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, m_speed, 0));
    }
}

void
MmWaveVehicularPlatoonHelper::SetChannelCondition (NetDeviceContainer c)
{
  NS_LOG_FUNCTION (this);

  Ptr<MmWaveVehicularPropagationLossModel> propagationLossModel =
    DynamicCast<MmWaveVehicularPropagationLossModel, PropagationLossModel> (
      m_propagationLossModel);

  NetDeviceContainer::Iterator dev = c.Begin ();
  NS_ASSERT_MSG (dev != c.End (), "Empty platoon");
  Ptr<MmWaveVehicularNetDevice> prev = DynamicCast<MmWaveVehicularNetDevice, NetDevice> (*dev);
  ++dev;
  while (dev != c.End ())
    {
      Ptr<MmWaveVehicularNetDevice> cur = DynamicCast<MmWaveVehicularNetDevice, NetDevice> (*dev);
      propagationLossModel->SetChannelCondition (GetMobilityForDevice (prev),
                                                 GetMobilityForDevice (cur),
                                                 "l");
      prev = cur;
      ++dev;
    }
}

void
MmWaveVehicularPlatoonHelper::SetRoutingTables (Ipv4InterfaceContainer ipv4Interfaces)
{
  NS_LOG_FUNCTION (this);

  Ipv4StaticRoutingHelper routingHelper;
  for (uint32_t i = 0; i != ipv4Interfaces.GetN (); ++i)
    {
      auto interfacePair = ipv4Interfaces.Get (i);
      Ptr<Ipv4StaticRouting> staticRouting = routingHelper.GetStaticRouting (interfacePair.first);
      uint32_t interface = interfacePair.second;
      for (uint32_t j = 0; j != ipv4Interfaces.GetN (); ++j)
        {
          if (i == j)
            {
              continue;
            }
          Ipv4Address dest = ipv4Interfaces.GetAddress (j);
          Ipv4Address nextHop = ipv4Interfaces.GetAddress (i + ((i < j) ? 1 : -1));
          staticRouting->AddHostRouteTo (dest, nextHop, interface);
          NS_LOG_DEBUG (ipv4Interfaces.GetAddress (i) << " -> " << dest << " next_hop " << nextHop);
        }

      // platoon leader as default gateway
      if (i > 0)
        {
          staticRouting->SetDefaultRoute (ipv4Interfaces.GetAddress (i - 1), interface);
          NS_LOG_DEBUG (ipv4Interfaces.GetAddress (i)
                        << " -> default next_hop " << ipv4Interfaces.GetAddress (i - 1));
        }
    }
}

void MmWaveVehicularPlatoonHelper::SetPropagationLossModel (
  Ptr<PropagationLossModel> propagationLossModel)
{
  NS_LOG_FUNCTION (this << propagationLossModel);
  m_propagationLossModel = propagationLossModel;
}

Ptr<MobilityModel>
MmWaveVehicularPlatoonHelper::GetMobilityForDevice (Ptr<MmWaveVehicularNetDevice> dev)
{
  NS_LOG_FUNCTION (this << dev);
  return dev->GetPhy ()->GetSpectrumPhy ()->GetMobility ();
}

} // namespace millicar
} // namespace ns3
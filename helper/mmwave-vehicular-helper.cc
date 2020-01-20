/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2019 University of Padova, Dep. of Information Engineering, SIGNET lab.
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
*/

#include "mmwave-vehicular-helper.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/antenna-array-model.h"
#include "ns3/mmwave-vehicular-spectrum-propagation-loss-model.h"
#include "ns3/pointer.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveVehicularHelper"); // TODO check if this has to be defined here

namespace mmwave_vehicular {

NS_OBJECT_ENSURE_REGISTERED (MmWaveVehicularHelper); // TODO check if this has to be defined here

MmWaveVehicularHelper::MmWaveVehicularHelper ()
{
  NS_LOG_FUNCTION (this);
}

MmWaveVehicularHelper::~MmWaveVehicularHelper ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
MmWaveVehicularHelper::GetTypeId ()
{
  static TypeId tid =
  TypeId ("ns3::MmWaveVehicularHelper")
  .SetParent<Object> ()
  .AddConstructor<MmWaveVehicularHelper> ()
  .AddAttribute ("PropagationLossModel",
                 "The type of path-loss model to be used. "
                 "The allowed values for this attributes are the type names "
                 "of any class inheriting from ns3::PropagationLossModel.",
                 StringValue (""),
                 MakeStringAccessor (&MmWaveVehicularHelper::SetPropagationLossModelType),
                 MakeStringChecker ())
  .AddAttribute ("SpectrumPropagationLossModel",
                 "The type of fast fading model to be used. "
                 "The allowed values for this attributes are the type names "
                 "of any class inheriting from ns3::SpectrumPropagationLossModel.",
                 StringValue (""),
                 MakeStringAccessor (&MmWaveVehicularHelper::SetSpectrumPropagationLossModelType),
                 MakeStringChecker ())
  .AddAttribute ("PropagationDelayModel",
                 "The type of propagation delay model to be used. "
                 "The allowed values for this attributes are the type names "
                 "of any class inheriting from ns3::PropagationDelayModel.",
                 StringValue (""),
                 MakeStringAccessor (&MmWaveVehicularHelper::SetPropagationDelayModelType),
                 MakeStringChecker ())
  .AddAttribute ("Numerology",
                 "Numerology to use for the definition of the frame structure."
                 "2 : subcarrier spacing will be set to 60 KHz"
                 "3 : subcarrier spacing will be set to 120 KHz",
                 UintegerValue (2),
                 MakeUintegerAccessor (&MmWaveVehicularHelper::SetNumerology),
                 MakeUintegerChecker<uint8_t> ())
  .AddAttribute ("Bandwidth",
                 "Bandwidth in Hz",
                 DoubleValue (1e8),
                 MakeDoubleAccessor (&MmWaveVehicularHelper::m_bandwidth),
                 MakeDoubleChecker<double> ())
  ;

  return tid;
}

void
MmWaveVehicularHelper::DoInitialize ()
{
  NS_LOG_FUNCTION (this);

  // intialize the RNTI counter
  m_rntiCounter = 0;

  // create the channel
  m_channel = CreateObject<SingleModelSpectrumChannel> ();
  if (!m_propagationLossModelType.empty ())
  {
    ObjectFactory factory (m_propagationLossModelType);
    m_channel->AddPropagationLossModel (factory.Create<PropagationLossModel> ());
  }
  if (!m_spectrumPropagationLossModelType.empty ())
  {
    ObjectFactory factory (m_spectrumPropagationLossModelType);
    m_channel->AddSpectrumPropagationLossModel (factory.Create<SpectrumPropagationLossModel> ());
  }
  if (!m_propagationDelayModelType.empty ())
  {
    ObjectFactory factory (m_propagationDelayModelType);
    m_channel->SetPropagationDelayModel (factory.Create<PropagationDelayModel> ());
  }

  // 3GPP vehicular channel needs proper configuration
  if (m_spectrumPropagationLossModelType == "ns3::MmWaveVehicularSpectrumPropagationLossModel")
  {
    Ptr<MmWaveVehicularSpectrumPropagationLossModel> threeGppSplm = DynamicCast<MmWaveVehicularSpectrumPropagationLossModel> (m_channel->GetSpectrumPropagationLossModel ());
    PointerValue plm;
    m_channel->GetAttribute ("PropagationLossModel", plm);
    threeGppSplm->SetPathlossModel (plm.Get<PropagationLossModel> ()); // associate pathloss and fast fading models
  }
}

void
MmWaveVehicularHelper::SetConfigurationParameters (Ptr<mmwave::MmWavePhyMacCommon> conf)
{
  NS_LOG_FUNCTION (this);
  m_phyMacConfig = conf;
}

Ptr<mmwave::MmWavePhyMacCommon>
MmWaveVehicularHelper::GetConfigurationParameters () const
{
  NS_LOG_FUNCTION (this);
  return m_phyMacConfig;
}


void
MmWaveVehicularHelper::SetNumerology (uint8_t index)
{

  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG ( (index == 2) || (index == 3), "Numerology index is not valid.");

  m_numerologyIndex = index;

  m_phyMacConfig = CreateObject<mmwave::MmWavePhyMacCommon> ();

  double subcarrierSpacing = 15 * std::pow (2, m_numerologyIndex) * 1000; // subcarrier spacing based on the numerology. Only 60KHz and 120KHz is supported in NR V2X.

  m_phyMacConfig->SetSymbPerSlot(14); // TR 38.802 Section 5.3: each slot must have 14 symbols < Symbol duration is dependant on the numerology
  m_phyMacConfig->SetSlotPerSubframe(std::pow (2, m_numerologyIndex)); // flexible number of slots per subframe - depends on numerology
  m_phyMacConfig->SetSubframePeriod (1000); // TR 38.802 Section 5.3: the subframe duration is 1ms, i.e., 1000us, and the frame length is 10ms.
  m_phyMacConfig->SetSlotPeriod ( (m_phyMacConfig->GetSubframePeriod() / m_phyMacConfig->GetSlotsPerSubframe()) / 1e6); // 1 ms is dedicated to each subframe, the slot period is evaluated accordingly

  m_phyMacConfig->SetSymbolPeriod ( (1 / subcarrierSpacing) * 1e6 ); // symbol period is required in microseconds

  double subCarriersPerRB = 12;

  m_phyMacConfig->SetNumChunkPerRB(1); // each resource block contains 1 chunk
  m_phyMacConfig->SetNumRb ( uint32_t( m_bandwidth / (subcarrierSpacing * subCarriersPerRB) ) );

  m_phyMacConfig->SetChunkWidth (subCarriersPerRB*subcarrierSpacing);

}

NetDeviceContainer
MmWaveVehicularHelper::InstallMmWaveVehicularNetDevices (NodeContainer nodes)
{
  NS_LOG_FUNCTION (this);

  Initialize (); // run DoInitialize if necessary

  NetDeviceContainer devices;
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      Ptr<Node> node = *i;

      // create the device
      Ptr<MmWaveVehicularNetDevice> device = InstallSingleMmWaveVehicularNetDevice (node, ++m_rntiCounter);

      // assign an address
      device->SetAddress (Mac64Address::Allocate ());

      devices.Add (device);
    }

  return devices;
}

Ptr<MmWaveVehicularNetDevice>
MmWaveVehicularHelper::InstallSingleMmWaveVehicularNetDevice (Ptr<Node> node, uint16_t rnti)
{
  NS_LOG_FUNCTION (this);

  // create the antenna
  Ptr<mmwave::AntennaArrayModel> aam = CreateObject<mmwave::AntennaArrayModel> ();

  // create and configure the tx spectrum phy
  Ptr<MmWaveSidelinkSpectrumPhy> ssp = CreateObject<MmWaveSidelinkSpectrumPhy> ();
  NS_ASSERT_MSG (node->GetObject<MobilityModel> (), "Missing mobility model");
  ssp->SetMobility (node->GetObject<MobilityModel> ());
  ssp->SetAntenna (aam);
  NS_ASSERT_MSG (m_channel, "First create the channel");
  ssp->SetChannel (m_channel);

  // add the spectrum phy to the spectrum channel
  m_channel->AddRx (ssp);

  // create and configure the chunk processor
  Ptr<mmwave::mmWaveChunkProcessor> pData = Create<mmwave::mmWaveChunkProcessor> ();
  pData->AddCallback (MakeCallback (&MmWaveSidelinkSpectrumPhy::UpdateSinrPerceived, ssp));
  ssp->AddDataSinrChunkProcessor (pData);

  // create the phy
  NS_ASSERT_MSG (m_phyMacConfig, "First set the configuration parameters");
  Ptr<MmWaveSidelinkPhy> phy = CreateObject<MmWaveSidelinkPhy> (ssp, m_phyMacConfig);

  // connect the rx callback of the spectrum object to the sink
  ssp->SetPhyRxDataEndOkCallback (MakeCallback (&MmWaveSidelinkPhy::Receive, phy));

  // connect the callback to report the SINR
  ssp->SetSidelinkSinrReportCallback (MakeCallback (&MmWaveSidelinkPhy::GenerateSinrReport, phy));
  
  // create the mac
  Ptr<MmWaveSidelinkMac> mac = CreateObject<MmWaveSidelinkMac> (m_phyMacConfig);
  mac->SetRnti (rnti);

  // connect phy and mac
  phy->SetPhySapUser (mac->GetPhySapUser ());
  mac->SetPhySapProvider (phy->GetPhySapProvider ());

  // create and configure the device
  Ptr<MmWaveVehicularNetDevice> device = CreateObject<MmWaveVehicularNetDevice> (phy, mac);
  node->AddDevice (device);
  device->SetNode (node);
  ssp->SetDevice (device);

  // connect the rx callback of the mac object to the rx method of the NetDevice
  mac->SetForwardUpCallback(MakeCallback(&MmWaveVehicularNetDevice::Receive, device));

  // initialize the channel (if needed)
  Ptr<MmWaveVehicularSpectrumPropagationLossModel> splm = DynamicCast<MmWaveVehicularSpectrumPropagationLossModel> (m_channel->GetSpectrumPropagationLossModel ());
  if (splm)
    splm->AddDevice (device, aam);

  return device;
}

void
MmWaveVehicularHelper::PairDevices (NetDeviceContainer devices)
{
  NS_LOG_FUNCTION (this);

  // define the scheduling pattern
  // NOTE we assume a fixed scheduling pattern which periodically repeats at
  // each subframe. Each slot in the subframe is assigned to a different user.
  // If the number of devices is greater than the number of slots per subframe
  // an assert is raised
  // TODO update this part to enable a more flexible configuration of the
  // scheduling pattern
  NS_ASSERT_MSG (devices.GetN () <= m_phyMacConfig->GetSlotsPerSubframe (), "Too many devices");
  std::vector<uint16_t> pattern (m_phyMacConfig->GetSlotsPerSubframe ());
  for (uint16_t i = 0; i < devices.GetN (); i++)
  {
    Ptr<MmWaveVehicularNetDevice> di = DynamicCast<MmWaveVehicularNetDevice> (devices.Get (i));
    pattern.at (i) = di->GetMac ()->GetRnti ();
    NS_LOG_DEBUG ("slot " << i << " assigned to rnti " << pattern.at (i));
  }

  for (NetDeviceContainer::Iterator i = devices.Begin (); i != devices.End (); ++i)
    {

      Ptr<MmWaveVehicularNetDevice> di = DynamicCast<MmWaveVehicularNetDevice> (*i);

      di->GetMac ()->SetSfAllocationInfo (pattern);

      for (NetDeviceContainer::Iterator j = devices.Begin (); j != devices.End (); ++j)
      {
        Ptr<MmWaveVehicularNetDevice> dj = DynamicCast<MmWaveVehicularNetDevice> (*j);
        Ptr<Node> jNode = dj->GetNode ();
        Ptr<Ipv4> jNodeIpv4 = jNode->GetObject<Ipv4> ();
        NS_ASSERT_MSG (jNodeIpv4 != 0, "Nodes need to have IPv4 installed before pairing can be activated");

        if (*i != *j)
        {
          // initialize the <IP address, RNTI> map of the devices
          int32_t interface =  jNodeIpv4->GetInterfaceForDevice (dj);
          di->RegisterDevice (jNodeIpv4->GetAddress (interface, 0).GetLocal (), dj->GetMac ()->GetRnti ());

          // register the associated devices in the PHY
          di->GetPhy ()->AddDevice (dj->GetMac ()->GetRnti (), dj);
        }
      }

    }
}

void
MmWaveVehicularHelper::SetPropagationLossModelType (std::string plm)
{
  NS_LOG_FUNCTION (this);
  m_propagationLossModelType = plm;
}

void
MmWaveVehicularHelper::SetSpectrumPropagationLossModelType (std::string splm)
{
  NS_LOG_FUNCTION (this);
  m_spectrumPropagationLossModelType = splm;
}

void
MmWaveVehicularHelper::SetPropagationDelayModelType (std::string pdm)
{
  NS_LOG_FUNCTION (this);
  m_propagationDelayModelType = pdm;
}

} // namespace mmwave_vehicular
} // namespace ns3

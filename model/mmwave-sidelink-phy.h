/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
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
*   along with this program; if not, write to the Free Software:100cento

*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/


#ifndef SRC_MMWAVE_MODEL_MMWAVE_SIDELINK_PHY_H_
#define SRC_MMWAVE_MODEL_MMWAVE_SIDELINK_PHY_H_

#include <ns3/mmwave-phy-mac-common.h>
#include <ns3/ptr.h>
#include <map>
#include "mmwave-sidelink-spectrum-phy.h"

namespace ns3 {

namespace mmwave {

class MmWaveSidelinkPhy : public Object
{

public:
  MmWaveSidelinkPhy ();

  /**
   * \brief MmWaveSidelinkPhy real constructor
   * \param channelPhy spectrum phy
   * \param confParams instance of MmWavePhyMacCommon containing the
   *        configuration parameters
   *
   * Usually called by the helper. It starts the event loop for the device.
   */
  MmWaveSidelinkPhy (Ptr<MmWaveSidelinkSpectrumPhy> spectrumPhy, Ptr<MmWavePhyMacCommon> confParams);

  virtual ~MmWaveSidelinkPhy ();

  // inherited from Object
  static TypeId GetTypeId (void);
  virtual void DoInitialize (void);
  virtual void DoDispose (void);

  void SetTxPower (double power);
  double GetTxPower () const;

  void SetNoiseFigure (double pf);
  double GetNoiseFigure () const;

  void SetConfigurationParameters (Ptr<MmWavePhyMacCommon> ptrConfig);
  Ptr<MmWavePhyMacCommon> GetConfigurationParameters (void) const;

  virtual Ptr<MmWaveSidelinkSpectrumPhy> GetSpectrumPhy () const;

  // bool SendPacket (Ptr<Packet> packet);
  //
  Ptr<SpectrumValue> CreateTxPowerSpectralDensity ();
  //
  // void DoSetSubChannels ();
  //
  // void SetSubChannelsForReception (std::vector <int> mask);
  // std::vector <int> GetSubChannelsForReception (void);
  //
  // void SetSubChannelsForTransmission (std::vector <int> mask);
  // std::vector <int> GetSubChannelsForTransmission (void);
  //
  // void DoSendControlMessage (Ptr<MmWaveControlMessage> msg);
  //
  // Ptr<MmWaveSpectrumPhy> GetDlSpectrumPhy () const;
  // Ptr<MmWaveSpectrumPhy> GetUlSpectrumPhy () const;

  void StartSlot (uint16_t slotNum);
  void StartVarTti ();
  void EndVarTti ();

  void AddPacketBurst (Ptr<PacketBurst> pb);

private:

  /**
   * \brief Transmit SL data
   * \param slotNum the slot index
   */
  Time SlData (uint16_t slotNum);

  /**
   * \brief Transform a MAC-made vector of RBG to a PHY-ready vector of SINR indices
   * \param rbgBitmask Bitmask which indicates with 1 the RBG in which there is a transmission,
   * with 0 a RBG in which there is not a transmission
   * \return a vector of indices.
   *
   * Example (4 RB per RBG, 4 total RBG assignable):
   * rbgBitmask = <0,1,1,0>
   * output = <4,5,6,7,8,9,10,11>
   *
   * (the rbgBitmask expressed as rbBitmask would be:
   * <0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0> , and therefore the places in which there
   * is a 1 are from the 4th to the 11th, and that is reflected in the output)
   */
  std::vector<int> FromRBGBitmaskToRBAssignment (const std::vector<uint8_t> rbgBitmask) const;

  std::vector<int> SetSubChannelsForTransmission ();

  bool SidelinkSlotAllocInfoExists (const SfnSf &retVal) const;
  SidelinkSlotAllocInfo RetrieveSidelinkSlotAllocInfo ();
  SidelinkSlotAllocInfo RetrieveSidelinkSlotAllocInfo (const SfnSf &sfnsf);

  void SendDataChannels (Ptr<PacketBurst> pb, Time duration, uint8_t slotInd, uint8_t mcs, uint32_t size, std::vector<int> rbBitmap);

private:
  double m_txPower; //!< the transmission power in dBm
  double m_noiseFigure; //!< the noise figure in dB
  Time m_lastSlotStart; //!< Time of the last slot start
  std::list<SidelinkSlotAllocInfo> m_slotAllocInfo; //!< slot allocation info list
  SidelinkSlotAllocInfo m_currSlotAllocInfo;

  bool m_receptionEnabled {false}; //!< Flag to indicate if we are currently receiveing data

  uint8_t m_varTtiNum {0};
  uint32_t m_currTbs {0};          //!< Current TBS of the receiveing DL data (used to compute the feedback)
  Ptr<MmWaveSidelinkSpectrumPhy> m_sidelinkSpectrumPhy; //!< the SpectrumPhy instance associated with this PHY
  Ptr<MmWavePhyMacCommon> m_phyMacConfig; //!< the configuration parameters
  std::list<Ptr<PacketBurst>> m_packetBurstBuffer; //!< buffer of packet bursts to send
  uint8_t m_mcs; //!< modulation and coding scheme value
};

}

}

#endif /* SRC_MMWAVE_MODEL_MMWAVE_SIDELINK_PHY_H_ */

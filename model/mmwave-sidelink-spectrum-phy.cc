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
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/

#include <ns3/object-factory.h>
#include <ns3/log.h>
#include <ns3/ptr.h>
#include <ns3/boolean.h>
#include <cmath>
#include <ns3/simulator.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/antenna-model.h>
#include "mmwave-sidelink-spectrum-phy.h"
#include "mmwave-phy-mac-common.h"
#include <ns3/mmwave-ue-phy.h>
#include "mmwave-radio-bearer-tag.h"
#include <stdio.h>
#include <ns3/double.h>
#include <ns3/mmwave-mi-error-model.h>
#include "mmwave-mac-pdu-tag.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MmWaveSidelinkSpectrumPhy");

namespace mmwave {

NS_OBJECT_ENSURE_REGISTERED (MmWaveSidelinkSpectrumPhy);

static const double EffectiveCodingRate[29] = {
  0.08,
  0.1,
  0.11,
  0.15,
  0.19,
  0.24,
  0.3,
  0.37,
  0.44,
  0.51,
  0.3,
  0.33,
  0.37,
  0.42,
  0.48,
  0.54,
  0.6,
  0.43,
  0.45,
  0.5,
  0.55,
  0.6,
  0.65,
  0.7,
  0.75,
  0.8,
  0.85,
  0.89,
  0.92
};

MmWaveSidelinkSpectrumPhy::MmWaveSidelinkSpectrumPhy ()
  : m_state (IDLE),
    m_componentCarrierId (0)
{
  m_interferenceData = CreateObject<mmWaveInterference> ();
  m_random = CreateObject<UniformRandomVariable> ();
  m_random->SetAttribute ("Min", DoubleValue (0.0));
  m_random->SetAttribute ("Max", DoubleValue (1.0));
}

MmWaveSidelinkSpectrumPhy::~MmWaveSidelinkSpectrumPhy ()
{

}

TypeId
MmWaveSidelinkSpectrumPhy::GetTypeId (void)
{
  static TypeId
    tid =
    TypeId ("ns3::MmWaveSidelinkSpectrumPhy")
    .SetParent<NetDevice> ()
    .AddAttribute ("DataErrorModelEnabled",
                   "Activate/Deactivate the error model of data [by default is active].",
                   BooleanValue (true),
                   MakeBooleanAccessor (&MmWaveSidelinkSpectrumPhy::m_dataErrorModelEnabled),
                   MakeBooleanChecker ())
  ;

  return tid;
}
void
MmWaveSidelinkSpectrumPhy::DoDispose ()
{

}

void
MmWaveSidelinkSpectrumPhy::Reset ()
{
  NS_LOG_FUNCTION (this);
  m_state = IDLE;
  m_endTxEvent.Cancel ();
  m_endRxDataEvent.Cancel ();
  m_endRxCtrlEvent.Cancel ();
  m_rxControlMessageList.clear ();
  m_expectedTbs.clear ();
  m_rxPacketBurstList.clear ();
  //m_txPacketBurst = 0;
  //m_rxSpectrumModel = 0;
}

void
MmWaveSidelinkSpectrumPhy::ResetSpectrumModel ()
{
  m_rxSpectrumModel = 0;
}

void
MmWaveSidelinkSpectrumPhy::SetDevice (Ptr<NetDevice> d)
{
  m_device = d;
}

Ptr<NetDevice>
MmWaveSidelinkSpectrumPhy::GetDevice () const
{
  return m_device;
}

void
MmWaveSidelinkSpectrumPhy::SetMobility (Ptr<MobilityModel> m)
{
  m_mobility = m;
}

Ptr<MobilityModel>
MmWaveSidelinkSpectrumPhy::GetMobility ()
{
  return m_mobility;
}

void
MmWaveSidelinkSpectrumPhy::SetChannel (Ptr<SpectrumChannel> c)
{
  m_channel = c;
}

Ptr<const SpectrumModel>
MmWaveSidelinkSpectrumPhy::GetRxSpectrumModel () const
{
  return m_rxSpectrumModel;
}

Ptr<AntennaModel>
MmWaveSidelinkSpectrumPhy::GetRxAntenna ()
{
  return m_antenna;
}

void
MmWaveSidelinkSpectrumPhy::SetAntenna (Ptr<AntennaModel> a)
{
  m_antenna = a;
}

void
MmWaveSidelinkSpectrumPhy::SetState (State newState)
{
  ChangeState (newState);
}

void
MmWaveSidelinkSpectrumPhy::ChangeState (State newState)
{
  NS_LOG_LOGIC (this << " state: " << m_state << " -> " << newState);
  m_state = newState;
}

void
MmWaveSidelinkSpectrumPhy::SetNoisePowerSpectralDensity (Ptr<const SpectrumValue> noisePsd)
{
  NS_LOG_FUNCTION (this << noisePsd);
  NS_ASSERT (noisePsd);
  m_rxSpectrumModel = noisePsd->GetSpectrumModel ();
  m_interferenceData->SetNoisePowerSpectralDensity (noisePsd);
}

void
MmWaveSidelinkSpectrumPhy::SetTxPowerSpectralDensity (Ptr<SpectrumValue> TxPsd)
{
  m_txPsd = TxPsd;
}

void
MmWaveSidelinkSpectrumPhy::SetPhyRxDataEndOkCallback (MmWavePhyRxDataEndOkCallback c)
{
  m_phyRxDataEndOkCallback = c;
}

void
MmWaveSidelinkSpectrumPhy::SetPhyRxCtrlEndOkCallback (MmWavePhyRxCtrlEndOkCallback c)
{
  m_phyRxCtrlEndOkCallback = c;
}

void
MmWaveSidelinkSpectrumPhy::AddExpectedTb (uint16_t rnti, uint8_t ndi, uint32_t tbSize, uint8_t mcs,
                                  std::vector<int> chunkMap, uint8_t rv,
                                  uint8_t symStart, uint8_t numSym)
{
  ExpectedTbMap_t::iterator it;
  it = m_expectedTbs.find (rnti);

  if (it != m_expectedTbs.end ())
    {
      m_expectedTbs.erase (it);
    }

  ExpectedTbInfo_t tbInfo = {ndi, tbSize, mcs, chunkMap, rv, 0.0, false, false, 0, symStart, numSym};
  m_expectedTbs.insert (std::pair<uint16_t, ExpectedTbInfo_t> (rnti,tbInfo));
}

void
MmWaveSidelinkSpectrumPhy::StartRx (Ptr<SpectrumSignalParameters> params)
{

  NS_LOG_FUNCTION (this);

  Ptr<MmWaveSidelinkSpectrumSignalParameters> mmwaveSidelinkParams =
    DynamicCast<MmWaveSidelinkSpectrumSignalParameters> (params);

  if (mmwaveSidelinkParams != 0)
    {
      bool isAllocated = true;

      // Ptr<mmwave::MmWaveUeNetDevice> ueRx = 0;
      // ueRx = DynamicCast<mmwave::MmWaveUeNetDevice> (GetDevice ());
      //
      // if ((ueRx != 0) && (ueRx->GetPhy (m_componentCarrierId)->IsReceptionEnabled () == false))
      //   {               // if the first cast is 0 (the device is MC) then this if will not be executed
      //     isAllocated = false;
      //   }

      if (isAllocated)
        {
          m_interferenceData->AddSignal (mmwaveSidelinkParams->psd, mmwaveSidelinkParams->duration);
          StartRxData (mmwaveSidelinkParams);
        }
    }
}

void
MmWaveSidelinkSpectrumPhy::StartRxData (Ptr<MmWaveSidelinkSpectrumSignalParameters> params)
{
  m_interferenceData->StartRx (params->psd);

  NS_LOG_FUNCTION (this);

  switch (m_state)
    {
    case TX:
      NS_FATAL_ERROR ("Cannot receive while transmitting");
      break;
    case RX_CTRL:
      NS_FATAL_ERROR ("Cannot receive control in data period");
      break;
    case RX_DATA:
    case IDLE:
      {
        if (m_rxPacketBurstList.empty ())
          {
            NS_ASSERT (m_state == IDLE);
            // first transmission, i.e., we're IDLE and we start RX
            m_firstRxStart = Simulator::Now ();
            m_firstRxDuration = params->duration;
            NS_LOG_LOGIC (this << " scheduling EndRx with delay " << params->duration.GetSeconds () << "s");

            m_endRxDataEvent = Simulator::Schedule (params->duration, &MmWaveSidelinkSpectrumPhy::EndRxData, this);
          }
        else
          {
            NS_ASSERT (m_state == RX_DATA);
            // sanity check: if there are multiple RX events, they
            // should occur at the same time and have the same
            // duration, otherwise the interference calculation
            // won't be correct
            NS_ASSERT ((m_firstRxStart == Simulator::Now ()) && (m_firstRxDuration == params->duration));
          }

        ChangeState (RX_DATA);
        if (params->packetBurst && !params->packetBurst->GetPackets ().empty ())
          {
            m_rxPacketBurstList.push_back (params->packetBurst);
          }
        m_rxControlMessageList.insert (m_rxControlMessageList.end (), params->ctrlMsgList.begin (), params->ctrlMsgList.end ());

        NS_LOG_LOGIC (this << " numSimultaneousRxEvents = " << m_rxPacketBurstList.size ());
      }
      break;
    default:
      NS_FATAL_ERROR ("Programming Error: Unknown State");
    }
}

void
MmWaveSidelinkSpectrumPhy::StartRxCtrl (Ptr<SpectrumSignalParameters> params)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case TX:
      NS_FATAL_ERROR ("Cannot RX while TX");
      break;
    case RX_DATA:
      NS_FATAL_ERROR ("Cannot RX data while receiving control");
      break;
    case RX_CTRL:
    case IDLE:
      {
        // the behavior is similar when we're IDLE or RX because we can receive more signals
        // simultaneously (e.g., at the eNB).
        Ptr<MmWaveSidelinkSpectrumSignalParameters> sidelinkParams = DynamicCast<MmWaveSidelinkSpectrumSignalParameters> (params);
        if (m_state == RX_CTRL)
          {
            NS_FATAL_ERROR ("TBD");
          }
        if (m_state == IDLE)
          {
            // first transmission, i.e., we're IDLE and we start RX
            NS_ASSERT (m_rxControlMessageList.empty ());
            m_firstRxStart = Simulator::Now ();
            m_firstRxDuration = params->duration;
            NS_LOG_LOGIC (this << " scheduling EndRx with delay " << params->duration);
            // store the DCIs
            m_rxControlMessageList = sidelinkParams->ctrlMsgList;
            m_endRxDlCtrlEvent = Simulator::Schedule (params->duration, &MmWaveSidelinkSpectrumPhy::EndRxCtrl, this);
            ChangeState (RX_CTRL);
          }
        else
          {
            m_rxControlMessageList.insert (m_rxControlMessageList.end (), sidelinkParams->ctrlMsgList.begin (), sidelinkParams->ctrlMsgList.end ());
          }
        break;
      }
    default:
      NS_FATAL_ERROR ("Programming Error: Unknown State");
    }
}

void
MmWaveSidelinkSpectrumPhy::EndRxData ()
{
  m_interferenceData->EndRx ();

  double sinrAvg = Sum (m_sinrPerceived) / (m_sinrPerceived.GetSpectrumModel ()->GetNumBands ());
  double sinrMin = 99999999999;
  for (Values::const_iterator it = m_sinrPerceived.ConstValuesBegin (); it != m_sinrPerceived.ConstValuesEnd (); it++)
    {
      if (*it < sinrMin)
        {
          sinrMin = *it;
        }
    }

  Ptr<MmWaveEnbNetDevice> enbRx = DynamicCast<MmWaveEnbNetDevice> (GetDevice ());
  Ptr<mmwave::MmWaveUeNetDevice> ueRx = DynamicCast<mmwave::MmWaveUeNetDevice> (GetDevice ());
  Ptr<McUeNetDevice> rxMcUe = DynamicCast<McUeNetDevice> (GetDevice ());

  NS_ASSERT (m_state = RX_DATA);
  ExpectedTbMap_t::iterator itTb = m_expectedTbs.begin ();
  while (itTb != m_expectedTbs.end ())
    {
      if ((m_dataErrorModelEnabled)&&(m_rxPacketBurstList.size () > 0))
        {

          MmWaveTbStats_t tbStats = MmWaveMiErrorModel::GetTbDecodificationStats (m_sinrPerceived,
                                                                                  itTb->second.rbBitmap, itTb->second.size, itTb->second.mcs, harqInfoList);
          itTb->second.tbler = tbStats.tbler;
          itTb->second.mi = tbStats.miTotal;
          itTb->second.corrupt = m_random->GetValue () > tbStats.tbler ? false : true;
          if (itTb->second.corrupt)
            {
              NS_LOG_INFO (this << " RNTI " << itTb->first << " size " << itTb->second.size << " mcs " << (uint32_t)itTb->second.mcs << " bitmap " << itTb->second.rbBitmap.size () << " rv " << rv << " TBLER " << tbStats.tbler << " corrupted " << itTb->second.corrupt);
            }
        }
      itTb++;
    }

  for (std::list<Ptr<PacketBurst> >::const_iterator i = m_rxPacketBurstList.begin ();
       i != m_rxPacketBurstList.end (); ++i)
    {
      for (std::list<Ptr<Packet> >::const_iterator j = (*i)->Begin (); j != (*i)->End (); ++j)
        {
          if ((*j)->GetSize () == 0)
            {
              continue;
            }

          LteRadioBearerTag bearerTag;
          if ((*j)->PeekPacketTag (bearerTag) == false)
            {
              NS_FATAL_ERROR ("No radio bearer tag found");
            }
          uint16_t rnti = bearerTag.GetRnti ();
          itTb = m_expectedTbs.find (rnti);
          if (itTb != m_expectedTbs.end ())
            {
              if (!itTb->second.corrupt)
                {
                  m_phyRxDataEndOkCallback (*j);
                }
              else
                {
                  NS_LOG_INFO ("TB failed");
                }

              MmWaveMacPduTag pduTag;
              if ((*j)->PeekPacketTag (pduTag) == false)
                {
                  NS_FATAL_ERROR ("No radio bearer tag found");
                }

              RxPacketTraceParams traceParams;
              traceParams.m_tbSize = itTb->second.size;
              traceParams.m_frameNum = pduTag.GetSfn ().m_frameNum;
              traceParams.m_sfNum = pduTag.GetSfn ().m_sfNum;
              traceParams.m_slotNum = pduTag.GetSfn ().m_slotNum;
              traceParams.m_rnti = rnti;
              traceParams.m_mcs = itTb->second.mcs;
              traceParams.m_rv = itTb->second.rv;
              traceParams.m_sinr = sinrAvg;
              traceParams.m_sinrMin = itTb->second.mi;                  //sinrMin;
              traceParams.m_tbler = itTb->second.tbler;
              traceParams.m_corrupt = itTb->second.corrupt;
              traceParams.m_symStart = itTb->second.symStart;
              traceParams.m_numSym = itTb->second.numSym;
              traceParams.m_ccId = m_componentCarrierId;

              // if (enbRx)
              //   {
              //     //traceParams.m_cellId = enbRx->GetCellId(); //now m_cellId is set correctly
              //     m_rxPacketTraceEnb (traceParams);
              //   }
              // else if (ueRx)
              //   {
              //
              //     //traceParams.m_cellId = ueRx->GetTargetEnb()->GetCellId(); //now m_cellId is set correctly
              //     m_rxPacketTraceUe (traceParams);
              //   }
              // else if (rxMcUe)
              //   {
              //     Ptr<McUeNetDevice> mcUe = DynamicCast<McUeNetDevice> (ueRx);
              //     if (mcUe != 0)
              //       {
              //         Ptr<MmWaveEnbNetDevice> mmWaveEnb = mcUe->GetMmWaveTargetEnb ();
              //         if (mmWaveEnb != 0)
              //           {
              //             //traceParams.m_cellId = mmWaveEnb->GetCellId(); //now m_cellId is set correctly
              //           }
              //       }
              //     m_rxPacketTraceUe (traceParams);                       // TODO consider a different trace for MC UE
              //   }
            }
        }
    }

  // forward control messages of this frame to MmWavePhy

  if (!m_rxControlMessageList.empty () && !m_phyRxCtrlEndOkCallback.IsNull ())
    {
      m_phyRxCtrlEndOkCallback (m_rxControlMessageList);
    }

  m_state = IDLE;
  m_rxPacketBurstList.clear ();
  m_expectedTbs.clear ();
  m_rxControlMessageList.clear ();
}

void
MmWaveSidelinkSpectrumPhy::EndRxCtrl ()
{
  NS_ASSERT (m_state = RX_CTRL);

  // control error model not supported
  // forward control messages of this frame to LtePhy
  if (!m_rxControlMessageList.empty ())
    {
      if (!m_phyRxCtrlEndOkCallback.IsNull ())
        {
          m_phyRxCtrlEndOkCallback (m_rxControlMessageList);
        }
    }

  m_state = IDLE;
  m_rxControlMessageList.clear ();
}

bool
MmWaveSidelinkSpectrumPhy::StartTxDataFrames (Ptr<PacketBurst> pb, std::list<Ptr<MmWaveControlMessage> > ctrlMsgList, Time duration, uint8_t slotInd)
{
  switch (m_state)
    {
    case RX_DATA:
    case RX_CTRL:
      NS_FATAL_ERROR ("Cannot transmit while receiving");
      break;
    case TX:
      NS_FATAL_ERROR ("Cannot transmit while a different transmission is still on");
      break;
    case IDLE:
      {
        NS_ASSERT (m_txPsd);

        m_state = TX;
        Ptr<MmWaveSidelinkSpectrumSignalParameters> txParams = Create<MmWaveSidelinkSpectrumSignalParameters> ();
        txParams->duration = duration;
        txParams->txPhy = this->GetObject<SpectrumPhy> ();
        txParams->psd = m_txPsd;
        txParams->packetBurst = pb;
        txParams->ctrlMsgList = ctrlMsgList;
        txParams->slotInd = slotInd;
        txParams->txAntenna = m_antenna;

        m_channel->StartTx (txParams);

        m_endTxEvent = Simulator::Schedule (duration, &MmWaveSidelinkSpectrumPhy::EndTx, this);
      }
      break;
    default:
      NS_LOG_FUNCTION (this << "Programming Error. Code should not reach this point");
    }
  return true;
}

bool
MmWaveSidelinkSpectrumPhy::StartTxDlControlFrames (std::list<Ptr<MmWaveControlMessage> > ctrlMsgList, Time duration)
{
  NS_LOG_LOGIC (this << " state: " << m_state);

  switch (m_state)
    {
    case RX_DATA:
    case RX_CTRL:
      NS_FATAL_ERROR ("Cannot transmit while receiving");
      break;
    case TX:
      NS_FATAL_ERROR ("Cannot transmit while a different transmission is still on");
      break;
    case IDLE:
      {
        NS_ASSERT (m_txPsd);

        m_state = TX;

        Ptr<MmWaveSidelinkSpectrumSignalParameters> txParams = Create<MmWaveSidelinkSpectrumSignalParameters> ();
        txParams->duration = duration;
        txParams->txPhy = GetObject<SpectrumPhy> ();
        txParams->psd = m_txPsd;
        txParams->pss = true;
        txParams->ctrlMsgList = ctrlMsgList;
        txParams->txAntenna = m_antenna;
        m_channel->StartTx (txParams);

        m_endTxEvent = Simulator::Schedule (duration, &MmWaveSidelinkSpectrumPhy::EndTx, this);
      }
    }
  return false;
}

void
MmWaveSidelinkSpectrumPhy::EndTx ()
{
  NS_ASSERT (m_state == TX);

  m_state = IDLE;
}

Ptr<SpectrumChannel>
MmWaveSidelinkSpectrumPhy::GetSpectrumChannel ()
{
  return m_channel;
}

void
MmWaveSidelinkSpectrumPhy::SetComponentCarrierId (uint8_t componentCarrierId)
{
  m_componentCarrierId = componentCarrierId;
}

void
MmWaveSidelinkSpectrumPhy::AddDataPowerChunkProcessor (Ptr<mmWaveChunkProcessor> p)
{
  m_interferenceData->AddPowerChunkProcessor (p);
}

void
MmWaveSidelinkSpectrumPhy::AddDataSinrChunkProcessor (Ptr<mmWaveChunkProcessor> p)
{
  m_interferenceData->AddSinrChunkProcessor (p);
}

void
MmWaveSidelinkSpectrumPhy::UpdateSinrPerceived (const SpectrumValue& sinr)
{
  NS_LOG_FUNCTION (this << sinr);
  m_sinrPerceived = sinr;
}

}

}
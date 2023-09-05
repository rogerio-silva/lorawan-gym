/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright(c) 2022:
 *        INF-UFG Informatics Institute - Federal University of Goiás
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
 * This script implements a complex scenario on LoRaWAN with multiple gateways,
 * end devices, and network servers. The future interest in this script is the
 * LoRa network slicing throughput and other metrics. We adopt  SF and TP
 * parameters setting and multichannel allocation to achieve a LoRaWAN Network
 * Slicing. Metrics of interest are Network Data Rate, Throughput, and Delay.
 * Specifically, we are interested in capturing metrics in several distances
 * between GW and ED, and after the run, we will try to change SF and TP parameters.
 *
 * Adapted from: Floris Van den Abeele <floris.vandenabeele@ugent.be>
 * Author: Rogério S Silva <rogeriosilva@inf.ufg.br>
 */

#include "ns3/gateway-lora-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/lorawan-module.h"
#include "ns3/propagation-module.h"
#include "ns3/test.h"
#include "ns3/log.h"
#include "ns3/callback.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-value.h"
#include "ns3/node.h"
#include "ns3/net-device.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/mac16-address.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/uinteger.h"
#include "ns3/nstime.h"
#include "ns3/abort.h"
#include "ns3/command-line.h"
#include "ns3/gnuplot.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;
using namespace lorawan;

static uint32_t g_received = 0;
static uint32_t g_sended = 0;
static u_int32_t g_errorNoDemodul = 0;
static u_int32_t g_errorInterf = 0;
static u_int32_t g_errorSens = 0;
static u_int32_t g_errorSF = 0;
static u_int32_t g_errorWrongFreq = 0;
static u_int32_t g_errorNoReceivers = 0;
static u_int32_t g_errorOccupied = 0;
static u_int32_t g_errorGWInterf = 0;

NS_LOG_COMPONENT_DEFINE ("SimplePeroNoMucho");

std::vector<int> packetsSent (6, 0);
std::vector<int> packetsReceived (6, 0);
std::vector<int> packetsRxError (6, 0);

void
OnTransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_sended++;
}

void
OnPacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_received++;
}

void
OnPacketErrorSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorSens++;
}

void
OnPacketErrorWrongSFCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorSF++;
}
void
OnPacketErrorInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorInterf++;
}
void
OnPacketErrorNoDemodulatorCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorNoDemodul++;
}
void
OnPacketErrorFrequencyCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorWrongFreq++;
}

void
OnPacketErrorNoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorNoReceivers++;
}
void
OnPacketErrorGWInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorGWInterf++;
}
void
OnPacketErrorOccupiedReceptionCallback (int oldValue, int newValue)
{
  g_errorOccupied++;
}

int
main (int argc, char *argv[])
{
  //  std::ostringstream os;
  //  std::ofstream bFile ("lorawan-psr-distance.plt");

  int minDistance = 100;
//  int maxDistance = 10000; // meters
//  int increment = 100;
  //  int maxPackets = 1000;
  int packetSize = 20;
  double txPower = 0;
  //  uint32_t channelIndex = 0;

  double frequency1 = 868.1;
  //  double frequency2 = 868.3;
  //  double frequency3 = 868.5;
  //  double frequency4 = 868.7;

  CommandLine cmd;

  cmd.AddValue ("txPower", "transmit power (dBm)", txPower);
  cmd.AddValue ("packetSize", "packet (MSDU) size (bytes)", packetSize);
  //  cmd.AddValue ("channelIndex", "channel index", channelIndex);

  cmd.Parse (argc, argv);

  Gnuplot psrplot = Gnuplot ("lorawan-distance.eps");
  Gnuplot2dDataset psrdataset ("lorawan-psr-vs-distance");

  LogComponentEnable ("EndDeviceLoraPhy", ns3::LOG_LEVEL_ALL);
  LogComponentEnable ("SimpleEndDeviceLoraPhy", ns3::LOG_LEVEL_ALL);
  LogComponentEnable ("SimplePeroNoMucho", ns3::LOG_LEVEL_ALL);

  /************************
  *  Create the channel  *
  ************************/

  // Create the lora channel object
  Ptr<LoraChannel> channel;
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);
  // Create the correlated shadowing component
  Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
      CreateObject<CorrelatedShadowingPropagationLossModel> ();
  // Add the effect to the channel propagation loss
  Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();
  shadowing->SetNext (buildingLoss);
  channel = CreateObject<LoraChannel> (loss, delay);


  //  NS_LOG_INFO ("Creating the channel...");
  //  LogicalLoraChannelHelper m_channelHelper;
  //  m_channelHelper.AddChannel (868.1);
  //  m_channelHelper.AddChannel (868.3);
  //  m_channelHelper.AddChannel (868.5);
  //  auto channelList = m_channelHelper.GetChannelList ();
  //  uint32_t channelListSize = channelList.size ();
  //
  //  if (channelIndex >= channelListSize)
  //    NS_LOG_ERROR ("Invalid channel index, supported channels vector size = " << channelListSize);
  //
  //  Ptr<LogicalLoraChannel> logicalLoraChannel = channelList.at (channelIndex);
  //
  //  os << "Packet (MSDU) size = " << packetSize << " bytes; tx power = " << txPower
  //     << " dBm; channelIndex = " << channelIndex;
  //
  //  // Create the lora channel object
  //  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  //  loss->SetPathLossExponent (3.76);
  //  loss->SetReference (1, 7.7);
  //
  //  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  //
  //  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
  *  Create the helpers  *
  ************************/

  NS_LOG_INFO ("Setting up helpers...");

  //  Ptr<ConstantPositionMobilityModel> mobilityED = CreateObject<ConstantPositionMobilityModel>();
  //  Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator> ();
  //  allocatorED->Add (Vector (minDistance, 0, 3));
  //  mobilityED.SetPosition (allocatorED);
  //  mobilityED.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

//  MobilityHelper mobilityGW;
//  Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator> ();
//  allocatorGW->Add (Vector (0, 0, 15));
//  mobilityGW.SetPositionAllocator (allocatorGW);
//  mobilityGW.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  //  helper.EnablePacketTracking ();
  //  helper.EnablePeriodicGlobalPerformancePrinting ("desempenhoGlobal.txt", Seconds (100));

  /************************
  *  Create End Devices  *
  ************************/

  NS_LOG_INFO ("Creating the end device...");
  //
  //  // Create a set of nodes
  //  NodeContainer endDevices;
  //  endDevices.Create (1);
  //  mobilityED.Install (endDevices);
  //
  //  // Create the LoraPhyHelper
  //  LoraPhyHelper phyHelperED = LoraPhyHelper ();
  //  phyHelperED.SetChannel (channel);
  //  // Create the LoraNetDevices of the end devices
  //  phyHelperED.SetDeviceType (LoraPhyHelper::ED);
  //  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  //  helper.Install (phyHelperED, macHelper, endDevices);
  Ptr<SimpleEndDeviceLoraPhy> endDevice = CreateObject<SimpleEndDeviceLoraPhy> ();
  endDevice->SetFrequency (frequency1);
  Ptr<ConstantPositionMobilityModel> mobilityED = CreateObject<ConstantPositionMobilityModel> ();
  mobilityED->SetPosition (Vector (minDistance, 0, 1.2));
  ;
  endDevice->SetMobility (mobilityED);
  endDevice->SetChannel (channel);
  endDevice->SetSpreadingFactor (7);
  endDevice->SetFrequency (frequency1);
  endDevice->TraceConnectWithoutContext ("StartSending",
                                         MakeCallback (OnTransmissionCallback));
//  endDevice->TraceConnectWithoutContext ("ReceivedPacket",
//                                         MakeCallback (OnPacketReceptionCallback));
//  endDevice->TraceConnectWithoutContext ("LostPacketBecauseWrongSpreadingFactor",
//                                         MakeCallback (OnPacketErrorWrongSFCallback));
//  endDevice->TraceConnectWithoutContext ("LostPacketBecauseWrongFrequency",
//                                         MakeCallback (OnPacketErrorFrequencyCallback));
//  endDevice->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
//                                         MakeCallback (OnPacketErrorNoDemodulatorCallback));
//  endDevice->TraceConnectWithoutContext ("LostPacketBecauseInterference",
//                                         MakeCallback (OnPacketErrorInterferenceCallback));
//  endDevice->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
//                                         MakeCallback (OnPacketErrorSensitivityCallback));
  /*
   * Install Track Sources on devices
   * */
  //  for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End (); node++)
  //    {
  //      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
  //          "StartSending", MakeCallback (OnTransmissionCallback));
  //    }

  /*********************
  *  Create Gateways  *
  *********************/

  NS_LOG_INFO ("Creating the gateways...");

//    NodeContainer gateways;
//    gateways.Create (1);
//
//    mobilityGW.Install (gateways);
//    // Create the LoraPhyHelper
//    LoraPhyHelper phyHelperGW = LoraPhyHelper ();
//    phyHelperGW.SetChannel (channel);
//
//    // Create a netdevice for each gateways
//    phyHelperGW.SetDeviceType (LoraPhyHelper::GW);
//    macHelper.SetDeviceType (LorawanMacHelper::GW);
//    helper.Install (phyHelperGW, macHelper, gateways);

  Ptr<SimpleGatewayLoraPhy> gateway = CreateObject<SimpleGatewayLoraPhy> ();
  Ptr<ConstantPositionMobilityModel> mobilityGW = CreateObject<ConstantPositionMobilityModel> ();
  mobilityGW->SetPosition (Vector(0, 0, 15));
  gateway->SetMobility (mobilityGW);
  gateway->SetChannel (channel);
  gateway->TraceConnectWithoutContext ("ReceivedPacket",
                                       MakeCallback (OnPacketReceptionCallback));
  gateway->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                       MakeCallback (OnPacketErrorNoMoreReceiversCallback));
  gateway->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                       MakeCallback (OnPacketErrorGWInterferenceCallback));
  gateway->TraceConnectWithoutContext ("OccupiedReceptionPaths",
                                       MakeCallback (OnPacketErrorOccupiedReceptionCallback));
  gateway->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                       MakeCallback (OnPacketErrorSensitivityCallback));
  /*
   * Install Track Sources on  gateways
   * */
  //  Ptr<LoraPhy> phy;
  //  for (NodeContainer::Iterator node = gateways.Begin (); node != gateways.End (); node++)
  //    {
  //      phy = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ();
  //      phy->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (OnPacketReceptionCallback));
  //      phy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
  //                                       MakeCallback (OnPacketErrorReceptionCallback));
  //    }

  //  macHelper.SetSpreadingFactorsUp (endDevice, gateways, channel);

  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/
  //    Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (5));

  LoraTxParameters txParams;
  LoraChannelParameters channelParameters;
  txParams.bandwidthHz = 125000;
  txParams.sf = 7;
  txParams.headerDisabled = false;
  txParams.crcEnabled = true;
  txParams.codingRate = 1;
  txParams.nPreamble = 8;
  txParams.lowDataRateOptimizationEnabled =
      LoraPhy::GetTSym (txParams) > MilliSeconds (16) ? true : false;

  Ptr<Packet> packet = Create<Packet> (packetSize);
  //  LoraFrameHeader frameHdr = LoraFrameHeader ();
  //  frameHdr.SetAsUplink ();
  //  frameHdr.SetFPort (1);
  //  frameHdr.SetAddress (LoraDeviceAddress ());
  //  frameHdr.SetAdr (0);
  //  frameHdr.SetAdrAckReq (0);
  //  frameHdr.SetFCnt (0);
  //  pkt->AddHeader (frameHdr);
  //
  //  OneShotSenderHelper oneShotSenderHelper;
  //  oneShotSenderHelper.SetSendTime (Seconds (1));
  //  ApplicationContainer applicationContainer = oneShotSenderHelper.Install (endDevices);

  //  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  //  appHelper.SetPeriod (Seconds (10));
  //  appHelper.SetPacketSize (packetSize);
  //  ApplicationContainer appContainer = appHelper.Install (endDevices);
  //
  //  appContainer.Start (Seconds (0));
  //  appContainer.Stop (Seconds (1000));

  //  LorawanMacHeader macHdr = LorawanMacHeader ();
  //  macHdr.SetMType (ns3::lorawan::LorawanMacHeader::UNCONFIRMED_DATA_UP);
  //  macHdr.SetMajor (1);
  //  pkt->AddHeader (macHdr);
  Time duration;
//  for (int j = minDistance; j < maxDistance + 2; j += increment)
//    {

      endDevice->SetSpreadingFactor (12);
      duration = endDevice->GetOnAirTime (packet, txParams);
      std::cout << duration.GetSeconds() << " Seconds" << std::endl;
//      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 2, 12,
//                           Seconds (duration.GetSeconds ()), frequency1);
//      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 4, 12,
//                           Seconds (duration.GetSeconds ()), frequency1);
//      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 6, 12,
//                           Seconds (duration.GetSeconds ()), frequency1);
//      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 8, 12,
//                           Seconds (duration.GetSeconds ()), frequency1);
//      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 10,
//                           12, Seconds (duration.GetSeconds ()), frequency1);
//      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 12,
//                           12, Seconds (duration.GetSeconds ()), frequency1);
      Simulator::Schedule (Seconds (1), &SimpleGatewayLoraPhy::StartReceive, gateway, packet, 14,
                           7, Seconds (duration.GetSeconds ()), frequency1);
//
//      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
//                           frequency1, 2);
//      endDevice->SwitchToStandby();
//      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
//                           frequency1, 4);
//      endDevice->SwitchToStandby();
//      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
//                           frequency1, 6);
//      endDevice->SwitchToStandby();
//      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
//                           frequency1, 8);
//      endDevice->SwitchToStandby();
//      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
//                           frequency1, 10);
//      endDevice->SwitchToStandby();
//      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
//                           frequency1, 12);
//      endDevice->SwitchToStandby();
      Simulator::Schedule (Seconds (1), &SimpleEndDeviceLoraPhy::Send, endDevice, packet, txParams,
                           frequency1, 14);
      endDevice->SwitchToStandby();

      Simulator::Run ();
      std::cout << "Sent " << g_sended << " Received " << g_received << " Errors WF"
                << g_errorWrongFreq << " GWInt" << g_errorGWInterf << " Int" << g_errorInterf
                << " NoDem" << g_errorNoDemodul << " SF" << g_errorSF << " Sens" << g_errorSens
                << " NoRec" << g_errorNoReceivers << " Occu" << g_errorOccupied
                << " packets for distance " << std::endl;

      //      psrdataset.Add (j, g_received / (maxPackets / 10));

      //      std::cout << "Distance " << endDevice->GetMobility()->GetDistanceFrom (gateway->GetMobility());
      mobilityED->SetPosition (Vector (500, 0, 0));
      endDevice->SetMobility (mobilityED);
//    }

  //  psrplot.AddDataset (psrdataset);
  //  psrplot.SetTitle (os.str ());
  //  psrplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  //  psrplot.SetLegend ("distance (m)", "Packet Success Rate (PSR)");
  //  psrplot.SetExtra ("set xrange [0:200]\nset yrange [0:1]\nset grid\nset style line 1 linewidth "
  //                    "5\nset style increment user");
  //  psrplot.GenerateOutput (bFile);
  //  bFile.close ();

  Simulator::Destroy ();
  //
  //  for (int sf = 7; sf < 13; sf++)
  //    {
  //      std::cout << sf << " S " << packetsSent.at (sf - 7) << " R " << packetsReceived.at (sf - 7)
  //                << " E " << packetsRxError.at (sf - 7) << std::endl;
  //    }
  //  helper.DoPrintGlobalPerformance ("desempenhoGlobal.dat");
  return 0;
}
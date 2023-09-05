/*
* This program creates a simple network which uses an ADR algorithm to set up
* the Spreading Factors of the devices in the Network.
 *
 * Trial #1: Set All DRs on each execution
*/

#include "ns3/point-to-point-module.h"
#include "ns3/lora-channel.h"
#include "ns3/mobility-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/lorawan-mac.h"
#include "ns3/lora-helper.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/config.h"
#include "ns3/object.h"
#include "ns3/rectangle.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Example2");

Ptr<Packet> m_latestReceivedPacket;
int m_receivedPacketCalls = 0;
int m_underSensitivityCalls = 0;
int m_interferenceCalls = 0;
int m_wrongFrequencyCalls = 0;

void
ReceivedPacket (Ptr<const Packet> packet, uint32_t node)
{
  NS_LOG_FUNCTION (packet << node);

  m_receivedPacketCalls++;

  m_latestReceivedPacket = packet->Copy ();
}

// Trace packets
void
UnderSensitivity (Ptr<const Packet> packet, uint32_t node)
{
  NS_LOG_FUNCTION (packet << node);

  m_underSensitivityCalls++;
}

// Trace sources that are called when a node changes its DR or TX power
void
OnDataRateChange (uint8_t oldDr, uint8_t newDr)
{
  //  NS_LOG_DEBUG ("DR" << unsigned (oldDr) << " -> DR" << unsigned (newDr));
  std::cout << "DR" << unsigned (oldDr) << " -> DR" << unsigned (newDr) << std::endl;
}

void
Interference (Ptr<const Packet> packet, uint32_t node)
{
  NS_LOG_FUNCTION (packet << node);

  m_interferenceCalls++;
}

void
WrongFrequency (Ptr<const Packet> packet, uint32_t node)
{
  NS_LOG_FUNCTION (packet << node);

  m_wrongFrequencyCalls++;
}

Ptr<EndDeviceLoraPhy>
GWReset (Ptr<LoraChannel> channel, Ptr<EndDeviceLoraPhy> gwPhy)
{
  m_receivedPacketCalls = 0;
  m_underSensitivityCalls = 0;
  m_interferenceCalls = 0;
  m_wrongFrequencyCalls = 0;
  // Create the LoraNetDevices of the gateways

  gwPhy->SetFrequency (868.1);
  gwPhy->SetChannel (channel);
  Ptr<ConstantPositionMobilityModel> mobilityGw = CreateObject<ConstantPositionMobilityModel> ();
  mobilityGw->SetPosition (Vector (0, 0, 0));
  gwPhy->SetMobility (mobilityGw);

  channel->Add (gwPhy);
  gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                     MakeCallback (&UnderSensitivity));
  gwPhy->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&ReceivedPacket));
  gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference", MakeCallback (&Interference));
  gwPhy->TraceConnectWithoutContext ("LostPacketBecauseWrongFrequency",
                                     MakeCallback (&WrongFrequency));

  return gwPhy;
}

void
OnTxPowerChange (double oldTxPower, double newTxPower)
{
  //  NS_LOG_DEBUG (oldTxPower << " dBm -> " << newTxPower << " dBm");
  std::cout << oldTxPower << " dBm -> " << newTxPower << " dBm" << std::endl;
}

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  Ptr<LoraChannel> channel;
  Ptr<EndDeviceLoraPhy> gwPhy = CreateObject<SimpleEndDeviceLoraPhy> ();

  /*
   * Channel -> GW
   * */
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);
  Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
      CreateObject<CorrelatedShadowingPropagationLossModel> ();
  Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();
  shadowing->SetNext (buildingLoss);
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel = CreateObject<LoraChannel> (loss, delay);

  // Helpers para EDs
  // Create the LoraPhyHelper -> ED
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);
  // Create the LoraMacHelper -> ED
  LorawanMacHelper macHelper = LorawanMacHelper ();
  MobilityHelper mobilityEd;
  // Create the LoraHelper -> ED
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();

  Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator> ();
  for (int i = 100; i < 10001; i += 100)
    {
      allocatorED->Add (Vector (i, 0, 0));
      mobilityEd.SetPositionAllocator (allocatorED);
    }
  mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  NodeContainer endDevices;
  endDevices.Create (100);

  // Install mobility model on fixed nodes
  mobilityEd.Install (endDevices);

  // Create a LoraDeviceAddressGenerator
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetRegion (LorawanMacHelper::EU);
  helper.Install (phyHelper, macHelper, endDevices);

  //  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  // Connect our traces
  Config::ConnectWithoutContext (
      "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
      MakeCallback (&OnTxPowerChange));
  Config::ConnectWithoutContext (
      "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
      MakeCallback (&OnDataRateChange));

  Time stateSamplePeriod = Seconds (20);
  std::string global = "globalPerformanceEX2.dat";
  helper.EnablePeriodicGlobalPerformancePrinting (global, stateSamplePeriod);

  // Start simulation
  gwPhy = GWReset (channel, gwPhy);
  LoraTxParameters txParameters;
  txParameters.sf = 12;
  txParameters.lowDataRateOptimizationEnabled=true;
  Ptr<Packet> packet = Create<Packet> (24);
  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  Simulator::Stop (Hours (2));
  Simulator::Run ();
  Simulator::Destroy ();

  NS_LOG_INFO ("Computing performance metrics...");

  std::cout << "[GW]" << std::endl
            << "TX-RX:" << gwPhy->TX << "-" << gwPhy->RX
            << std::endl;
  std::cout << "[Data]" << std::endl
            << "Received:" << m_receivedPacketCalls << " Under Sens:" << m_underSensitivityCalls
            << " Interference:" << m_interferenceCalls << " Wrong Freq:" << m_wrongFrequencyCalls
            << std::endl;
  return 0;
}
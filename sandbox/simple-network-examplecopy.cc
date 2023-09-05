/*
 * This script simulates a simple network in which one end device sends one
 * packet to the gateway.
 */
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/lorawan-module.h"
#include "ns3/propagation-module.h"
#include <algorithm>
#include <ctime>
#include <iomanip>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("SimpleLorawanNetworkExample");

NodeContainer endDevices;
NodeContainer gateways;
Ptr<LoraChannel> channel;
MobilityHelper mobilityED, mobilityGW;

int noMoreReceivers = 0;
int interfered = 0;
int received = 0;
int underSensitivity = 0;
int sent = 0;

int nDevices = 0;
int nGateways = 0;

/**********************
 *  Global Callbacks  *
 **********************/

enum PacketOutcome { _RECEIVED, _INTERFERED, _NO_MORE_RECEIVERS, _UNDER_SENSITIVITY, _UNSET };

struct myPacketStatus
{
  Ptr<Packet const> packet;
  uint32_t senderId;
  int outcomeNumber;
  std::vector<enum PacketOutcome> outcomes;
};

std::map<Ptr<Packet const>, myPacketStatus> packetTracker;

void
CheckReceptionByAllGWsComplete (std::map<Ptr<Packet const>, myPacketStatus>::iterator it)
{
  // Check whether this packet is received by all gateways
  if ((*it).second.outcomeNumber == nGateways)
    {
      // Update the statistics
      myPacketStatus status = (*it).second;
      for (int j = 0; j < nGateways; j++)
        {
          switch (status.outcomes.at (j))
            {
              case _RECEIVED: {
                received += 1;
                break;
              }
              case _UNDER_SENSITIVITY: {
                underSensitivity += 1;
                break;
              }
              case _NO_MORE_RECEIVERS: {
                noMoreReceivers += 1;
                break;
              }
              case _INTERFERED: {
                interfered += 1;
                break;
              }
              case _UNSET: {
                break;
              }
            }
        }
      // Remove the packet from the tracker
      packetTracker.erase (it);
    }
}

void
TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("Transmitted a packet from device " << systemId);
  // Create a packetStatus
  myPacketStatus status;
  status.packet = packet;
  status.senderId = systemId;
  status.outcomeNumber = 0;
  status.outcomes = std::vector<enum PacketOutcome> (nGateways, _UNSET);

  packetTracker.insert (std::pair<Ptr<Packet const>, myPacketStatus> (packet, status));
}

void
PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // Remove the successfully received packet from the list of sent ones
  // NS_LOG_INFO ("A packet was successfully received at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _RECEIVED;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because of interference at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _INTERFERED;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because there were no more receivers at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _NO_MORE_RECEIVERS;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet arrived at the gateway under sensitivity at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _UNDER_SENSITIVITY;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

/**
* Places the end devices according to the allocator object in the input file..
* @param filename: arquivo de entrada
* @return number of devices
**/
int
EndDevicesPlacement (std::string filename)
{
  double edX = 0.0, edY = 0.0, edZ = 0.0;
  int nDev = 0;
  Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator> ();
  const char *c = filename.c_str ();
  // Get Devices position from File
  std::ifstream in_File (c);
  if (!in_File)
    {
      std::cout << "Could not open the file - '" << filename << "'" << std::endl;
    }
  else
    {
      while (in_File >> edX >> edY >> edZ)
        {
          allocatorED->Add (Vector (edX, edY, edZ));
          nDev++;
        }
      in_File.close ();
    }

  endDevices.Create (nDev);
  mobilityED.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityED.SetPositionAllocator (allocatorED);
  mobilityED.Install (endDevices);
  return nDev;
}

/**
* Places the gateways according to the allocator object in the input file..
* @param filename: arquivo de entrada
* @return number of gateways
**/
int
GatewaysPlacement (std::string filename)
{
  double gwX = 0.0, gwY = 0.0, gwZ = 0.0;
  Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator> ();
  const char *c = filename.c_str ();
  // Get Devices position from File
  std::ifstream in_File (c);
  int nGat = 0;
  if (!in_File)
    {
      std::cout << "Could not open the file - '" << filename << "'" << std::endl;
    }
  else
    {
      while (in_File >> gwX >> gwY >> gwZ)
        {
          allocatorGW->Add (Vector (gwX, gwY, gwZ));
          nGat++;
        }
      in_File.close ();
    }
  gateways.Create (nGat);
  mobilityGW.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityGW.SetPositionAllocator (allocatorGW);
  mobilityGW.Install (gateways);
  return nGat;
}

/**
* Check if the number of gateways accomplishes to device's coverage rate.
* @param coverageRate :: expected coverage rate
* @return pair<bool, int> :: whether coverage rate is achieved and the number of required gateways
**/
std::pair<bool, int>
NodeCoverage (double coverageRate)
{
  int reached = 0, requiredGateways = 0;
  // checking node coverage
  const double sensitivity[6] = {-130.0, -132.5, -135.0, -137.5, -140.0, -142.5};
  for (NodeContainer::Iterator gw = gateways.Begin (); gw != gateways.End (); ++gw)
    {
      Ptr<Node> oGateway = *gw;
      reached = 0;
      for (NodeContainer::Iterator ed = endDevices.Begin (); ed != endDevices.End (); ++ed)
        {
          Ptr<Node> oDevice = *ed;
          Ptr<MobilityModel> mobGW = oGateway->GetObject<MobilityModel> ();
          Ptr<MobilityModel> mobED = oDevice->GetObject<MobilityModel> ();
          double rx = channel->GetRxPower (14, mobGW, mobED);

          if (rx - sensitivity[5] > 0)
            {
              reached++;
            }
        }
      requiredGateways += 1;
      if (reached / endDevices.GetN () > coverageRate)
        break;
    }
  NS_LOG_INFO ("Required Gateways: " + std::to_string (requiredGateways) +
               " Coverage rate: " + std::to_string (reached / endDevices.GetN ()));
  return {reached / endDevices.GetN () > coverageRate, requiredGateways};
}

void
PrintPacketRates (std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream ratesFile;
  ratesFile.open (c, std::ios::app);

  if (!ratesFile)
    {
      ratesFile.open (c, std::ios::out);
    }
  double receivedRate = double (received) / (sent);
  double error = (noMoreReceivers + interfered + underSensitivity) / nGateways;
  ratesFile << nDevices << " " << nGateways << " " << sent << " " << received << " " << receivedRate
            << " " << error <<  std::endl;
  ratesFile.close ();
}

int
main (int argc, char *argv[])
{

  double expCoverage = 0.90; // expected coverage rate (0.01 to 1.00)
  double simulationTime = 600;
  int appPeriodSeconds = 10;
  bool okumura = false;
  bool printRates = true;
  bool verbose = false;

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("nGateways",
                "Number of gateways to include in the simulation"
                " \n {1, 2, 4, 9, 25, 81, 289, 1089}",
                nGateways);
  cmd.AddValue ("expCoverage", "Expected node coverage", expCoverage);
  cmd.AddValue ("okumura", "Uses okumura-hate propagation mode", okumura);
  cmd.AddValue ("verbose", "Whether to print output or not", verbose);
  cmd.AddValue ("printRates", "Whether to print result rates", printRates);
  cmd.Parse (argc, argv);

  // Set up logging
  if (verbose)
    {
      LogComponentEnable ("SimpleLorawanNetworkExample", LOG_LEVEL_ALL);
        LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
        LogComponentEnable ("LoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ALL);
        LogComponentEnable ("LorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable ("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable ("GatewayLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
        LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_ALL);
        LogComponentEnable ("LoraHelper", LOG_LEVEL_ALL);
        LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_ALL);
        LogComponentEnable ("LorawanMacHelper", LOG_LEVEL_ALL);
        LogComponentEnable ("OneShotSenderHelper", LOG_LEVEL_ALL);
        LogComponentEnable ("OneShotSender", LOG_LEVEL_ALL);
        LogComponentEnable ("LorawanMacHeader", LOG_LEVEL_ALL);
        LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_ALL);
        LogComponentEnableAll (LOG_PREFIX_FUNC);
        LogComponentEnableAll (LOG_PREFIX_NODE);
        LogComponentEnableAll (LOG_PREFIX_TIME);
    }
  /************************
  *  Create the channel  *
  ************************/

  // Create the lora channel object
  // modelo de propagação (okumura ou logdistance)
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  if (okumura)
    {
      Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel> ();
      channel = CreateObject<LoraChannel> (loss, delay);
    }
  else
    {
      Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
      loss->SetPathLossExponent (3.76);
      loss->SetReference (1, 10.0);
      channel = CreateObject<LoraChannel> (loss, delay);
    }

  /************************
  *  Create the helpers  *
  ************************/

  NS_LOG_INFO ("Setting up helpers...");

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();

  /************************
  *  Create End Devices  *
  ************************/

  NS_LOG_INFO ("Creating end devices...");
  // Create a set of nodes
  EndDevicesPlacement (
      "/home/rogerio/git/sim-res/datafile/4x4x2/in/endDevicesPositions.dat");

  // Assign a mobility model to the node
  mobilityED.Install (endDevices);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      phy->TraceConnectWithoutContext ("StartSending", MakeCallback (&TransmissionCallback));
      nDevices++;
    }

  /*********************
  *  Create Gateways  *
  *********************/

  NS_LOG_INFO ("Creating gateways...");

  std::string filename = "/home/rogerio/git/sim-res/datafile/equi-cartesian/"
                         "to_NS3/cartesianPlacement_" +
                         std::to_string (nGateways) + ".dat";
  nGateways = GatewaysPlacement (filename) ;

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  // Install reception paths on gateways
  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); j++)
    {
      Ptr<Node> object = *j;
      // Get the device
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<GatewayLoraPhy> gwPhy =
          netDevice->GetObject<LoraNetDevice> ()->GetPhy ()->GetObject<GatewayLoraPhy> ();
      // Global callbacks (every gateway)
      gwPhy->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&PacketReceptionCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                         MakeCallback (&InterferenceCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                         MakeCallback (&NoMoreReceiversCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                         MakeCallback (&UnderSensitivityCallback));
    }

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/

  Time appStopTime = Seconds (simulationTime); // one hour
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds)); // each minute
  appHelper.SetPacketSize (41);
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);
  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();
  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  // Create a NS for the network
  nsHelper.SetAdr ("ns3::AdrComponent");
  nsHelper.EnableAdr (true);
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (appStopTime + Minutes (10));

  Simulator::Run ();

  Simulator::Destroy ();
  if (printRates)
    {
      NS_LOG_INFO ("Computing performance metrics...");

      std::string ratesFile =
          "/home/rogerio/git/sim-res/datafile/4x4x2/out/transmissionData_";
      ratesFile = ratesFile + std::to_string (gateways.GetN ()) + ".dat";
      PrintPacketRates (ratesFile);
    }
  return 0;
}



// /**********************
// *  Global Callbacks  *
// **********************/
//
//void
//TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
//{
//  // NS_LOG_INFO ("Transmitted a packet from device " << systemId);
//  // Create a packetStatus
//  sent++;
//}
//
//void
//PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
//{
//  // Remove the successfully received packet from the list of sent ones
//  // NS_LOG_INFO ("A packet was successfully received at gateway " << systemId);
//  received++;
//}
//
//void
//InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
//{
//  // NS_LOG_INFO ("A packet was lost because of interference at gateway " << systemId);
//  interfered++;
//}
//
//void
//NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
//{
//  // NS_LOG_INFO ("A packet was lost because there were no more receivers at gateway " << systemId);
//  noMoreReceivers++;
//}
//
//void
//UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
//{
//  NS_LOG_INFO ("A packet arrived at the gateway under sensitivity at gateway " << systemId);
//  underSensitivity++;
//}

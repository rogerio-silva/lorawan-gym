/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest in this script is the throughput of the
 * network.
 */

#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
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
#include <algorithm>
#include <ctime>
#include "ns3/mpi-interface.h"

#ifdef NS3_MPI
#include <mpi.h>
#else
#error "ndn-simple-mpi scenario can be compiled only if NS3_MPI is enabled"
#endif

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 500;
int nGateways = 8;
double sideLength = 5000;
double simulationTime = 600; //600
int appPeriodSeconds = 600; //600
std::vector<int> sfQuantity (6);

int noMoreReceivers = 0;
int interfered = 0;
int received = 0;
int underSensitivity = 0;
std::string file_prefix = "EDs_x_SF";
std::string file_extension = ".dat";
bool okumura = false;
bool realisticChannelModel = false;

// Output control
bool printEDs = false;
bool buildingsEnabled = false;
bool printToA = false;
bool printProb = true;
bool nullmsg = false;
bool verbose = false;

time_t oldtime = std::time (0);

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

// Periodically print simulation time
void
PrintSimulationTime (void)
{
  // NS_LOG_INFO ("Time: " << Simulator::Now().GetHours());
  std::cout << "Simulated time: " << Simulator::Now ().GetHours () << " hours" << std::endl;
  std::cout << "Real time from last call: " << std::time (0) - oldtime << " seconds" << std::endl;
  oldtime = std::time (0);
  Simulator::Schedule (Minutes (30), &PrintSimulationTime);
}

void
PrintToA (std::string file_name)
{
  /***********************
  * Print TimeOnAir Duration  *
  ************************/
  Time duration;
  Ptr<Packet> packet = Create<Packet> (10);
  LoraTxParameters txParams;
  for (int cr = 0; cr < 5; ++cr)
    {
      std::string filename = file_name;
      filename += std::to_string (cr) + ".dat";
      const char *c = filename.c_str ();
      std::ofstream toaFile;
      toaFile.open (c);
      for (int sf = 7; sf < 13; ++sf)
        {
          txParams.sf = sf;
          txParams.headerDisabled = false;
          txParams.codingRate = cr;
          txParams.bandwidthHz = 125000;
          txParams.nPreamble = 8;
          txParams.crcEnabled = 1;
          txParams.lowDataRateOptimizationEnabled = 0;
          duration = LoraPhy::GetOnAirTime (packet, txParams);
          //          toaFile << "ToA (SF "<< sf <<", CR "<< cr <<", BW 125kHz): " << duration.GetSeconds () << std::endl;
          toaFile << sf << " " << cr << " " << duration.GetSeconds () << std::endl;
        }
      toaFile.close ();
    }
}

void
PrintEndDevicesPositionWithSF (NodeContainer endDevices, std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  spreadingFactorFile << "x, y, z, s" << std::endl;
  std::cout << "N Dev: " << endDevices.GetN () << std::endl;
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      int sf = int (mac->GetDataRate ());
      Vector pos = position->GetPosition ();
      spreadingFactorFile << pos.x << "," << pos.y << "," << pos.z << "," << sf << std::endl;
    }
  spreadingFactorFile.close ();
}

// // Also print the gateways
//for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
//  {
//    Ptr<Node> object = *j;
//    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
//    Vector pos = position->GetPosition ();
//    spreadingFactorFile << pos.x << " " << pos.y << " GW" << std::endl;
//  }

void
PrintEndDevicesParameters (NodeContainer endDevices, NodeContainer gateways, std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  for (NodeContainer::Iterator gw = gateways.Begin (); gw != gateways.End (); ++gw)
    {
      Ptr<Node> oGateway = *gw;
      for (NodeContainer::Iterator ed = endDevices.Begin (); ed != endDevices.End (); ++ed)
        {
          Ptr<Node> oDevice = *ed;
          double distanceFromGW = oDevice->GetObject<MobilityModel> ()->GetDistanceFrom (
              oGateway->GetObject<MobilityModel> ());
          NS_ASSERT (distanceFromGW != 0);
          Ptr<NetDevice> netDevice = oDevice->GetDevice (0);
          Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
          NS_ASSERT (loraNetDevice != 0);
          Ptr<EndDeviceLorawanMac> mac =
              loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
          int sf = mac->GetSfFromDataRate (mac->GetDataRate ());
          int txPower = mac->GetTransmissionPower ();
          int dBm = mac->GetDbmForTxPower (mac->GetTransmissionPower ());
          std::vector<double> txDbmForTxPower;
          mac->SetTxDbmForTxPower (txDbmForTxPower);
          spreadingFactorFile << oDevice->GetId () << " " << distanceFromGW << " " << sf << " "
                              << txPower << " " << dBm << std::endl;
        }
    }
  spreadingFactorFile.close ();
}

int
main (int argc, char *argv[])
{
  /***********
  *  Seed    *
  ***********/

  ns3::RngSeedManager::SetSeed (1971);

  /*****************************
  *  Setup via command prompt  *
  ******************************/

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("rings", "Number of gateway rings to include", nGateways);
  cmd.AddValue ("sideLength", "The sideLength of the area to simulate", sideLength);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue ("printEDs", "Whether or not to print a file containing the ED's positions",
                printEDs);
  cmd.AddValue ("fName", "File Name", file_prefix);
  cmd.AddValue ("okumura", "The sideLength of the area to simulate", okumura);
  cmd.AddValue ("printToA", "Whether or not generate ToA durations file", printToA);
  cmd.AddValue ("nullmsg", "Enable the use of null-message synchronization", nullmsg);
  cmd.Parse (argc, argv);

  // Enable parallel simulator with the command line arguments
  if (nullmsg)
    {
      GlobalValue::Bind ("SimulatorImplementationType",
                         StringValue ("ns3::NullMessageSimulatorImpl"));
    }
  else
    {
      GlobalValue::Bind ("SimulatorImplementationType",
                         StringValue ("ns3::DistributedSimulatorImpl"));
    }
  MpiInterface::Enable (&argc, &argv);

  // Set up logging {verbose = true}
  if (verbose)
    {
      LogComponentEnable ("ComplexLorawanNetworkExample", ns3::LOG_LEVEL_ALL);
      LogComponentEnable ("LoraChannel", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraPhy", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("EndDeviceLoraPhy", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("GatewayLoraPhy", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraInterferenceHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraMac", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("EndDeviceLoraMac", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("GatewayLoraMac", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LogicalLoraChannelHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LogicalLoraChannel", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraPhyHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraMacHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("PeriodicSenderHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("PeriodicSender", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraMacHeader", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraFrameHeader", ns3::LOG_LEVEL_DEBUG);
    }
  uint32_t systemId = MpiInterface::GetSystemId ();
  //  uint32_t systemCount = MpiInterface::GetSize ();
  std::cout << "SysId: " << systemId << std::endl;

  /****************
  *  Setup Network*
  *****************/

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator", "X",
                                 PointerValue (CreateObjectWithAttributes<UniformRandomVariable> (
                                     "Min", DoubleValue (-sideLength), "Max", DoubleValue (sideLength))),
                                 "Y",
                                 PointerValue (CreateObjectWithAttributes<UniformRandomVariable> (
                                     "Min", DoubleValue (-sideLength), "Max", DoubleValue (sideLength))));
  //  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (sideLength),
  //                                 "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  std::cout << "SysId: " << systemId << "Mobility " << mobility.GetMobilityModelType() << std::endl;

  /************************
  *  Create the channel  *
  ************************/

  // Create the lora channel object
  // modelo de propagação (okumura ou logdistance)
  Ptr<LoraChannel> channel;
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

      if (realisticChannelModel)
        {
          // Create the correlated shadowing component
          Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
              CreateObject<CorrelatedShadowingPropagationLossModel> ();

          // Add the effect to the channel propagation loss
          Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

          shadowing->SetNext (buildingLoss);
        }
      channel = CreateObject<LoraChannel> (loss, delay);
    }
  std::cout << "SysId: " << systemId << "Channel " << channel->GetId() << std::endl;

  /************************
  *  Create the helpers  *
  ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LoraMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();

  /************************
  *  Create End Devices  *
  ************************/
  NodeContainer endDevices, endDevices1, endDevices2, endDevices3;
  if (systemId != 3)
    {
      // Create a set of nodes

      //Devices serão processados pelos 1º, 2º e 3º processadores
      endDevices1.Create (nDevices / 3, 0);
      endDevices2.Create (nDevices / 3, 1);
      endDevices3.Create (nDevices / 3 + 1, 2);

      mobility.Install (endDevices1);
      mobility.Install (endDevices2);
      mobility.Install (endDevices3);

      // Make it so that nodes are at a certain height > 0
      Ptr<UniformRandomVariable> rz = CreateObject<UniformRandomVariable> ();
      rz->SetAttribute ("Min", DoubleValue (1.0));
      rz->SetAttribute ("Max", DoubleValue (3.0));

      // Create the LoraNetDevices of the end devices
      phyHelper.SetDeviceType (LoraPhyHelper::ED);
      macHelper.SetDeviceType (LorawanMacHelper::ED_A);
      helper.Install (phyHelper, macHelper, endDevices1);
      helper.Install (phyHelper, macHelper, endDevices2);
      helper.Install (phyHelper, macHelper, endDevices3);
      endDevices.Add (endDevices1);
      endDevices.Add (endDevices2);
      endDevices.Add (endDevices3);
      std::cout << endDevices1.GetN () << std::endl;

      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
        {
          Ptr<MobilityModel> mobilityED = (*j)->GetObject<MobilityModel> ();
          Vector position = mobilityED->GetPosition ();
          position.z = rz->GetValue ();
          mobilityED->SetPosition (position);

          Ptr<Node> node = *j;
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
          Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
          phy->TraceConnectWithoutContext ("StartSending", MakeCallback (&TransmissionCallback));
        }

      std::cout << "SysId: " << systemId << "Devices Created " << std::endl;
    }
  /*********************
  *  Create Gateways  *
  *********************/
  NodeContainer gateways;
  if(systemId==3)
    {
      // Create the gateway nodes (allocate them uniformly on the disc)

      //Gateways serão processados pelo 4º processador
      gateways.Create (nGateways, 3);

      Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
      allocator->Add (Vector (-2000.0, 2000.0, 15.0));
      allocator->Add (Vector (2000.0, 2000.0, 15.0));
      allocator->Add (Vector (2000.0, -2000.0, 15.0));
      allocator->Add (Vector (-2000.0, -2000.0, 15.0));
      allocator->Add (Vector (-2000.0, 2000.0, 15.0));
      allocator->Add (Vector (2000.0, 2000.0, 15.0));
      allocator->Add (Vector (2000.0, -2000.0, 15.0));
      allocator->Add (Vector (-2000.0, -2000.0, 15.0));

      mobility.SetPositionAllocator (allocator);
      mobility.Install (gateways);

      // Create a netdevice for each gateway
      phyHelper.SetDeviceType (LoraPhyHelper::GW);
      macHelper.SetDeviceType (LorawanMacHelper::GW);
      helper.Install (phyHelper, macHelper, gateways);

      /************************
      *  Configure Gateways  *
      ************************/

      // Install reception paths on gateways
      for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); j++)
        {

          Ptr<Node> object = *j;
          // Get the device
          Ptr<NetDevice> netDevice = object->GetDevice (0);
          Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
          NS_ASSERT (loraNetDevice != 0);
          Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();

          // Global callbacks (every gateway)
          gwPhy->TraceConnectWithoutContext ("ReceivedPacket",
                                             MakeCallback (&PacketReceptionCallback));
          gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                             MakeCallback (&InterferenceCallback));
          gwPhy->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                             MakeCallback (&NoMoreReceiversCallback));
          gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                             MakeCallback (&UnderSensitivityCallback));
        }
      std::cout << "SysId: " << systemId << "Gateways Created " << std::endl;
    }
  /**********************************************
  *  Set up the end device's spreading factor  *
  **********************************************/

  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/
  Time appStopTime = Seconds (simulationTime);
  if (systemId != 3)
    {
      PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
      appHelper.SetPeriod (Seconds (appPeriodSeconds));
      ApplicationContainer appContainer;
      appContainer.Start (Seconds (0));
      appContainer.Stop (appStopTime);
    }
  std::cout << "SysId: " << systemId << "Applications installed (Devices) " << std::endl;
  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  if(systemId==3)
    {
      networkServer.Create (1, 3);
      //Create the NetworkServerHelper
      NetworkServerHelper nsHelper = NetworkServerHelper ();

      // Create a NS for the network
      if (systemId != 3)
        {
          nsHelper.SetEndDevices (endDevices1);
          nsHelper.SetEndDevices (endDevices2);
          nsHelper.SetEndDevices (endDevices3);
        }
      else
        {
          nsHelper.SetEndDevices (gateways);
        }

      nsHelper.Install (networkServer);

      //Create the ForwarderHelper
      ForwarderHelper forHelper = ForwarderHelper ();
      //Create a forwarder for each gateway
      forHelper.Install (gateways);
      std::cout << "SysId: " << systemId << "Network Server installed" << std::endl;
    }
  /**********************
   * Print output files *
   *********************/

  if (printEDs)
    {
      std::string ss_filename, par_filename;
      std::string propagation;
      if (okumura)
        propagation = "OHA";
      else
        propagation = "LOG";

      ss_filename = "/home/rogerio/git/sim-res/endDevicesPlacement_" +
                    std::to_string (nDevices) + file_extension;
      par_filename = "Param-" + propagation + "-" + file_prefix + "_" + std::to_string (nDevices) +
                     "_" + std::to_string (nGateways) + file_extension;
      PrintEndDevicesPositionWithSF (endDevices, ss_filename);
      PrintEndDevicesParameters (endDevices, gateways, par_filename);
    }

  if (printToA)
    {
      std::string toa_filename;
      toa_filename = "ToA-Durations-per-SF-and-CR";
      PrintToA (toa_filename);
    }

  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (appStopTime + Hours (2));

  PrintSimulationTime ();

  Simulator::Run ();
  std::cout << "SysId: " << systemId << "End of Run " << std::endl;
  Simulator::Destroy ();
  std::cout << "SysId: " << systemId << "The End " << std::endl;
  /*************
  *  Results  *
  *************/
  if (printProb)
    {
      double receivedProb = double (received) / nDevices;
      double interferedProb = double (interfered) / nDevices;
      double noMoreReceiversProb = double (noMoreReceivers) / nDevices;
      double underSensitivityProb = double (underSensitivity) / nDevices;
      double receivedProbGivenAboveSensitivity = double (received) / (nDevices - underSensitivity);
      double interferedProbGivenAboveSensitivity =
          double (interfered) / (nDevices - underSensitivity);
      double noMoreReceiversProbGivenAboveSensitivity =
          double (noMoreReceivers) / (nDevices - underSensitivity);
      std::cout << "Received: " << received << "#DEV: " << nDevices
                << " DEV/TEMP0: " << double (nDevices) / simulationTime
                << " RECEIVED: " << receivedProb << " INTERFERRED: " << interferedProb
                << " NO MORE RECEIVED: " << noMoreReceiversProb
                << " UNDER SENSIT: " << underSensitivityProb
                << " ABOVE SENS: " << receivedProbGivenAboveSensitivity
                << " INTERFERRED ABOVE: " << interferedProbGivenAboveSensitivity
                << " NO MORE ABOVE: " << noMoreReceiversProbGivenAboveSensitivity << std::endl;
    }
  MpiInterface::Disable ();
  return 0;
}

// Print the packetTracker contents
//     std::cout << "Packet outcomes" << std::endl;
//     std::map<Ptr<Packet const>, myPacketStatus>::iterator i;
//     for (i = packetTracker.begin (); i != packetTracker.end (); i++)
//       {
//         myPacketStatus status = (*i).second;
//         std::cout.width (4);
//         std::cout << status.senderId << "\t";
//         for (int j = 0; j < nGateways; j++)
//           {
//             switch (status.outcomes.at (j))
//               {
//               case _RECEIVED:
//                 {
//                   std::cout << "R ";
//                   break;
//                 }
//               case _UNDER_SENSITIVITY:
//                 {
//                   std::cout << "U ";
//                   break;
//                 }
//               case _NO_MORE_RECEIVERS:
//                 {
//                   std::cout << "N ";
//                   break;
//                 }
//               case _INTERFERED:
//                 {
//                   std::cout << "I ";
//                   break;
//                 }
//               case _UNSET:
//                 {
//                   std::cout << "E ";
//                   break;
//                 }
//               }
//           }
//         std::cout << std::endl;
//       }

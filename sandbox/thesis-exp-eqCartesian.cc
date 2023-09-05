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
#include <iomanip>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ThesisExpEqCartesian");

// Network settings
int nDevices = 0;
int nGateways = 0;
double simulationTime = 3600;
int appPeriodSeconds = 10;

int noMoreReceivers = 0;
int interfered = 0;
int received = 0;
int underSensitivity = 0;
int sent = 0;
bool okumura = false;
bool realisticChannelModel = false;

// Output control
bool printEDs = false;
bool printRate = false;
bool verbose = false;

NodeContainer endDevicesContainer = NodeContainer ();
NodeContainer gatewaysContainer = NodeContainer ();

/**
* This method positions the end devices according to the input file.
* @param filename: arquivo de entrada
* @return number of devices
**/
int
EndDevicesPlacement (std::string filename)
{
  double edX = 0.0, edY = 0.0, edZ = 0.0;
  int nDev = 0;
  Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator> ();
  MobilityHelper mobilityED;
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

  endDevicesContainer.Create (nDev);
  mobilityED.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityED.SetPositionAllocator (allocatorED);
  mobilityED.Install (endDevicesContainer);
  return nDev;
}

/**
* This method positions the gateways according to the input file.
* @param filename: arquivo de entrada
* @return number of gateways
**/

int
GatewaysPlacement (std::string filename)
{
  double gwX = 0.0, gwY = 0.0, gwZ = 0.0;
  int nDev = 0;
  Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator> ();
  MobilityHelper mobilityGW;
  const char *c = filename.c_str ();
  // Get Devices position from File
  std::ifstream in_File (c);
  if (!in_File)
    {
      std::cout << "Could not open the file - '" << filename << "'" << std::endl;
    }
  else
    {
      while (in_File >> gwX >> gwY >> gwZ)
        {
          allocatorGW->Add (Vector (gwX, gwY, gwZ));
          nDev++;
        }
      in_File.close ();
    }

  gatewaysContainer.Create (nDev);
  mobilityGW.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityGW.SetPositionAllocator (allocatorGW);
  mobilityGW.Install (gatewaysContainer);
  return nDev;
}

/**********************
 *  Global Callbacks  *
 **********************/

void
TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("Transmitted a packet from device " << systemId);
  // Create a packetStatus
  sent++;
}

void
PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // Remove the successfully received packet from the list of sent ones
  // NS_LOG_INFO ("A packet was successfully received at gateway " << systemId);
  received++;
}

void
InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because of interference at gateway " << systemId);
  interfered++;
}

void
NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because there were no more receivers at gateway " << systemId);
  noMoreReceivers++;
}

void
UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_INFO ("A packet arrived at the gateway under sensitivity at gateway " << systemId);
  underSensitivity++;
}

time_t oldtime = std::time (0);

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
PrintEndDevicesPositions (std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  for (NodeContainer::Iterator j = endDevicesContainer.Begin (); j != endDevicesContainer.End ();
       ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      //      int sf = int (mac->GetDataRate ());
      Vector pos = position->GetPosition ();
      spreadingFactorFile << pos.x << " " << pos.y << " " << pos.z << std::endl;
    }
  spreadingFactorFile.close ();
}

void
PrintEndDevicesParameters (std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  for (NodeContainer::Iterator gw = gatewaysContainer.Begin (); gw != gatewaysContainer.End ();
       ++gw)
    {
      Ptr<Node> oGateway = *gw;
      for (NodeContainer::Iterator ed = endDevicesContainer.Begin ();
           ed != endDevicesContainer.End (); ++ed)
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
          spreadingFactorFile << oDevice->GetId () << " " << distanceFromGW << " " << sf << " "
                              << txPower << std::endl;
        }
    }
  spreadingFactorFile.close ();
}

void
PrintPacketRate ()
{
  double receivedRate = double (received) / (sent * nGateways);
  double interferedRate = double (interfered) / (sent * nGateways);
  double noMoreReceiversRate = double (noMoreReceivers) / (sent * nGateways);
  double underSensitivityRate = double (underSensitivity) / (sent * nGateways);

  double sumpacks = receivedRate + interferedRate + noMoreReceiversRate + underSensitivityRate;
  double receivedAboveSensitivityRate =
      double (received) / ((sent * nGateways) - underSensitivity);
  double interferedAboveSensitivityRate =
      double (interfered) / ((sent * nGateways) - underSensitivity);
  double noMoreReceiversAboveSensitivityRate =
      double (noMoreReceivers) / (sent * nGateways - underSensitivity);
  std::cout << std::fixed << std::setprecision (2);
  std::cout << "Received: " << received << " Sent: " << sent << " #DEVICES: " << nDevices
            << " #Gateways: " << nGateways << " Sum: " << sumpacks
            << " Sent/Period: " << double (sent) / (simulationTime * 24)
            << "\nRECEIVED %: " << receivedRate << " INTERFERRED %: " << interferedRate
            << " NO MORE RECEIVED %: " << noMoreReceiversRate
            << " UNDER SENSIT %: " << underSensitivityRate
            << "\nReceived ABOVE SENS: " << receivedAboveSensitivityRate
            << " INTERFERRED ABOVE: " << interferedAboveSensitivityRate
            << " NO MORE RECEIVERS ABOVE: " << noMoreReceiversAboveSensitivityRate << std::endl;
}

int
main (int argc, char *argv[])
{

  // Create the time value from the period
  Time timeStart = Simulator::Now ();
  /***********
  *  Files    *
  ***********/

  std::string exp_dir = "4x4x2";
  std::string sim_dir = "/home/rogerio/git/sim-res/datafile/" + exp_dir;
  std::string devicesInputFilename = sim_dir + "/in/endDevicesPositions.dat";
  std::string gatewaysInputFilename = sim_dir + "/in/gatewaysCartesianPositions.dat";
  std::string sftp_inputFilename = sim_dir + "/in/opt_sf_tp.dat";
  std::string plrI_OutputFilename = sim_dir + "/out/plrI.dat";
  std::string skl_OutputFilename = sim_dir + "/out/skl.dat";
  std::string devs_OutputFilename = sim_dir + "/out/devicesSF_TP.dat";
  std::string ss_filename = sim_dir + "/out/devicesPositions.dat";
  std::string par_filename = sim_dir + "/out/devicesParameters.dat";

  /***********
  *  Seed    *
  ***********/

  //  ns3::RngSeedManager::SetSeed (1971);

  /*****************************
  *  Setup via command prompt  *
  ******************************/

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("printEDs", "Whether or not to print a file containing the ED's positions",
                printEDs);
  cmd.AddValue ("okumura", "Uses okumura-hate propagation mode", okumura);

  cmd.Parse (argc, argv);

  // Set up logging
  if (verbose)
    {
      LogComponentEnable ("ThesisExpEqCartesian", ns3::LOG_LEVEL_ALL);
      //      LogComponentEnable ("LoraChannel", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LoraPhy", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("EndDeviceLoraPhy", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("GatewayLoraPhy", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LoraInterferenceHelper", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LorawanMac", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("EndDeviceLorawanMac", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("GatewayLorawanMac", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LogicalLoraChannelHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("LogicalLoraChannel", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LoraHelper", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LoraPhyHelper", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LorawanMacHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("PeriodicSenderHelper", ns3::LOG_LEVEL_DEBUG);
      LogComponentEnable ("PeriodicSender", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LorawanMacHeader", ns3::LOG_LEVEL_DEBUG);
      //      LogComponentEnable ("LoraFrameHeader", ns3::LOG_LEVEL_DEBUG);
    }
  /****************
  *  Setup Network*
  *****************/

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

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

  nDevices = EndDevicesPlacement (devicesInputFilename);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevicesContainer);

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevicesContainer.Begin (); j != endDevicesContainer.End ();
       ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      phy->TraceConnectWithoutContext ("StartSending", MakeCallback (&TransmissionCallback));
    }

  /*********************
  *  Create Gateways  *
  *********************/

  nGateways = GatewaysPlacement (gatewaysInputFilename);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gatewaysContainer);

  /************************
  *  Configure Gateways  *
  ************************/

  // Install reception paths on gateways
  for (NodeContainer::Iterator j = gatewaysContainer.Begin (); j != gatewaysContainer.End (); j++)
    {

      Ptr<Node> object = *j;
      // Get the device
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != 0);
      Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();

      // Global callbacks (every gateway)
      gwPhy->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&PacketReceptionCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                         MakeCallback (&InterferenceCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                         MakeCallback (&NoMoreReceiversCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                         MakeCallback (&UnderSensitivityCallback));
    }

  /**********************************************
  *  Set up the end device's spreading factor  *
  **********************************************/

  //  macHelper.SetSpreadingFactorsUp (endDevicesContainer, gatewaysContainer, channel);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/

  Time appStopTime = Seconds (simulationTime * 24); // one day
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds)); // each minute
  appHelper.SetPacketSize (41);
  ApplicationContainer appContainer = appHelper.Install (endDevicesContainer);

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
  nsHelper.SetEndDevices (endDevicesContainer);
  nsHelper.SetGateways (gatewaysContainer);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gatewaysContainer);

  /**********************
   * Print output files *
   *********************/
  if (printEDs)
    {
      PrintEndDevicesPositions (ss_filename);
      PrintEndDevicesParameters (par_filename);
    }

  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (appStopTime + Hours (1));

  Simulator::Run ();

  LoraPacketTracker &tracker = helper.GetPacketTracker ();
  std::cout << "Sent packets | Receiver at least one gateway" << std::endl;
  std::cout << tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1)) << std::endl;

  for (NodeContainer::Iterator j = gatewaysContainer.Begin (); j != gatewaysContainer.End (); j++)
    {

      Ptr<Node> object = *j;
      // Get the device
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      std::cout << "totPacketsSent receivedPackets interferedPackets noMoreGwPackets "
                   "underSensitivityPackets lostBecauseTxPackets"
                << std::endl;
      std::cout << tracker.PrintPhyPacketsPerGw (Seconds (0), appStopTime + Hours (1),
                                                 object->GetId ())
                << std::endl;
    }

  Simulator::Destroy ();

  if (printRate)
    PrintPacketRate ();

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

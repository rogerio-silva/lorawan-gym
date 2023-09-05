/*
 * This program creates a simple network which uses an ADR algorithm to set up
 * the Spreading Factors of the devices in the Network.
 *
 * Author: Rogério Silva <rogeriosousa@discente.ufg.br>
 */

#include "ns3/point-to-point-module.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/lora-channel.h"
#include "ns3/mobility-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/lora-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/periodic-sender.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/config.h"
#include "ns3/rectangle.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/lorawan-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("AdrExample-ROG");

// Trace sources that are called when a node changes its DR or TX power
void
OnDataRateChange (uint8_t oldDr, uint8_t newDr)
{
  NS_LOG_DEBUG ("DR" << unsigned (oldDr) << " -> DR" << unsigned (newDr));
}
void
OnTxPowerChange (double oldTxPower, double newTxPower)
{
  NS_LOG_DEBUG (oldTxPower << " dBm -> " << newTxPower << " dBm");
}

int
main (int argc, char *argv[])
{
  bool verbose = true;
  bool initializeSF = true;
  bool adrEnabled = true;
  //To compute the number of devices
  int devDistance = 50;
  int maxDEVxGW_Distance = 5000; //max distance from gateway to device placement
  int minDEVxGW_Distance = 100; //max distance from gateway to device placement

  int nPeriods = 20; //Times to try communication establishment

  double maxRandomLoss = 10;
  std::string adrType = "ns3::NoAdrComponent";

  CommandLine cmd;
  cmd.AddValue ("verbose", "Whether to print output or not", verbose);
  cmd.AddValue ("MultipleGwCombiningMethod", "ns3::NoAdrComponent::MultipleGwCombiningMethod");
  cmd.AddValue ("MultiplePacketsCombiningMethod",
                "ns3::NoAdrComponent::MultiplePacketsCombiningMethod");
  cmd.AddValue ("HistoryRange", "ns3::NoAdrComponent::HistoryRange");
  cmd.AddValue ("MType", "ns3::EndDeviceLorawanMac::MType");
  cmd.AddValue ("EDDRAdaptation", "ns3::EndDeviceLorawanMac::EnableEDDataRateAdaptation");
  cmd.AddValue ("ChangeTransmissionPower", "ns3::NoAdrComponent::ChangeTransmissionPower");
  cmd.AddValue ("devDistance", "Distance between devices", devDistance);
  cmd.AddValue ("minDistance", "Shorter distance between gateway and device", minDEVxGW_Distance);
  cmd.AddValue ("maxDistance", "Greater distance between gateway and device", maxDEVxGW_Distance);
  cmd.AddValue ("PeriodsToSimulate", "Number of periods to simulate", nPeriods);
  cmd.AddValue ("maxRandomLoss", "Maximum amount in dB of the random loss component",
                maxRandomLoss);
  cmd.AddValue ("initializeSF", "Whether to initialize the SFs", initializeSF);
  cmd.AddValue ("MaxTransmissions", "ns3::EndDeviceLorawanMac::MaxTransmissions");
  cmd.Parse (argc, argv);

  int nGateways = 1;
  int nDevices = std::floor ((maxDEVxGW_Distance - minDEVxGW_Distance) / devDistance) + 1;
  // Logging
  //////////

  LogComponentEnable ("AdrExample-ROG", LOG_LEVEL_ALL);
  //  LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_ALL);
  // LogComponentEnable ("NetworkServer", LOG_LEVEL_ALL);
  // LogComponentEnable ("NetworkController", LOG_LEVEL_ALL);
  LogComponentEnable ("NetworkScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable ("NetworkStatus", LOG_LEVEL_ALL);
  //  LogComponentEnable ("EndDeviceStatus", LOG_LEVEL_ALL);
  LogComponentEnable ("NoAdrComponent", LOG_LEVEL_ALL);
  //  LogComponentEnable ("Forwarder", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("MacCommand", LOG_LEVEL_ALL);
  // LogComponentEnable ("AdrExploraSf", LOG_LEVEL_ALL);
  // LogComponentEnable ("AdrExploraAt", LOG_LEVEL_ALL);
  //  LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);

  // Set the EDs to require Data Rate control from the NS
  Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (adrEnabled));

  // Create a simple wireless channel
  // /////////////////////////////////

  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);

  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (0.0));
  x->SetAttribute ("Max", DoubleValue (maxRandomLoss));

  Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> ();
  randomLoss->SetAttribute ("Variable", PointerValue (x));

  loss->SetNext (randomLoss);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  // Helpers
  //////////

  // End Device mobility
  MobilityHelper mobilityGw;

  // Gateway placement
  Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator> ();
  gwAllocator->Add (Vector (0.0, 0.0, 30.0));
  mobilityGw.SetPositionAllocator (gwAllocator);
  mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();

  ////////////////
  // Create GWs //
  ////////////////

  NodeContainer gateways;
  gateways.Create (nGateways);
  mobilityGw.Install (gateways);

  // Create the LoraNetDevices of the gateways
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  // Create EDs
  /////////////

  MobilityHelper mobilityEd;
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // End devices placement
  Ptr<ListPositionAllocator> edAllocator = CreateObject<ListPositionAllocator> ();
  for (int i = minDEVxGW_Distance; i <= maxDEVxGW_Distance; i += devDistance)
    {
      edAllocator->Add (Vector (-i, 0.0, 1.5));
    }

  mobilityEd.SetPositionAllocator (edAllocator);
  mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
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

  // Install applications in EDs
  int appPeriodSeconds = 1200; // One packet every 20 minutes
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  ApplicationContainer appContainer = appHelper.Install (endDevices);
  // Do not set spreading factors up: we will wait for the NS to do this
  if (initializeSF)
    {
      macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

    }

  ////////////
  // Create NS
  ////////////

    NodeContainer networkServers;
    networkServers.Create (1);

    // Install the NetworkServer application on the network server
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGateways (gateways);
    networkServerHelper.SetEndDevices (endDevices);
    networkServerHelper.EnableAdr (adrEnabled);
    networkServerHelper.SetAdr (adrType);
    networkServerHelper.Install (networkServers);

    // Install the Forwarder application on the gateways
    ForwarderHelper forwarderHelper;
    forwarderHelper.Install (gateways);

  // Connect our traces
  Config::ConnectWithoutContext (
      "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
      MakeCallback (&OnTxPowerChange));
  Config::ConnectWithoutContext (
      "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
      MakeCallback (&OnDataRateChange));

  // Activate printing of ED MAC parameters
  Time stateSamplePeriod = Seconds (1200);
  helper.EnablePeriodicDeviceStatusPrinting (endDevices, gateways, "nodeData.txt",
                                             stateSamplePeriod);
  helper.EnablePeriodicPhyPerformancePrinting (gateways, "phyPerformance.txt", stateSamplePeriod);
  helper.EnablePeriodicGlobalPerformancePrinting ("globalPerformance.txt", stateSamplePeriod);

  LoraPacketTracker &tracker = helper.GetPacketTracker ();

  // Start simulation
  Time simulationTime = Seconds (1200 * nPeriods);
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Computing performance metrics...");

  std::cout << tracker.CountMacPacketsGlobally (Seconds (0),
                                                Seconds (1200 * nPeriods))
            << std::endl;

  return 0;
}

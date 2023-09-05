/*
* This program creates a simple network which uses an ADR algorithm to set up
* the Spreading Factors of the devices in the Network.
 *
 * Trial #1: Set DR on each execution
 * script runLoraSlice.sh
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
#include "ns3/lora-tx-current-model.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Example1");

// Trace sources that are called when a node changes its DR or TX power
void
OnDataRateChange (uint8_t oldDr, uint8_t newDr)
{
//  NS_LOG_DEBUG ("DR" << unsigned (oldDr) << " -> DR" << unsigned (newDr));
  std::cout << "DR" << unsigned (oldDr) << " -> DR" << unsigned (newDr) << std::endl;
}
void
OnTxPowerChange (double oldTxPower, double newTxPower)
{
//  NS_LOG_DEBUG (oldTxPower << " dBm -> " << newTxPower << " dBm");
  std::cout << oldTxPower << " dBm -> " << newTxPower << " dBm"<< std::endl;
}

int
main (int argc, char *argv[])
{
  int dR = 0;
  CommandLine cmd;
  cmd.AddValue ("dR", "The Data Rate to simulate", dR);
  cmd.Parse (argc, argv);

//  bool verbose = true;
  bool adrEnabled = false;
//  bool initializeSF = false;
  int nDevices = 500;
  int nGateways = 1;
  int nPeriods = 10;
//  double mobileNodeProbability = 0;
//  double sideLength = 7500;
  double maxRandomLoss = 2;

  std::string adrType = "ns3::AdrComponent";
  // Set the EDs to require Data Rate control from the NS
  Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));
  Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (dR));

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

  MobilityHelper mobilityEd, mobilityGw;

  Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator> ();
  for (int i = 100; i < 7501; i+=100)
    {
      allocatorED->Add (Vector (i, 0, 0));
      mobilityEd.SetPositionAllocator (allocatorED);
    }
  mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator> ();
  allocatorGW->Add (Vector (0, 0, 0));
  allocatorGW->Add (Vector (7500, 0, 0));
  mobilityGw.SetPositionAllocator (allocatorGW);
  mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();

  NodeContainer gateways;
  gateways.Create (nGateways);
  mobilityGw.Install (gateways);

  // Create the LoraNetDevices of the gateways
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  NodeContainer endDevices;
  endDevices.Create (nDevices);

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

  int appPeriodSeconds = 600; // One packet every 10 minutes
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

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
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
                                 MakeCallback (&OnTxPowerChange));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
                                 MakeCallback (&OnDataRateChange));

  Time stateSamplePeriod = Seconds (60);
  std::string pathToFiles = "packetsPerDR/";
  std::string node = pathToFiles + "nodeData";
  node += std::to_string(dR) + ".dat";
  helper.EnablePeriodicDeviceStatusPrinting (endDevices, gateways, node, stateSamplePeriod);
  std::string phy = pathToFiles + "phyPerformance";
  phy += std::to_string(dR) + ".dat";
  helper.EnablePeriodicPhyPerformancePrinting (gateways, phy, stateSamplePeriod);
  std::string global = pathToFiles + "globalPerformance";
  global += std::to_string(dR) + ".dat";
  helper.EnablePeriodicGlobalPerformancePrinting (global, stateSamplePeriod);

  LoraPacketTracker& tracker = helper.GetPacketTracker ();

  // Start simulation
  Time simulationTime = Seconds (60 * nPeriods);
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Computing performance metrics...");

  std::cout << tracker.PrintPhyPacketsPerGw (Seconds (0), simulationTime, gateways.Get (0)->GetId())
            << std::endl;
// To use: change nGateways to 2
  //  std::cout << tracker.PrintPhyPacketsPerGw (Seconds (0), simulationTime, gateways.Get (1)->GetId())
//            << std::endl;
  std::cout <<  tracker.CountMacPacketsGlobally(Seconds (0),
                                                simulationTime )<< std::endl;


  return 0;

}
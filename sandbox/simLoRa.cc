#include "ns3/core-module.h"
#include "ns3/command-line.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("LoRaSimulation");
/****************
*  Tracing  *
****************/

int
main (int argc, char* argv[])
{

  /****************
  *  Setup        *
  ****************/

  std::string adrType = "ns3::AdrComponent";

  CommandLine cmd;
  bool verbose = false;
  bool adrEnabled = true;
  int nDevices = 1;
  int nGateways = 1;
  std::string metric = "throughput";

  //  bool initializeSF = false;
  //  int sf[6] = {7, 8, 9, 10, 11, 12};

  cmd.AddValue ("verb", "Whether to print output or not", verbose);
  cmd.AddValue ("adr", "Whether to enable ADR or not", adrEnabled);
  cmd.AddValue ("nD", "Number of end devices", nDevices);
  cmd.AddValue ("nG", "Number of gateways", nGateways);
  cmd.AddValue ("metric", "Metric [throughput, packetLoss]", metric);
  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("LoRaSimulation", ns3::LOG_LEVEL_ALL);
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
    } else {
      LogComponentEnable ("LoRaSimulation", ns3::LOG_LEVEL_ERROR);
    }

  /************************
  * Presettings:          *
  *  Channel and Mobility *
  ************************/
  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  // Create the mobility env
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  allocator->Add (Vector (1000,0,0));
  allocator->Add (Vector (0,1000,0));
  allocator->Add (Vector (-1000,0,0));
  allocator->Add (Vector (0,-1000,0));
  allocator->Add (Vector (0,0,0));
  mobility.SetPositionAllocator (allocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Create the Lora & Phy & Mac Helpers
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);
  LorawanMacHelper macHelper = LorawanMacHelper ();
  LoraHelper helper = LoraHelper ();

  /****************
  *  Topology  *
  ****************/
  /*  End Node  */
  NodeContainer ed;
  ed.Create (1);
  mobility.Install (ed);
  /* Set the device type to the PHY and MAC layers to the  End Device */
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, ed);

  /*  Gateway Node  */
  NodeContainer gw;
  gw.Create (1);
  mobility.Install (gw);
  /* Set the device type to the PHY and MAC layers to the Gateway */
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gw);

  /**************************
   *  Network Server  *
   ***************************/
  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();
  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  // Create the NS node
  NodeContainer ns;
  ns.Create (1);

  mobility.Install (ns);

  // Create a NS to the network
  nsHelper.SetEndDevices (ed);
  nsHelper.SetGateways (gw);
  nsHelper.Install (ns);
  // Create a forwarder to the gateway
  forHelper.Install (gw);

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  Time appStopTime = Seconds (60);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (60));
  appHelper.SetPacketSize (23);
  Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
      "Min", DoubleValue (0), "Max", DoubleValue (10));
  ApplicationContainer appContainer = appHelper.Install (ed);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  AnimationInterface anim ("lora-animation.xml"); // Mandatory
  for (uint32_t i = 0; i < ed.GetN (); ++i)
    {
      anim.UpdateNodeDescription (ed.Get (i), "ED"); // Optional
      anim.UpdateNodeColor (ed.Get (i), 255, 0, 0); // Optional
      anim.UpdateNodeSize (ed.Get (i) -> GetId (), 20, 20);
    }
  for (uint32_t i = 0; i < gw.GetN (); ++i)
    {
      anim.UpdateNodeDescription (gw.Get (i), "GW"); // Optional
      anim.UpdateNodeColor (gw.Get (i), 0, 255, 0); // Optional
      anim.UpdateNodeSize (gw.Get (i) -> GetId (), 20, 20);
    }
  anim.EnablePacketMetadata (); // Optional
  anim.EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
  anim.EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional

 /****************
  *  Simulation  *
  ****************/


  Simulator::Stop (Minutes(10));


  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
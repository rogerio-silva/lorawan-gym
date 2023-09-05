/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/nr-module.h"
#include "ns3/antenna-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("simGlobecom-log");

AnimationInterface * pAnim = 0;

struct rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct rgb colors [] = {
    { 255, 0, 0 }, // Red
    { 0, 255, 0 }, // Blue
    { 0, 0, 255 }  // Green
};

void modify ()
{
  static uint32_t nId = 0;
  if(nId==4)
    nId=0;
  // Every update change the node description for node 2
  std::ostringstream node0Oss;
  node0Oss << "-----Node:" << Simulator::Now ().GetSeconds ();
  pAnim->UpdateNodeDescription (nId, node0Oss.str ());

  // Every update change the color for node 4
  static uint32_t index = 0;

  index++;
  if (index == 3)
    index = 0;
  struct rgb color = colors[index];
  for (uint32_t nodeId = 4; nodeId < 8; ++nodeId)
    pAnim->UpdateNodeColor (nId++, color.r, color.g, color.b);

  if (Simulator::Now ().GetSeconds () < 10) // This is important or the simulation
    // will run endlessly
    Simulator::Schedule (Seconds (1), modify);
}

int main(int argc, char *argv[]){

    Time::SetResolution (Time::NS);

    // Network topology
    /*
     * (((O)))
     *   /X\       5G       X  X
     *   /BS\<------------->[DR]
     *  /XXX\               X  X
     */

    // Scenario parameters (that we will use inside this script):
    // gNB - Base Station 5G, UAVs - Mobile Gateways, endD - LoRa Sensors
    uint16_t gNbNum = 4;
//    uint16_t uavNum = 1;
//    uint16_t endDNum = 1;
    uint16_t uavNumPergNb = 2;

    // General parameters
    bool logging = false;

    // 5G Traffic parameters (that we will use inside this script):
    uint32_t udpPacketSizeBe = 1252;
    uint32_t lambdaBe = 10000;

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    uint16_t numerologyBwp = 4;
    double centralFrequencyBand = 3.5e9;
    double bandwidthBand = 100e6;
    double totalBandwidth = bandwidthBand;
    double totalTxPower = 4;

    double x = pow (10, totalTxPower / 10);

    // Simulation parameters. Please don't use double to indicate seconds; use
    // ns-3 Time values which use integers to avoid portability issues.
    Time simTime = MilliSeconds (1000);
    Time udpAppStartTime = MilliSeconds (400);

    // Where we will store the output files.
    std::string simTag = "sim-1";
    std::string outputDir = "/home/ns3/ns-3-dev/sim-res/";
    std::string animFile = "NR2UAV-animation.xml";

    if (logging)
    {
        LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable ("LtePdcp", LOG_LEVEL_INFO);
    }

    /*
      * Default values for the simulation. We are progressively removing all
      * the instances of SetDefault, but we need it for legacy code (LTE)
    */
    Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));

    /*
    * Create the scenario. In our examples, we heavily use helpers that setup
    * the gnbs and ue following a pre-defined pattern. Please have a look at the
    * GridScenarioHelper documentation to see how the nodes will be distributed.
    */
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows (gNbNum);
    gridScenario.SetColumns (gNbNum);
    gridScenario.SetHorizontalBsDistance (5.0);
    gridScenario.SetVerticalBsDistance (5.0);
    gridScenario.SetBsHeight (3.5);
    gridScenario.SetUtHeight (1.0);
    // must be set before BS number
    gridScenario.SetSectorization (GridScenarioHelper::TRIPLE);
    gridScenario.SetBsNumber (gNbNum);
    gridScenario.SetUtNumber (uavNumPergNb * gNbNum);
    gridScenario.SetScenarioHeight (4); // Create a 3x3 scenario where the UE will
    gridScenario.SetScenarioLength (4); // be distribuited.
    randomStream += gridScenario.AssignStreams (randomStream);
    gridScenario.CreateScenario ();

    NodeContainer uavNodeContainer;
    // to increase the number of uavs
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals ().GetN (); ++j)
    {
        Ptr<Node> uav = gridScenario.GetUserTerminals ().Get (j);
        uavNodeContainer.Add (uav);
    }

    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - EpcHelper, which will setup the core network
     * - IdealBeamformingHelper, which takes care of the beamforming part
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
    */
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

    // Put the pointers inside nrHelper
    nrHelper->SetBeamformingHelper (idealBeamformingHelper);
    nrHelper->SetEpcHelper (epcHelper);

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;  // in this example, both bands have a single CC

    // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
    // a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf (centralFrequencyBand, bandwidthBand, numCcPerBand, BandwidthPartInfo::RMa_LoS);

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc (bandConf);

    /*
      * Attributes of ThreeGppChannelModel still cannot be set in our way.
    */
    Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod",TimeValue (MilliSeconds (0)));
    nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
    nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));

    nrHelper->InitializeOperationBand (&band);
    allBwps = CcBwpCreator::GetAllBwps ({band});

    Packet::EnableChecking ();
    Packet::EnablePrinting ();

    /*
     *  Case (i): Attributes valid for all the nodes
    */
    // Beamforming method
    idealBeamformingHelper->SetAttribute ("BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));

    // Core latency
    epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
    nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (4));
    nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
    nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (8));
    nrHelper->SetGnbAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

    uint32_t bwpId = 0;

    // gNb routing between Bearer and bandwidth part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute ("GBR_CONV_VOICE", UintegerValue (bwpId));

    // Ue routing between Bearer and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_CONV_VOICE", UintegerValue (bwpId));

    // install and get the pointers to the NetDevices, which contains all the NR stack:
    NetDeviceContainer eNBNetDev = nrHelper->InstallGnbDevice (gridScenario.GetBaseStations (), allBwps);
    NetDeviceContainer uavNetDev = nrHelper->InstallUeDevice (uavNodeContainer, allBwps);

    randomStream += nrHelper->AssignStreams (eNBNetDev, randomStream);
    randomStream += nrHelper->AssignStreams (uavNetDev, randomStream);

    // Get the first netdevice (enbNetDev.Get (0)) and the first bandwidth part (0)
    // and set the attribute.
    nrHelper->GetGnbPhy (eNBNetDev.Get (0), 0)->SetAttribute ("Numerology", UintegerValue (numerologyBwp));
    nrHelper->GetGnbPhy (eNBNetDev.Get (0), 0)->SetAttribute ("TxPower", DoubleValue (10 * log10 ((bandwidthBand / totalBandwidth) * x)));

    // When all the configuration is done, explicitly call UpdateConfig ()

    for (auto it = eNBNetDev.Begin (); it != eNBNetDev.End (); ++it)
    {
        DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
    }

    for (auto it = uavNetDev.Begin (); it != uavNetDev.End (); ++it)
    {
        DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }
    // From here, it is standard NS3. In the future, we will create helpers
    // for this part as well.

    // create the internet and install the IP stack on the UAVs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = epcHelper->GetPgwNode ();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
    internet.Install (gridScenario.GetUserTerminals ());

    Ipv4InterfaceContainer uavIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (uavNetDev));

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals ().GetN (); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (gridScenario.GetUserTerminals ().Get (j)->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    // attach UAVs to the closest eNB
    nrHelper->AttachToClosestEnb (uavNetDev, eNBNetDev);

    /*
   * Traffic part. Install traffic: type conversational voice,
   * identified by a particular source port.
   */
    uint16_t dlPortUav = 1234;

    ApplicationContainer serverApps;

    // The sink will always listen to the specified ports
    UdpServerHelper dlPacketSinkVoice (dlPortUav);

    // The server, that is the application which is listening, is installed in the UE
    serverApps.Add (dlPacketSinkVoice.Install (uavNodeContainer));

    // Voice configuration and object creation:
    UdpClientHelper dlClientVoice;
    dlClientVoice.SetAttribute ("RemotePort", UintegerValue (dlPortUav));
    dlClientVoice.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
    dlClientVoice.SetAttribute ("PacketSize", UintegerValue (udpPacketSizeBe));
    dlClientVoice.SetAttribute ("Interval", TimeValue (Seconds (1.0 / lambdaBe)));

    // The bearer that will carry voice traffic
    EpsBearer voiceBearer (EpsBearer::GBR_CONV_VOICE);

    // The filter for the voice traffic
    Ptr<EpcTft> voiceTft = Create<EpcTft> ();
    EpcTft::PacketFilter dlpfVoice;
    dlpfVoice.localPortStart = dlPortUav;
    dlpfVoice.localPortEnd = dlPortUav;
    voiceTft->Add (dlpfVoice);

    /*
   * Let's install the applications!
   */
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < uavNodeContainer.GetN (); ++i)
    {
        Ptr<Node> uav = uavNodeContainer.Get (i);
        Ptr<NetDevice> uavDevice = uavNetDev.Get (i);
        Address uavAddress = uavIpIface.GetAddress (i);

        // The client, who is transmitting, is installed in the remote host,
        // with destination address set to the address of the UE
        dlClientVoice.SetAttribute ("RemoteAddress", AddressValue (uavAddress));
        clientApps.Add (dlClientVoice.Install (remoteHost));

        // Activate a dedicated bearer for the traffic type
        nrHelper->ActivateDedicatedEpsBearer (uavDevice, voiceBearer, voiceTft);
    }

    // start UDP server and client apps
    serverApps.Start (udpAppStartTime);
    clientApps.Start (udpAppStartTime);
    serverApps.Stop (simTime);
    clientApps.Stop (simTime);

    // enable the traces provided by the nr module
    //nrHelper->EnableTraces();

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add (remoteHost);
    endpointNodes.Add (gridScenario.GetUserTerminals ());

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install (endpointNodes);
    monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
    monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
    monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));

    pAnim = new AnimationInterface(animFile);
    Simulator::Schedule (Seconds (1), modify);

    Simulator::Stop (simTime);
    Simulator::Run ();

    // Print per-flow statistics
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile;
    std::string filename = outputDir + simTag;
//    std::string filename = simTag;

    outFile.open (filename.c_str (), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open ())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf (std::ios_base::fixed);

    double flowDuration = (simTime - udpAppStartTime).GetSeconds ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t) t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str ("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str ("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> " << t.destinationAddress << ":" << t.destinationPort << ") proto " << protoStream.str () << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0  << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            averageFlowThroughput += i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000  << " Mbps\n";
            outFile << "  Mean delay:  " << 1000 * i->second.delaySum.GetSeconds () / i->second.rxPackets << " ms\n";
//            outFile << "  Mean upt:  " << i->second. / i->second.rxPackets / 1000/1000 << " Mbps \n";
            outFile << "  Mean jitter:  " << 1000 * i->second.jitterSum.GetSeconds () / i->second.rxPackets  << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    outFile << "\n\n  Mean flow throughput: " << averageFlowThroughput / stats.size () << "\n";
    outFile << "  Mean flow delay: " << averageFlowDelay / stats.size () << "\n";

    outFile.close ();

    std::ifstream f (filename.c_str ());

    if (f.is_open ())
    {
        std::cout << f.rdbuf ();
    }

    Simulator::Destroy ();
    return 0;
}
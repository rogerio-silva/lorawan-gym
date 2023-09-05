#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/applications-module.h"
#include "ns3/animation-interface.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ESTUDOS");

void CourseChange (std::string context, Ptr <const MobilityModel> model){
  Vector position = model->GetPosition ();
//  NS_LOG_UNCOND (context << " x = " << position.x << ", y = " << position.y);
  std::basic_string<char> node = context.substr (11,1)=="/" ? context.substr (10,1):context.substr (10,2);
  NS_LOG_UNCOND (position.x << "," << position.y << "," << node);
};

int
main (int argc, char **argv)
{
  Time::SetResolution (Time::NS);

  CommandLine cmd;
  uint32_t nCsma = 3;
  uint32_t nWifi = 8;
  bool verbose = false;
  bool tracing = false;
  bool animation = false;
  cmd.AddValue ("nC", "[--nC=??] Número de nós CSMA", nCsma);
  cmd.AddValue ("nW", "[--nW=??] Número de nós Wifi", nWifi);
  cmd.AddValue ("v", "[--v] Mostrar echo das aplicações", verbose);
  cmd.AddValue ("t", "[--t] Rastrear as aplicações", tracing);
  cmd.AddValue ("a", "[--a] Animação", animation);
  cmd.Parse (argc, argv);

  uint32_t nP2P = 2;

  if (nWifi > 18)
    {
      std::cout << "nWifi de dispositivos deve ser 18 ou menos" << std::endl;
    }

  if (verbose)
    {
      LogComponentEnable ("ESTUDOS", ns3::LOG_LEVEL_ALL);
      LogComponentEnable ("UdpEchoClientApplication", ns3::LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", ns3::LOG_LEVEL_INFO);
    }

  NodeContainer nodesP2P;
  nodesP2P.Create (nP2P);

  NodeContainer nodesCSMA;
  nodesCSMA.Add (nodesP2P.Get (1));
  nodesCSMA.Create (nCsma);

  NodeContainer nodesWIFI;
  nodesWIFI.Create (nWifi);
  NodeContainer nodeWIFI_AP = nodesP2P.Get (0);

  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer devicesP2P;
  devicesP2P = p2p.Install (nodesP2P);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

  NetDeviceContainer devicesCSMA;
  devicesCSMA = csma.Install (nodesCSMA);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy;
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  //  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  WifiMacHelper mac;
  Ssid ssid = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));

  NetDeviceContainer devicesWIFI;
  devicesWIFI = wifi.Install (phy, mac, nodesWIFI);
  NetDeviceContainer devicesAP;
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  devicesAP = wifi.Install (phy, mac, nodeWIFI_AP);

  MobilityHelper mobility;

  //  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
  //                                 "MinX", DoubleValue (0.0),
  //                                 "MinY", DoubleValue (0.0),
  //                                 "DeltaX", DoubleValue (5.0),
  //                                 "DeltaY", DoubleValue (10.0),
  //                                 "GridWidth", UintegerValue (3),
  //                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", DoubleValue (20.0), "Y",
                                 DoubleValue (20.0), "Rho",
                                 StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"));

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Time", StringValue("1s"),
                             "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=1.8]"),
                             "Direction", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=20.0]"),
                             "Bounds", StringValue("0|50|0|50"));

//                             RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobility.Install (nodesWIFI);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodeWIFI_AP);
  mobility.Install (nodesCSMA);

  InternetStackHelper stack;
  stack.Install (nodesCSMA);
  stack.Install (nodeWIFI_AP);
  stack.Install (nodesWIFI);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfacesP2P;
  interfacesP2P = address.Assign (devicesP2P);
  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer interfacesCSMA;
  interfacesCSMA = address.Assign (devicesCSMA);
  address.SetBase ("10.1.3.0", "255.255.255.0");
  address.Assign (devicesWIFI);
  address.Assign (devicesAP);

  UdpEchoServerHelper echoServer (9000);
  ApplicationContainer serverApp = echoServer.Install (nodesCSMA.Get (nCsma));
  serverApp.Start (Seconds (1.0));
  serverApp.Stop (Seconds (10.0));

  UdpEchoClientHelper echoClient (interfacesCSMA.GetAddress (nCsma), 9000);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApp = echoClient.Install (nodesWIFI.Get (nWifi - 1));
  clientApp.Start (Seconds (2.0));
  clientApp.Stop (Seconds (10.0));

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  Simulator::Stop (Seconds (10.0));
  if (tracing)
    {
      p2p.EnablePcapAll ("est-3");
      phy.EnablePcap ("est-3", devicesAP.Get (0));
      csma.EnablePcap ("est-3", devicesCSMA.Get (1), true);
    }

  std::cout << animation << " : " <<  tracing << std::endl;

  if (animation)
    {
      AnimationInterface anim ("est3.xml");
      anim.EnablePacketMetadata (true);
    }

  std::ostringstream  oss;
  oss << "/NodeList/*/$ns3::MobilityModel/CourseChange";
  Config::Connect (oss.str(), MakeCallback (&CourseChange));

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
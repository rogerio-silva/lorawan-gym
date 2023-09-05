#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("EVENTS");

// Simple ns3 simulation with two node point to point network
int main (int argc, char** argv)  {
  CommandLine cmd;
  cmd.Parse (argc, argv);

  LogComponentEnable ("EVENTS", ns3::LOG_DEBUG);

  NodeContainer nodesCSMA;
  nodesCSMA.Create (8 );

  NS_LOG_LOGIC ("Node Creation");

  MobilityHelper mob;
  mob.SetPositionAllocator ("ns3::GridPositionAllocator",
     "MinX", DoubleValue (1.0),
     "MinY", DoubleValue (1.0),
     "DeltaX", DoubleValue (2.0),
     "DeltaY", DoubleValue (2.0),
     "GridWidth", UintegerValue (2),
     "LayoutType", StringValue ("RowFirst"));
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  NS_LOG_LOGIC ("Constant Mobility model Creation");

  mob.Install (nodesCSMA);
  mob.AssignStreams (nodesCSMA, 0);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate (5000000)));
  csma.SetChannelAttribute ("Delay", StringValue ("2ms"));
  csma.SetDeviceAttribute ("Mtu", UintegerValue(1500));

  NS_LOG_LOGIC ("Point-to-Point (Device and Channel attributes) Creation");


  NetDeviceContainer devices = csma.Install (nodesCSMA);
  InternetStackHelper stack;
  stack.Install (nodesCSMA);

  NS_LOG_LOGIC ("Net Device (IP Stack nodesCSMA install) Creation");

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  NS_LOG_LOGIC ("IPv4 Creation");

  Ipv4InterfaceContainer interfaces = address.Assign (devices);
  NS_LOG_LOGIC ("Adding IPv4 to devices interfaces");

  UdpEchoServerHelper echoServer (9000);
  ApplicationContainer serverApps = echoServer.Install (nodesCSMA.Get (0));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (60.0));
  NS_LOG_LOGIC ("Adding server Apps");

  UdpEchoClientHelper echoClient (interfaces.GetAddress (0), 9000);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (100));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.5)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (256));
  NS_LOG_LOGIC ("Adding client 1 Apps");

  UdpEchoClientHelper echoClient2 (interfaces.GetAddress (0), 9000);
  echoClient2.SetAttribute ("MaxPackets", UintegerValue (200));
  echoClient2.SetAttribute ("Interval", TimeValue (Seconds (0.75)));
  echoClient2.SetAttribute ("PacketSize", UintegerValue (256));
  NS_LOG_LOGIC ("Adding client 2 Apps");

  UdpEchoClientHelper echoClient3 (interfaces.GetAddress (0), 9000);
  echoClient2.SetAttribute ("MaxPackets", UintegerValue (200));
  echoClient3.SetAttribute ("Interval", TimeValue (Seconds (0.5)));
  echoClient3.SetAttribute ("PacketSize", UintegerValue (256));
  NS_LOG_LOGIC ("Adding client 3 Apps");

  ApplicationContainer clientApps = echoClient.Install (nodesCSMA.Get (1));
  clientApps.Start (Seconds (1.0));
  clientApps.Stop (Seconds (60.0));

  ApplicationContainer clientApps2 = echoClient2.Install (nodesCSMA.Get (2));
  clientApps2.Start (Seconds (10.0));
  clientApps2.Stop (Seconds (60.0));

  ApplicationContainer clientApps3 = echoClient3.Install (nodesCSMA.Get (3));
  clientApps3.Start (Seconds (30.0));
  clientApps3   .Stop (Seconds (60.0));

  AnimationInterface anim("eventsCSMA.xml");
  anim.EnablePacketMetadata (true);
  anim.SetMaxPktsPerTraceFile (50000);

  AsciiTraceHelper ascii;
  csma.EnableAsciiAll (ascii.CreateFileStream ("csma-events.tr"));

  Simulator::Run ();
  Simulator::Destroy ();
}
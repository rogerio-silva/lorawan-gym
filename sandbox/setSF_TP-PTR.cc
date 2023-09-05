#include "ns3/core-module.h"
#include "ns3/command-line.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("SetSF_TP");

static uint32_t g_received = 0;
static uint32_t g_sended = 0;
static u_int32_t g_errorNoDemodul = 0;
static u_int32_t g_errorInterf = 0;
static u_int32_t g_errorSens = 0;
static u_int32_t g_errorSF = 0;
static u_int32_t g_errorWrongFreq = 0;
static u_int32_t g_errorNoReceivers = 0;
static u_int32_t g_errorOccupied = 0;
static u_int32_t g_errorGWInterf = 0;

uint8_t
SFToDR (uint8_t sf)
{
  return (12 - sf);
}

uint8_t
TP (uint8_t sf)
{
  uint8_t tp;
  switch (sf)
    {
    case 7: tp=2; break;
    case 8: tp=5; break;
    case 9: tp=8; break;
    case 10: tp=10; break;
    default: tp=14;
    }
  return tp;
}

double
Sensitivity (uint8_t sf)
{
  // Uplink sensitivity (Source: SX1301 datasheet)
  // {SF7, SF8, SF9, SF10, SF11, SF12}
  const double sensitivity[6] = {-130.0, -132.5, -135.0,
                                 -137.5, -140.0, -142.5}; // {SF7, SF8, SF9, SF10, SF11, SF12}

  return sensitivity[sf - 7];
}

/******************
 * CALLBACK FUNCTIONS
 */

void
OnTransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_sended++;
}

void
OnPacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_received++;
}

void
OnPacketErrorSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorSens++;
}

void
OnPacketErrorWrongSFCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorSF++;
}
void
OnPacketErrorInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorInterf++;
}
void
OnPacketErrorNoDemodulatorCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorNoDemodul++;
}
void
OnPacketErrorFrequencyCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorWrongFreq++;
}

void
OnPacketErrorNoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorNoReceivers++;
}
void
OnPacketErrorGWInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  g_errorGWInterf++;
}
void
OnPacketErrorOccupiedReceptionCallback (int oldValue, int newValue)
{
  g_errorOccupied++;
}

/******************
 * MAIN
 * @param argc
 * @param argv
 * @return
 */

/* TODO desenvolver um método para variar o SF e TP (como o NoAdr-component),
 * porém construído como uma classe que possa ser instânciada a qualquer instante
 * (como lora-utils)
*/

void
myMain (uint8_t sf, uint8_t tp, Vector vPos, bool printToFile, std::ofstream& outputFile)
{
  //  LogComponentEnable ("SetSF_TP", LOG_LEVEL_ALL);
  //  LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
  //  LogComponentEnable ("SimpleEndDeviceLoraPhy", LOG_LEVEL_ALL);
  //  LogComponentEnable ("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);

  int distance = 0;

  NS_LOG_INFO ("Creating the channelURA...");
  Ptr<LoraChannel> channelURA, channelRA, channelBE;
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 10.0);
  // Create the correlated shadowing component
  Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
      CreateObject<CorrelatedShadowingPropagationLossModel> ();
  // Add the effect to the channelURA propagation loss
  Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();
  shadowing->SetNext (buildingLoss);
  channelURA = CreateObject<LoraChannel> (loss, delay);
  channelRA = CreateObject<LoraChannel> (loss, delay);
  channelBE = CreateObject<LoraChannel> (loss, delay);

  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channelURA);
  LorawanMacHelper macHelper = LorawanMacHelper ();
  macHelper.SetRegion (ns3::lorawan::LorawanMacHelper::EU);
  LoraHelper loraHelper = LoraHelper ();
  loraHelper.EnablePacketTracking ();

  NS_LOG_INFO ("Creating the end device...");
  NodeContainer endDeviceContainer;
  endDeviceContainer.Create (1);
  MobilityHelper mobilityED;
  Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator> ();
  allocatorED->Add (vPos);
  mobilityED.SetPositionAllocator (allocatorED);
  mobilityED.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityED.Install (endDeviceContainer);

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  loraHelper.Install (phyHelper, macHelper, endDeviceContainer);
  Ptr<LoraPhy> phyED;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  for (NodeContainer::Iterator j = endDeviceContainer.Begin (); j != endDeviceContainer.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyED = loraNetDevice->GetPhy ();
      phyED->TraceConnectWithoutContext ("StartSending", MakeCallback (OnTransmissionCallback));
      macED = loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
      macED->SetDataRate (SFToDR (sf));
      macED->SetTransmissionPower (tp);
    }

  NS_LOG_INFO ("Creating the gatewayContainer...");

  NodeContainer gatewayContainer;
  gatewayContainer.Create (1);
  MobilityHelper mobilityGW;
  Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator> ();
  allocatorGW->Add (Vector (0, 0, 0));
  mobilityGW.SetPositionAllocator (allocatorGW);
  mobilityGW.Install (gatewayContainer);

  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  //  macHelper.SetSpreadingFactorsUp (endDeviceContainer, gatewayContainer, channelURA);

  loraHelper.Install (phyHelper, macHelper, gatewayContainer);
  Ptr<LoraPhy> phyGW;

  for (NodeContainer::Iterator j = gatewayContainer.Begin (); j != gatewayContainer.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyGW = loraNetDevice->GetPhy ();
      phyGW->TraceConnectWithoutContext ("StartSending", MakeCallback (OnTransmissionCallback));
      phyGW->TraceConnectWithoutContext ("ReceivedPacket",
                                         MakeCallback (OnPacketReceptionCallback));
      phyGW->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                         MakeCallback (OnPacketErrorNoMoreReceiversCallback));
      phyGW->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                         MakeCallback (OnPacketErrorGWInterferenceCallback));
      phyGW->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                         MakeCallback (OnPacketErrorSensitivityCallback));
    }
  phyGW->SetChannel(channelURA);

  distance = phyED->GetMobility ()->GetDistanceFrom (phyGW->GetMobility ());
  double txPowerDbm = macED->GetTransmissionPower ();
  double rxPowerDbm =
      channelURA->GetRxPower (txPowerDbm, phyED->GetMobility (), phyGW->GetMobility ());
  NS_LOG_INFO ("Creating the Sender application...");
  PeriodicSenderHelper sender = PeriodicSenderHelper ();
  sender.SetPeriod (Seconds (300));
  sender.SetPacketSize (20);
  ApplicationContainer appContainer = sender.Install (endDeviceContainer);
  appContainer.Start (Seconds (10));
  appContainer.Stop (Seconds (7200));

  loraHelper.EnablePeriodicGlobalPerformancePrinting ("globData_SetSFTP.txt", Seconds (300));

  NS_LOG_INFO ("Simulation...");
  Simulator::Stop (Seconds (7200));

  Simulator::Run ();

  NS_LOG_INFO ("Results...");
  if (g_received>0)
    {
      if (printToFile)
        {
          outputFile << (int) sf << "," << (int) tp << "," << distance << ","
                     << !(g_sended - g_received) << std::endl;
        }
      else
        {
          std::list<double> freqs = phyGW->GetObject<GatewayLoraPhy> ()->GetFrequencies ();
          std::cout << "Frequencies: ";
          for (std::list<double>::iterator it = freqs.begin (); it != freqs.end (); ++it)
            {
              std::cout << " " << *it << ",";
            }
          std::cout << std::endl;
          std::cout << "sf: " << (int) sf << " Distance: " << distance
                    << " Transmitted power: " << txPowerDbm << " Received power: " << rxPowerDbm
                    << " Sensitivity: " << Sensitivity (sf)
                    << " Link margin: " << rxPowerDbm - Sensitivity (sf) << std::endl;
          std::cout << " Sent " << g_sended << std::endl
                    << " Received: " << g_received << std::endl
                    << " Errors Wrong Frequency: " << g_errorWrongFreq << std::endl
                    << " GWInterference Error: " << g_errorGWInterf << std::endl
                    << " Interference Error: " << g_errorInterf << std::endl
                    << " NoDemodulated Error: " << g_errorNoDemodul << std::endl
                    << " sf Error: " << g_errorSF << std::endl
                    << " Sensitivity Error: " << g_errorSens << std::endl
                    << " NoReceivers Error: " << g_errorNoReceivers << std::endl;
        }
    }
  Simulator::Destroy ();
}


int
main (int argc, char *argv[])
{

  bool printToFile = false;

  CommandLine cmd;
  cmd.AddValue ("printToFile", "Whether to print output or not", printToFile);
  cmd.Parse (argc, argv);

  for (uint8_t sf = 7; sf <= 12; sf++)
    {
      std::string fileName = "/home/ns3/ns-3-dev/sim-res/SFxDistance_" + std::to_string (sf) + ".dat";
      const char *c = fileName.c_str ();
      std::ofstream outputFile;
      outputFile.open (c, std::ofstream::out | std::ofstream::trunc);
//      for (uint8_t tp = 2; tp <= 14; tp += 2)
//        {
          for (int posX = 50; posX <= 10000; posX += 50)
            {
              g_received = 0;
              g_sended = 0;
              g_errorNoDemodul = 0;
              g_errorInterf = 0;
              g_errorSens = 0;
              g_errorSF = 0;
              g_errorWrongFreq = 0;
              g_errorNoReceivers = 0;
              g_errorOccupied = 0;
              g_errorGWInterf = 0;
              Vector vPos (posX, 0, 1.5);

              myMain (sf, TP (sf), vPos, printToFile, outputFile);
              if (g_received<=0) break;
            }
          outputFile << std::endl;
//        }
      outputFile.close ();
    }
  return 0;
}

#include "ns3/core-module.h"
#include "ns3/command-line.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("SlicedLoRaComm");

// Global vars
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

enum distributionMode { CARTESIAN, DENSE, OPTIMIZED };

// Model Vars
NodeContainer endDevicesContainer;
NodeContainer gatewaysContainer;
Ptr<LoraChannel> channel;

// Global Settings parameters
int lossModel = 0; //0-LogDistance 1-Okumura-Hata
int nGateways = 1;
int nDevices = 1;
int sideLength = 1;
int operationMode = 0;
Time appStopTime = Seconds (300);

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
 * CONFIGURATION FUNCTIONS
 */

uint8_t
SFToDR (uint8_t sf)
{
  return (12 - sf);
}

double
Sensitivity (uint8_t sf)
{
  // Uplink sensitivity (Source: SX1301 datasheet)
  // {SF7, SF8, SF9, SF10, SF11, SF12}
  const double sensitivity[6] = {-130.0, -132.5, -135.0,
                                 -137.5, -140.0, -142.5}; // {SF7, SF8, SF9, SF10, SF11, SF12}
  //  const double sensitivity[6] = {-123.0, -126.0, -129.0,
  //                                 -132.0, -134.0, -137.0}; // {SF7, SF8, SF9, SF10, SF11, SF12}

  return sensitivity[sf - 7];
}

Ptr<LoraChannel>
ChannelSettings ()
{
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  if (lossModel == 0)
    {
      Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel> ();
      channel = CreateObject<LoraChannel> (loss, delay);
    }
  else
    {
      Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
      loss->SetPathLossExponent (3.76);
      loss->SetReference (1, 7.7);
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
          CreateObject<CorrelatedShadowingPropagationLossModel> ();
      // Add the effect to the channel propagation loss
      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();
      shadowing->SetNext (buildingLoss);
      channel = CreateObject<LoraChannel> (loss, delay);
    }
  return channel;
}

Ptr<ListPositionAllocator>
CartesianPlacement (NodeContainer gateways)
{
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  //  int nGateways = gateways.GetN();
  Ptr<LoraPhy> phyED;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  for (NodeContainer::Iterator gw = gateways.Begin (); gw != gateways.End (); ++gw)
    {
      Ptr<Node> nodeGW = *gw;
      Ptr<LoraNetDevice> loraGWDevice = nodeGW->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phyGW = loraGWDevice->GetPhy ();
      for (NodeContainer::Iterator ed = endDevicesContainer.Begin ();
           ed != endDevicesContainer.End (); ++ed)
        {
          Ptr<Node> node = *ed;
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
          phyED = loraNetDevice->GetPhy ();
          phyED->TraceConnectWithoutContext ("StartSending", MakeCallback (OnTransmissionCallback));
          macED = loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
          macED->SetDataRate (SFToDR (12));
          macED->SetTransmissionPower (14);
          //          double txPowerDbm = macED->GetTransmissionPower ();
          //          double rxPowerDbm =
          //              channel->GetRxPower (txPowerDbm, phyED->GetMobility (), phyGW->GetMobility ());
        }
    }
  return allocator;
}
Ptr<ListPositionAllocator>
DensePlacement (NodeContainer gateways)
{
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  Ptr<LoraPhy> phyED;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  for (NodeContainer::Iterator j = endDevicesContainer.Begin (); j != endDevicesContainer.End ();
       ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyED = loraNetDevice->GetPhy ();
      phyED->TraceConnectWithoutContext ("StartSending", MakeCallback (OnTransmissionCallback));
      macED = loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
      macED->SetDataRate (SFToDR (12));
      macED->SetTransmissionPower (14);
    }

  return allocator;
}
Ptr<ListPositionAllocator>
OptimalPlacement (NodeContainer gateways)
{
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  Ptr<LoraPhy> phyED;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  for (NodeContainer::Iterator j = endDevicesContainer.Begin (); j != endDevicesContainer.End ();
       ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyED = loraNetDevice->GetPhy ();
      phyED->TraceConnectWithoutContext ("StartSending", MakeCallback (OnTransmissionCallback));
      macED = loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
      macED->SetDataRate (SFToDR (12));
      macED->SetTransmissionPower (14);
    }

  return allocator;
}

void
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
}

void
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
}

void
GatewayPlacementScenerys ()
{
  NodeContainer gateways;
  bool needGateway = true;
  Ptr<ListPositionAllocator> allocator;
  allocator = CreateObject<ListPositionAllocator> ();
  MobilityHelper mobilityGW;
  while (needGateway) //Verificar cobertura
    {
      //      bool noNeedMoreGateway = false;
      NodeContainer gateway;
      gateway.Create (1);
      //      phyHelper.SetDeviceType (LoraPhyHelper::GW);
      //      macHelper.SetDeviceType (LorawanMacHelper::GW);
      //      gateways.Add (gateway);
      //      helper.Install (phyHelper, macHelper, gateways);
      //      switch (operationMode)
      //        {
      //        case 0: // CARTESIAN
      //          allocator = CartesianPlacement (gateways, channel);
      //          mobilityGW.SetPositionAllocator (allocator);
      //          break;
      //        case 1:
      //          allocator = DensePlacement (gateways, channel);
      //          mobilityGW.SetPositionAllocator (allocator);
      //          break;
      //        case 2:
      //          allocator = OptimalPlacement (gateways, channel);
      //          mobilityGW.SetPositionAllocator (allocator);
      //          break;
      //        default:
      //          NS_LOG_INFO ("Wrong distribution mode");
      //        }
      needGateway = false;
    }
  mobilityGW.Install (gatewaysContainer);
}

void
PrintDevicesData (std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream devicesFile;
  devicesFile.open (c);
  devicesFile << "X, Y, Z" << std::endl;
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
      int sf = int (mac->GetDataRate ());
      Vector pos = position->GetPosition ();
      devicesFile << pos.x << " " << pos.y << " " << pos.z << " " << sf << std::endl;
    }
  devicesFile.close ();
}

void
SetSlices ()
{
  double qt[3], pt[3] = {0,0,0};
  uint32_t nDevs = endDevicesContainer.GetN ();
  qt[0] = ceil (0.2 * (double) nDevs);
  qt[1] = floor (0.3 * (double) nDevs);
  qt[2] = floor (0.5 * (double) nDevs);
  for (NodeContainer::Iterator e = endDevicesContainer.Begin (); e != endDevicesContainer.End ();
       ++e)
    {
      Ptr<Node> nodeED = *e;
      Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice (0)->GetObject<LoraNetDevice> ();
      if (pt[0] < qt[0])
        {
          loraEndDevice->SetSlice (0);
          pt[0] += 1;
        }
      else if (pt[1] < qt[1])
        {
          loraEndDevice->SetSlice (1);
          pt[1] += 1;
        }
      else
        {
          loraEndDevice->SetSlice (2);
          pt[2] += 1;
        }
    }
}

void
GetFeasibleSFTP (std::string filename)
{
  NS_LOG_INFO ("Setting SF and TP...");
  std::ofstream devicesFile;
  devicesFile.open (filename.c_str ());
  Ptr<LoraPhy> phyED;
  Ptr<LoraPhy> phyGW;
  Ptr<LorawanMac> macGW;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  double txPowerDbm = 2.0, rxPowerDbm, linkMargin;
  uint8_t sf = 12;
  int qED = 0;
  for (NodeContainer::Iterator e = endDevicesContainer.Begin (); e != endDevicesContainer.End ();
       ++e)
    {
      Ptr<Node> nodeED = *e;
      Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyED = loraEndDevice->GetPhy ();
      macED = loraEndDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
      std::cout << "Device Nº: " << ++qED << std::endl;
      for (NodeContainer::Iterator g = gatewaysContainer.Begin (); g != gatewaysContainer.End ();
           ++g)
        {
          Ptr<Node> nodeGW = *g;
          Ptr<LoraNetDevice> lorawanGateway = nodeGW->GetDevice (0)->GetObject<LoraNetDevice> ();
          phyGW = lorawanGateway->GetPhy ();
          sf = 13;
          linkMargin = -1;
          while (sf > 6)
            {
              txPowerDbm = 2.0;
              sf -= 1;
              while (txPowerDbm < 15.0)
                {
                  rxPowerDbm = channel->GetRxPower (txPowerDbm, phyED->GetMobility (),
                                                    phyGW->GetMobility ());
                  linkMargin = rxPowerDbm - Sensitivity (sf);
//                  double distance = phyED->GetMobility ()->GetDistanceFrom (phyGW->GetMobility ());
                  if (linkMargin > 0)
                    { //PLR1 == 0 Device E Alcança gateway G com configurações sf e txPowerDbm
                      devicesFile << nodeED->GetId () << "," << nodeGW->GetId () << "," << (int) sf
                                  << "," << txPowerDbm << std::endl;
                    }
                  txPowerDbm += 2.0;
                }
            }
        }
      macED->SetTransmissionPower (txPowerDbm);
      macED->SetDataRate (SFToDR (sf));
    }
  devicesFile.close ();
}

void
SceneryConfig ()
{
  // Channel
  NS_LOG_INFO ("Channel...");
  channel = ChannelSettings ();
  //        rules that are aplied to jammer devices: Duty Cycle and Tx Power, and maximum payload
  //        length, it can be set by editing the ApplyCommonEuConfigurationsJm function in the
  //        lora-mac-helper.cc file:
  //      channelHelper.AddSubBand (868, 868.6, 1, 14); // (firstFrequency, lastFrequency, dutyCycle, maxTxPowerDbm)
  //      channelHelper.AddSubBand (868.7, 869.2, 1, 14); // (firstFrequency, lastFrequency, dutyCycle, maxTxPowerDbm)
  //      channelHelper.AddSubBand (869.4, 869.65, 1, 27); // (firstFrequency, lastFrequency, dutyCycle, maxTxPowerDbm)
  //      Ptr<LoraChannel> channelURA = ChannelSettings (lossModel);
  //      Ptr<LoraChannel> channelRA = ChannelSettings (lossModel);
  //      Ptr<LoraChannel> channelBE = ChannelSettings (lossModel);
  //    }
  //  else
  //    {
  //      Ptr<LoraChannel> channel = ChannelSettings ();
  //    }

  NS_LOG_INFO ("Helpers (PHY, MAC, LoRa...)");
  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);
  // Create the LoraMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();
  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();
  // Setting address generator
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  NS_LOG_INFO ("End devices...");
  //Arquivo de entrada com posicionamento dos devices
  std::string devicesFilename =
      "/home/rogerio/git/sim-res/datafile/4x4x2/endDevicesPositions.dat";
  NS_LOG_INFO ("End Devices placement...");
  EndDevicesPlacement (devicesFilename);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevicesContainer);

  NS_LOG_INFO ("Gateways...");
  /********
   * Gateway Placement
   *    * Distribuição cartesiana equidistante
   *    * Distibuição Cartesiana com analise de densidade
   *    * Distribuição ótima
   */
  NS_LOG_INFO ("Gateway placement...");
  // Grid para posicionamento com cubos de 50x50x10 e 400000 gateways
  // Arquivo de entrada para geração de dados de entrada para solver
  GatewaysPlacement (
      "/home/rogerio/git/sim-res/datafile/4x4x2/gatewaysPositions.dat");
  // Depends on the sliceType
  // MAC and PHY settings
  macHelper.SetRegion (ns3::lorawan::LorawanMacHelper::EU);
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  macHelper.SetAddressGenerator (addrGen);
  helper.EnablePacketTracking ();
  helper.Install (phyHelper, macHelper, gatewaysContainer);


  // Set the optimal parameters sf and tp and the ED slice_type and prints PLR_1
  GetFeasibleSFTP ("/home/rogerio/git/sim-res/datafile/4x4x2/plr1_4x4x2.txt");
  // associate devices to slices
  SetSlices ();
  //  macHelper.SetSpreadingFactorsUp (endDevicesContainer, gatewaysContainer, channel);
}

void
gatewaysPlacement_Equidistant (NodeContainer gws)
{
}

// Configurado para entrada do solver
// Imprime o plr(k,m): device k alcança o gateway m
void
PrintPLR_I (std::string filename)
{
  std::ofstream devicesFile;
  devicesFile.open (filename.c_str ());
  Ptr<LoraPhy> phyED;
  Ptr<LoraPhy> phyGW;
  Ptr<LorawanMac> macGW;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  for (NodeContainer::Iterator e = endDevicesContainer.Begin (); e != endDevicesContainer.End ();
       ++e)
    {
      Ptr<Node> nodeED = *e;
      Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyED = loraEndDevice->GetPhy ();
      macED = loraEndDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
      for (NodeContainer::Iterator g = gatewaysContainer.Begin (); g != gatewaysContainer.End ();
           ++g)
        {
          Ptr<Node> nodeGW = *g;
          Ptr<LoraNetDevice> lorawanGateway = nodeGW->GetDevice (0)->GetObject<LoraNetDevice> ();
          phyGW = lorawanGateway->GetPhy ();
          macGW = lorawanGateway->GetMac ();
          int sf = macGW->GetSfFromDataRate (macED->GetDataRate ());
          double txPowerDbm = macED->GetTransmissionPower ();
          double rxPowerDbm =
              channel->GetRxPower (txPowerDbm, phyED->GetMobility (), phyGW->GetMobility ());
          double linkMargin = rxPowerDbm - Sensitivity (sf);
          //          double distance = phyED->GetMobility ()->GetDistanceFrom (phyGW->GetMobility ());
          if (linkMargin > 0)
            {
              devicesFile << nodeED->GetId () << "," << nodeGW->GetId () << std::endl;
//              devicesFile << nodeED->GetId () << " " << nodeGW->GetId () << " " << distance << " "
//                          << linkMargin << " " << txPowerDbm << " " << sf << std::endl;
            }
        }
    }
  devicesFile.close ();
}

// Configurado para entrada do solver
// Imprime o S(k,l): device k associado ao slice l
void
PrintSKL (std::string filename)
{
  std::ofstream devicesFile;
  devicesFile.open (filename.c_str ());
  Ptr<LoraPhy> phyED;
  Ptr<ClassAEndDeviceLorawanMac> macED;
  for (NodeContainer::Iterator e = endDevicesContainer.Begin (); e != endDevicesContainer.End ();
       ++e)
    {
      Ptr<Node> nodeED = *e;
      Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice (0)->GetObject<LoraNetDevice> ();
      phyED = loraEndDevice->GetPhy ();
      devicesFile << (int) nodeED->GetId () << "," << (int) loraEndDevice->GetSlice () << std::endl;
    }
  devicesFile.close ();
}

/******************
 * MAIN PROGRAM
 */

int
main (int argc, char *argv[])
{

  ns3::RngSeedManager::SetSeed (1971);

  CommandLine cmd;
  cmd.AddValue ("lossModel", "Propagation loss model", lossModel);
  cmd.AddValue ("nDevs", "Number of Devices in the scenery", nDevices);
  cmd.AddValue ("nGateways", "Number of Gateways in the scenery", nGateways);
  cmd.AddValue ("sideLength", "Placement area side length", sideLength);
  cmd.AddValue ("operationMode", "Distribution mode [0-CARTESIAN, 1-DENSE, 2-OPTIMIZED]",
                operationMode);
  cmd.Parse (argc, argv);

  // Set up logging
  LogComponentEnable ("SlicedLoRaComm", ns3::LOG_LEVEL_ALL);
  //    LogComponentEnable ("LoraChannel", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LoraPhy", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("EndDeviceLoraPhy", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("GatewayLoraPhy", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LoraInterferenceHelper", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable("LoraMac", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable("EndDeviceLoraMac", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable("GatewayLoraMac", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LogicalLoraChannelHelper", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LogicalLoraChannel", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LoraHelper", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LoraPhyHelper", ns3::LOG_LEVEL_DEBUG);
  //    //   LogComponentEnable("LoraMacHelper", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("PeriodicSenderHelper", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("PeriodicSender", ns3::LOG_LEVEL_DEBUG);
  //    //   LogComponentEnable("LoraMacHeader", ns3::LOG_LEVEL_DEBUG);
  //    LogComponentEnable ("LoraFrameHeader", ns3::LOG_LEVEL_DEBUG);
  NS_LOG_INFO ("Scenery Settings...");
  SceneryConfig ();

  //  NS_LOG_INFO ("Creating the Sender application...");
  //  OneShotSenderHelper oneShotHelper = OneShotSenderHelper ();
  //  oneShotHelper.SetSendTime (Seconds (4));
  //  oneShotHelper.Install (endDevicesContainer);

  NS_LOG_INFO ("Results...");
  // Resultado para 100 devices e 400000 gateways (50x50x10)
//    PrintPLR_I ("/home/rogerio/git/sim-res/datafile/4x4x2/plr1_4x4x4.txt");
//  PrintSKL ("/home/rogerio/git/sim-res/datafile/4x4x2/skl_4x4x4.txt");
  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (appStopTime + Hours (2));

  Simulator::Run ();

  Simulator::Destroy ();
}

/**
 * Create Sliced LoRaWAN
 * Dawaliby: * Urgency and reliability-aware (URA) slice
 *           * Reliability-aware (RA) slice gives
 *           * Best effort (BE) slice
 *
**/

///**
//   * \brief Get the number of devices on each SliceType stored in this container.
//   *
//   * \returns vector contains the number of devices on each sliceType.
//   */
//std::vector<uint32_t> GetSlicesN (void) const;

//std::vector<uint32_t> NodeContainer::GetSlicesN(void) const
//{
//
//}
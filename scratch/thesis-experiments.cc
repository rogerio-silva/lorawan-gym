/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023 UNIVERSIDADE FEDERAL DE GOIÁS
 * Copyright (c) NumbERS - INSTITUTO FEDERAL DE GOIÁS - CAMPUS INHUMAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Rogério S. Silva <rogerio.sousa@ifg.edu.br>
 */

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("ThesisExperiments");

// Global vars
static uint32_t g_received = 0;
static uint32_t g_sent = 0;
static u_int32_t g_errorInterference = 0;
static u_int32_t g_errorSens = 0;
static u_int32_t g_errorNoReceivers = 0;

// Model Vars
NodeContainer endDevicesContainer;
NodeContainer gatewaysContainer;
Ptr<LoraChannel> channel;

// Global Settings parameters
int lossModel = 0;    // [0-LOG-DISTANCE, 1-OKUMURA-HATA]
bool verbose = false; // Print Log
int nGateways = 0;
int nDevices = 0;
int sideLength = 10000;
bool printPercentuals = false;

/******************
 * CALLBACK FUNCTIONS
 */

enum PacketOutcome
{
    _RECEIVED,
    _INTERFERED,
    _NO_MORE_RECEIVERS,
    _UNDER_SENSITIVITY,
    _UNSET
};

struct myPacketStatus
{
    Ptr<const Packet> packet;
    uint32_t senderId;
    int outcomeNumber;
    std::vector<enum PacketOutcome> outcomes;
};

std::map<Ptr<const Packet>, myPacketStatus> packetTracker;

void
OnTransmissionCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    g_sent++;
}

void
OnPacketReceptionCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    g_received++;
}

void
OnPacketErrorSensitivityCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    g_errorSens++;
}

void
OnPacketErrorInterferenceCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    g_errorInterference++;
}

void
OnPacketErrorNoMoreReceiversCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    g_errorNoReceivers++;
}

/******************
 * CONFIGURATION FUNCTIONS
 */

uint8_t
SFToDR(uint8_t sf)
{
    return (12 - sf);
}

double
Sensitivity(uint8_t sf)
{
    // Uplink sensitivity (Source: SX1301 datasheet)
    // {SF7, SF8, SF9, SF10, SF11, SF12}
    const double sensitivity[6] =
        {-130.0, -132.5, -135.0, -137.5, -140.0, -142.5}; // {SF7, SF8, SF9, SF10, SF11, SF12}
    //  const double sensitivity[6] = {-123.0, -126.0, -129.0,
    //                                 -132.0, -134.0, -137.0}; // {SF7, SF8, SF9, SF10, SF11, SF12}

    return sensitivity[sf - 7];
}

Ptr<LoraChannel>
ChannelSettings()
{
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    if (lossModel == 1) // Okumura-hata
    {
        Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel>();
        channel = CreateObject<LoraChannel>(loss, delay);
    }
    else // Log-distance
    {
        Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
        loss->SetPathLossExponent(3.76);
        loss->SetReference(1, 10.0);
        // Create the correlated shadowing component
        //      Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
        //          CreateObject<CorrelatedShadowingPropagationLossModel> ();
        // Add the effect to the channel propagation loss
        //      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>
        //      (); shadowing->SetNext (buildingLoss);
        channel = CreateObject<LoraChannel>(loss, delay);
    }
    return channel;
}

/**
 * This method positions the devices according to the input file.
 * @param filename: arquivo de entrada
 * @return number of devices
 **/
int
EndDevicesPlacement(std::string filename)
{
    double edX = 0.0;
    double edY = 0.0;
    double edZ = 0.0;
    int nDev = 0;
    Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator>();
    MobilityHelper mobilityED;
    const char* c = filename.c_str();
    // Get Devices position from File
    std::ifstream in_File(c);
    if (!in_File)
    {
        std::cout << "Could not open the file - '" << filename << "'" << std::endl;
    }
    else
    {
        while (in_File >> edX >> edY >> edZ)
        {
            allocatorED->Add(Vector(edX, edY, edZ));
            nDev++;
        }
        in_File.close();
    }

    endDevicesContainer.Create(nDev);
    mobilityED.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityED.SetPositionAllocator(allocatorED);
    mobilityED.Install(endDevicesContainer);
    return nDev;
}

/**
 * This method positions the gateways according to the input file.
 * @param filename: arquivo de entrada
 * @return void
 **/

int
GatewaysPlacement(std::string filename)
{
    double gwX = 0.0;
    double gwY = 0.0;
    double gwZ = 0.0;
    int nDev = 0;
    Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator>();
    MobilityHelper mobilityGW;
    const char* c = filename.c_str();
    // Get Devices position from File
    std::ifstream in_File(c);
    if (!in_File)
    {
        std::cout << "Could not open the file - '" << filename << "'" << std::endl;
    }
    else
    {
        while (in_File >> gwX >> gwY >> gwZ)
        {
            allocatorGW->Add(Vector(gwX, gwY, gwZ));
            nDev++;
        }
        in_File.close();
    }

    gatewaysContainer.Create(nDev);
    mobilityGW.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGW.SetPositionAllocator(allocatorGW);
    mobilityGW.Install(gatewaysContainer);
    return nDev;
}

/**
 * Set slice to end device
 */
void SetSlices()
{
    double qt[3];
    double pt[3] = {0, 0, 0};
    uint32_t nDevs = endDevicesContainer.GetN();
    qt[0] = ceil(0.2 * (double)nDevs);
    qt[1] = floor(0.3 * (double)nDevs);
    qt[2] = floor(0.5 * (double)nDevs);
    for (NodeContainer::Iterator e = endDevicesContainer.Begin(); e != endDevicesContainer.End();
         ++e)
    {
        Ptr<Node> nodeED = *e;
        Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice(0)->GetObject<LoraNetDevice>();
        if (pt[0] < qt[0])
        {
            loraEndDevice->SetSlice(0);
            pt[0] += 1;
        }
        else if (pt[1] < qt[1])
        {
            loraEndDevice->SetSlice(1);
            pt[1] += 1;
        }
        else
        {
            loraEndDevice->SetSlice(2);
            pt[2] += 1;
        }
    }
}

void
GetFeasibleSFTP(std::string filename)
{
    NS_LOG_INFO("Setting SF and TP...");
    uint8_t sf = 12;
    double txPowerDbm = 2.0;
    double rxPowerDbm;
    double linkMargin;
    std::ofstream devicesFile;
    devicesFile.open(filename.c_str());
    //  devicesFile << "device gateway sf tp" << std::endl;
    Ptr<LoraPhy> phyED;
    Ptr<LoraPhy> phyGW;
    Ptr<LorawanMac> macGW;
    Ptr<ClassAEndDeviceLorawanMac> macED;
    for (NodeContainer::Iterator e = endDevicesContainer.Begin(); e != endDevicesContainer.End();
         ++e)
    {
        Ptr<Node> nodeED = *e;
        Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice(0)->GetObject<LoraNetDevice>();
        phyED = loraEndDevice->GetPhy();
        macED = loraEndDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
        for (NodeContainer::Iterator g = gatewaysContainer.Begin(); g != gatewaysContainer.End();
             ++g)
        {
            Ptr<Node> nodeGW = *g;
            Ptr<LoraNetDevice> lorawanGateway = nodeGW->GetDevice(0)->GetObject<LoraNetDevice>();
            phyGW = lorawanGateway->GetPhy();
            sf = 6;
            linkMargin = -1;
            while (sf < 12 && linkMargin < 0)
            {
                txPowerDbm = 2.0;
                sf += 1;
                linkMargin = -1;
                while (txPowerDbm <= 14.0 && linkMargin < 0)
                {
                    rxPowerDbm =
                        channel->GetRxPower(txPowerDbm, phyED->GetMobility(), phyGW->GetMobility());
                    linkMargin = rxPowerDbm - Sensitivity(sf);
                    //                  double distance = phyED->GetMobility ()->GetDistanceFrom
                    //                  (phyGW->GetMobility ());
                    if (linkMargin > 0)
                    { // PLR1 == 0 Device E Alcança gateway G com configurações sf e txPowerDbm
                        devicesFile << nodeED->GetId() << " " << nodeGW->GetId() << " " << (int)sf
                                    << " " << txPowerDbm << std::endl;
                    }
                    txPowerDbm += 2.0;
                }
            }
        }
        macED->SetTransmissionPower(txPowerDbm > 15 ? 14 : txPowerDbm);
        macED->SetDataRate(SFToDR(sf));
    }
    devicesFile.close();
}

/*********************
 * OUTPUT PRINT METHODS
 ***********************/

// Configurado para entrada do solver
// Imprime o S(k,l): device k associado ao slice l
void
PrintSKL(std::string filename)
{
    std::ofstream devicesFile;
    devicesFile.open(filename.c_str());
    //  devicesFile << "device slice" << std::endl;
    Ptr<LoraPhy> phyED;
    Ptr<ClassAEndDeviceLorawanMac> macED;
    for (NodeContainer::Iterator e = endDevicesContainer.Begin(); e != endDevicesContainer.End();
         ++e)
    {
        Ptr<Node> nodeED = *e;
        Ptr<LoraNetDevice> loraEndDevice = nodeED->GetDevice(0)->GetObject<LoraNetDevice>();
        devicesFile << (int)nodeED->GetId() << " " << (int)loraEndDevice->GetSlice() << std::endl;
    }
    devicesFile.close();
}

// Configurado para entrada do solver
// Imprime o S(k,l): device k associado ao slice l
void
PrintDevicesData(std::string filename)
{
    const char* c = filename.c_str();
    std::ofstream devicesFile;
    devicesFile.open(c);
    //  devicesFile << "x y z sf tp" << std::endl;
    for (NodeContainer::Iterator j = endDevicesContainer.Begin(); j != endDevicesContainer.End();
         ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        Ptr<EndDeviceLorawanMac> mac = object->GetDevice(0)
                                           ->GetObject<LoraNetDevice>()
                                           ->GetMac()
                                           ->GetObject<EndDeviceLorawanMac>();
        double tp = mac->GetTransmissionPower();
        int sf = int(mac->GetSfFromDataRate(mac->GetDataRate()));
        Vector pos = position->GetPosition();
        devicesFile << pos.x << " " << pos.y << " " << pos.z << " " << sf << " " << tp << std::endl;
    }
    devicesFile.close();
}

/******************
 * MAIN PROGRAM
 */

int
main(int argc, char* argv[])
{
    int seed = 1;
    int nPlanes = 1;

    CommandLine cmd;
    cmd.AddValue("lossModel", "Propagation loss model [0-LOG-DISTANCE, 1-OKUMURA-HATA]", lossModel);
    cmd.AddValue("verbose", "Print Log if true [--verbose]", verbose);
    cmd.AddValue("nDevices", "Number of Devices in the scenery", nDevices);
    cmd.AddValue("nGateways", "Number of Gateways in the scenery", nGateways);
    cmd.AddValue("sideLength", "Placement area side length", sideLength);
    cmd.AddValue("seed", "Independent replications seed", seed);
    cmd.AddValue("nPlanes", "Number of 3D-Planes in the scenery", nPlanes);

    cmd.Parse(argc, argv);

    ns3::RngSeedManager::SetSeed(seed);

    std::string pathOpt = "/home/rogerio/git/test/" + std::to_string(nGateways) + "x" +
                          std::to_string(nPlanes) + "/";
    //  std::string pathOpt = "/home/rogerio/git/thesis-simulations/optimizer/model/input_data/" +
    //                        std::to_string (nGateways) + "x" + std::to_string (nPlanes) + "/";

    std::string pathDev = "/home/rogerio/git/sim-res/datafile/devices/placement/";
    std::string pathEqu = "/home/rogerio/git/sim-res/datafile/baseline/equidistant/placement/";

    std::string devicesInputFilename = pathDev + "endDevices_LNM_Placement_" +
                                       std::to_string(seed) + "s+" + std::to_string(nDevices) +
                                       "d.dat";
    std::string gatewaysInputFilename =
        pathEqu + "equidistantPlacement_" + std::to_string(nGateways) + ".dat";

    if (nPlanes > 1)
    {
        gatewaysInputFilename = pathEqu + "equidistantPlacement_" + std::to_string(nGateways) +
                                "x" + std::to_string(nPlanes) + ".dat";
    }

    std::string plrI_OutputFilename = pathOpt + "plrI_" + std::to_string(seed) + "s_" +
                                      std::to_string(nGateways) + "x" + std::to_string(nPlanes) +
                                      "Gv_" + std::to_string(nDevices) + "D.dat";
    std::string skl_OutputFilename = pathOpt + "skl_" + std::to_string(seed) + "s_" +
                                     std::to_string(nGateways) + "x" + std::to_string(nPlanes) +
                                     "Gv_" + std::to_string(nDevices) + "D.dat";
    std::string devs_OutputFilename = pathOpt + "devicesSF_TP_" + std::to_string(seed) + "s_" +
                                      std::to_string(nGateways) + "x" + std::to_string(nPlanes) +
                                      "Gv_" + std::to_string(nDevices) + "D.dat";

    if (verbose)
    {
        LogComponentEnable("ThesisExperiments", ns3::LOG_LEVEL_ALL);
        LogComponentEnable("LoraChannel", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LoraPhy", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("EndDeviceLoraPhy", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("GatewayLoraPhy", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LoraInterferenceHelper", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LorawanMac", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("EndDeviceLorawanMac", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("GatewayLorawanMac", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LoraHelper", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LoraPhyHelper", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LorawanMacHelper", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("PeriodicSenderHelper", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("PeriodicSender", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LorawanMacHeader", ns3::LOG_LEVEL_DEBUG);
        LogComponentEnable("LoraFrameHeader", ns3::LOG_LEVEL_DEBUG);
    }

    NS_LOG_INFO("Channel configuration...");
    channel = ChannelSettings();

    NS_LOG_INFO("Helpers (PHY, MAC, LoRa)...");
    // Create the LoraPhyHelper
    LoraPhyHelper phyHelperED = LoraPhyHelper();
    LoraPhyHelper phyHelperGW = LoraPhyHelper();
    phyHelperED.SetChannel(channel);
    phyHelperGW.SetChannel(channel);
    // Create the LoraMacHelper
    LorawanMacHelper macHelperED = LorawanMacHelper();
    LorawanMacHelper macHelperGW = LorawanMacHelper();
    // Create the LoraHelper
    LoraHelper helper = LoraHelper();
    // Setting address generator
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    NS_LOG_INFO("End Devices placement...");
    nDevices = EndDevicesPlacement(devicesInputFilename);
    phyHelperED.SetDeviceType(LoraPhyHelper::ED);
    macHelperED.SetAddressGenerator(addrGen);
    macHelperED.SetDeviceType(LorawanMacHelper::ED_A);
    macHelperED.SetRegion(LorawanMacHelper::EU);
    helper.Install(phyHelperED, macHelperED, endDevicesContainer);

    // Connect trace sources
    for (NodeContainer::Iterator j = endDevicesContainer.Begin(); j != endDevicesContainer.End();
         ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        phy->TraceConnectWithoutContext("StartSending", MakeCallback(&OnTransmissionCallback));
    }

    NS_LOG_INFO("Gateway placement...");
    nGateways = GatewaysPlacement(gatewaysInputFilename);

    // MAC and PHY settings
    phyHelperGW.SetDeviceType(LoraPhyHelper::GW);
    macHelperGW.SetDeviceType(LorawanMacHelper::GW);
    macHelperGW.SetAddressGenerator(addrGen);
    helper.EnablePacketTracking();
    helper.Install(phyHelperGW, macHelperGW, gatewaysContainer);
    NS_LOG_DEBUG("Trace Sources...");
    // Install reception paths on gateways
    for (NodeContainer::Iterator j = gatewaysContainer.Begin(); j != gatewaysContainer.End(); j++)
    {
        Ptr<Node> object = *j;
        // Get the device
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();

        Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy()->GetObject<GatewayLoraPhy>();
        // Global callbacks (every gateway)
        gwPhy->TraceConnectWithoutContext("ReceivedPacket",
                                          MakeCallback(&OnPacketReceptionCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseInterference",
                                          MakeCallback(&OnPacketErrorInterferenceCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseNoMoreReceivers",
                                          MakeCallback(&OnPacketErrorNoMoreReceiversCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseUnderSensitivity",
                                          MakeCallback(&OnPacketErrorSensitivityCallback));
    }

    // Input file for generating input data for solver
    // Set the optimal parameters sf and tp and the ED slice_type
    // and print PLR1
    GetFeasibleSFTP(plrI_OutputFilename);
    SetSlices();
    // Print device x slice association
    PrintSKL(skl_OutputFilename);
    PrintDevicesData(devs_OutputFilename);

    NS_LOG_DEBUG("Network Server configuration...");

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/
    double simulationTime = 10;
    Time appStopTime = Seconds(simulationTime);

    NS_LOG_DEBUG("Completed configuration");

    NS_LOG_INFO("Results...");

    /****************
     *  Simulation  *
     ****************/

    Simulator::Stop(appStopTime);

    Simulator::Run();

    Simulator::Destroy();
}

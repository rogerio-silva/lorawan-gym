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
#include "ns3/end-device-lora-phy.h"
#include "ns3/forwarder-helper.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/opengym-module.h"
#include "ns3/propagation-module.h"
#include "ns3/simulator.h"
#include "ns3/stats-module.h"
#include "ns3/traced-value.h"

#include <iomanip>

// Packet characterization
#define PACKET_SIZE 400
// QoS, Data rate and Delay
#define MAX_RK 6835.94
#define MIN_RK 183.11
#define QOS_BOUND 1.6
// UAVs deployment area
#define H_POSITIONS 10
#define V_POSITIONS 10
// Deployment area
#define AREA_HEIGHT 10000.0
#define AREA_WIDTH 10000.0
#define UAV_ALTITUDE 30.0
#define MOVEMENT_STEP 1000.0

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("LoRaWAN-OpenAIGym");

Ptr<LoraChannel> channel;
MobilityHelper mobilityED;
MobilityHelper mobilityGW;
NodeContainer endDevices;
NodeContainer gateways;

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
    uint32_t receiverId;
    Time sentTime;
    Time receivedTime;
    uint8_t senderSF;
    uint8_t receiverSF;
    double senderTP;
    double receiverTP;
    uint32_t outcomeNumber;
    std::vector<enum PacketOutcome> outcomes;
};

std::map<Ptr<const Packet>, myPacketStatus> packetTracker;

uint32_t nDevices = 0;
uint32_t nGateways = 0;
int noMoreReceivers = 0;
int interfered = 0;
int received = 0;
int underSensitivity = 0;
int sent = 0;

/**********************
 *  Global Callbacks  *
 **********************/
void CheckReceptionByAllGWsComplete(std::map<Ptr<const Packet>, myPacketStatus>::iterator it);
void TransmissionCallback(Ptr<const Packet> packet, uint32_t systemId);
void PacketReceptionCallback(Ptr<const Packet> packet, uint32_t systemId);
void InterferenceCallback(Ptr<const Packet> packet, uint32_t systemId);
void NoMoreReceiversCallback(Ptr<const Packet> packet, uint32_t systemId);
void UnderSensitivityCallback(Ptr<const Packet> packet, uint32_t systemId);
void EndDevicesPlacement(std::string filename);
void PrintData();

int
main(int argc, char* argv[])
{
    // Parameters of the environment
    const int packetSize = 50;
    double simulationTime = 10;
    double envStepTime = 0.1; // seconds, ns3gym env step time interval
    double stopTime = envStepTime + simulationTime;
    uint32_t seed = 1;
    uint32_t simSeed = 1;

    uint32_t openGymPort = 5555;
    uint32_t testArg = 0;

    // Parameters of the scenario
    bool verbose = true;
    bool up = true;

    bool printRates = true;

    double rew = 1.0;

    CommandLine cmd;
    cmd.AddValue("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
    cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
    cmd.AddValue("nGateways", "Number of gateways to include in the simulation", nGateways);
    cmd.AddValue("verbose", "Whether to print output or not", verbose);
    cmd.AddValue("up", "Spread Factor UP", up);
    cmd.AddValue("simSeed", "Seed", simSeed);
    cmd.AddValue("simulationTime", "Simulation time", simulationTime);
    cmd.AddValue("envStepTime", "Simulation Step time", envStepTime);

    cmd.Parse(argc, argv);

    NS_LOG_UNCOND("Ns3Env parameters:");
    NS_LOG_UNCOND("--simulationTime: " << simulationTime);
    NS_LOG_UNCOND("--openGymPort: " << openGymPort);
    NS_LOG_UNCOND("--envStepTime: " << envStepTime);
    NS_LOG_UNCOND("--seed: " << simSeed);
    NS_LOG_UNCOND("--testArg: " << testArg);

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(simSeed);

    Config::SetDefault("ns3::EndDeviceLorawanMac::DRControl", BooleanValue(true));

    /************************************
     *  Logger settings                 *
     ************************************/
    if (verbose)
    {
        //        LogComponentEnable ("EndDeviceStatus", ns3::LOG_LEVEL_ALL);
        LogComponentEnable("ns3::LorawanRl", ns3::LOG_LEVEL_ALL);
        LogComponentEnable("ns3::LorawanGymEnv", ns3::LOG_LEVEL_ALL);
    }

    /************************
     *  Create the channel  *
     ************************/
    // Create the lora channel object
    // modelo de propagação (okumura ou logdistance)
    NS_LOG_INFO("Setting up channel...");
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 10.0);
    channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/
    NS_LOG_INFO("Setting up helpers...");
    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);
    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper();
    // Create the LoraHelper
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    /************************
     *  Create End Devices  *
     ************************/
    NS_LOG_INFO("Creating end devices...");
    // Create a set of nodes
    EndDevicesPlacement("/home/rogerio/git/sim-res/"
                        "datafile/devices/placement/endDevices_LNM_Placement_" +
                        std::to_string(seed) + "s+" + std::to_string(nDevices) + "d.dat");

    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    // Create the LoraNetDevices of the end devices
    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    macHelper.SetRegion(LorawanMacHelper::EU);
    helper.Install(phyHelper, macHelper, endDevices);

    // Connect trace sources
    for (NodeContainer::Iterator j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        phy->TraceConnectWithoutContext("StartSending", MakeCallback(&TransmissionCallback));
    }

    /*********************
     *  Create Gateways  *
     *********************/
    NS_LOG_INFO("Creating gateways...");
    gateways.Create(nGateways);
    // Inicia todos os UAVs
    Ptr<ListPositionAllocator> uavs_positions = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nGateways; i++)
    {
        uavs_positions->Add(Vector(6000, 7000, 30));
    }

    mobilityGW.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGW.SetPositionAllocator(uavs_positions);
    mobilityGW.Install(gateways);

    // Create a net device for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    /************************************
     *  Create the openGym environment  *
     ************************************/
    //    for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); j++)
    //    {
    //        Ptr<Node> object = *j;
    //        Ptr<NetDevice> netDevice = object->GetDevice(0);
    //        Ptr<MobilityModel> gwMob = (*j)->GetObject<MobilityModel>();
    //        Vector position = gwMob->GetPosition();
    //        position = Vector (6500,7000,30);
    //        gwMob->SetPosition(position);
    //    }

    Ptr<OpenGymInterface> openGymInterface = OpenGymInterface::Get(openGymPort);
    Config::SetDefault("ns3::LorawanRl::StepTime",
                       TimeValue(Seconds(envStepTime))); // Time step of env
    Config::SetDefault("ns3::LorawanRl::Duration",
                       TimeValue(Seconds(simulationTime))); // Duration of env sim
    Config::SetDefault("ns3::LorawanRl::Reward", DoubleValue(rew)); // Reward

    // Configura os traces
    for (NodeContainer::Iterator g = gateways.Begin(); g != gateways.End(); ++g)
    {
        Ptr<Node> object = *g;
        // Get the device
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<MobilityModel> mobility = netDevice->GetObject<MobilityModel>();
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy()->GetObject<GatewayLoraPhy>();
        gwPhy->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&PacketReceptionCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseInterference",
                                          MakeCallback(&InterferenceCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseNoMoreReceivers",
                                          MakeCallback(&NoMoreReceiversCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseUnderSensitivity",
                                          MakeCallback(&UnderSensitivityCallback));
        //        std::ostringstream oss;
        //        oss.str("");
        //        // "/NodeList/7/$ns3::MobilityModel/CourseChange"
        //        oss << "/NodeList/" << object->GetId() << "/$ns3::MobilityModel/CourseChange";
        //        std::cout << oss.str() << std::endl;
        //        Config::ConnectWithoutContext(oss.str(), MakeBoundCallback(CollectCommData,
        //        myGymEnv));
    }

    // Force ADR
    ns3::lorawan::LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    /**************************
     *  Create Network Server  *
     ***************************/

    // Create the NS node
    NodeContainer networkServer;
    networkServer.Create(1);
    NetworkServerHelper nsHelper = NetworkServerHelper();
    nsHelper.SetAdr("ns3::AdrComponent");
    nsHelper.EnableAdr(true);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.SetGateways(gateways);
    nsHelper.Install(networkServer);

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/
    // Force send packets
    PeriodicSenderHelper periodicSenderHelper;
    periodicSenderHelper.SetPeriod(Seconds(envStepTime));
    periodicSenderHelper.SetPacketSize(packetSize);
    ApplicationContainer applicationContainer = periodicSenderHelper.Install(endDevices);
    // Create the ForwarderHelper
    ForwarderHelper forHelper = ForwarderHelper();
    forHelper.Install(gateways);

    applicationContainer.Start(Seconds(0.1));
    applicationContainer.Stop(Seconds(simulationTime - envStepTime));

    NS_LOG_DEBUG("Completed configuration");

    /****************
     *  Simulation  *
     ****************/
    NS_LOG_UNCOND("Simulation start");

    Simulator::Stop(Seconds(stopTime));

    NS_LOG_INFO("Computing performance metrics...");

    Simulator::Run();
    NS_LOG_UNCOND("Simulation stop");

    if (printRates)
        PrintData();

    openGymInterface->NotifySimulationEnd();
    Simulator::Destroy();
    NS_LOG_UNCOND("Simulation finished");
}

void
CheckReceptionByAllGWsComplete(std::map<Ptr<const Packet>, myPacketStatus>::iterator it)
{
    // Check whether this packet is received by all gateways
    if ((*it).second.outcomeNumber == nGateways)
    {
        // Update the statistics
        myPacketStatus status = (*it).second;
        for (uint32_t j = 0; j < nGateways; j++)
        {
            switch (status.outcomes.at(j))
            {
            case _RECEIVED: {
                received += 1;
                break;
            }
            case _UNDER_SENSITIVITY: {
                underSensitivity += 1;
                break;
            }
            case _NO_MORE_RECEIVERS: {
                noMoreReceivers += 1;
                break;
            }
            case _INTERFERED: {
                interfered += 1;
                break;
            }
            case _UNSET: {
                break;
            }
            }
        }
        // Remove the packet from the tracker
        //              packetTracker.erase (it);
    }
}

void
TransmissionCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    NS_LOG_INFO("Transmitted a packet from device " << systemId);
    // Coleta informações sobre o pacote (SF, TP, etc.)
    LoraTag tag;
    packet->PeekPacketTag(tag);

    // Create a packetStatus
    myPacketStatus status;
    status.packet = packet;
    status.senderId = systemId;
    status.sentTime = Simulator::Now();
    status.outcomeNumber = 0;
    status.outcomes = std::vector<enum PacketOutcome>(nGateways, _UNSET);
    status.senderSF = tag.GetSpreadingFactor();

    packetTracker.insert(std::pair<Ptr<const Packet>, myPacketStatus>(packet, status));
    sent += 1;
}

void
PacketReceptionCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    // Remove the successfully received packet from the list of sent ones
    NS_LOG_INFO("A packet was successfully received at gateway " << systemId);

    // Coleta informações sobre o pacote (SF, TP, etc.)
    LoraTag tag;
    packet->PeekPacketTag(tag);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it = packetTracker.find(packet);

    if (it != packetTracker.end())
    {
        if ((*it).second.outcomes.size() > systemId - nDevices)
        {
            // lembre que o ID do gateways é enumerado após todos os devices, logo numero sequencial
            //  do GW é contado subtraindo o nDevices. Ainda que os pacotes originados do mesmo
            //  device no mesmo instante é recebido pelos gateways na área de alcance, <e que essas
            //  duplicidades são removidas pela CheckReceptionByAllGWsComplete> sem certeza nisso
            (*it).second.outcomes.at(systemId - nDevices) = _RECEIVED;
            (*it).second.outcomeNumber += 1;
            //      if ((*it).second.outcomeNumber == 1 || (*it).second.receivedTime == Seconds (0))
            if ((*it).second.receivedTime == Seconds(0))
            {
                (*it).second.receivedTime = Simulator::Now();
                (*it).second.receiverId = systemId;
                //          cc++;
            }
            (*it).second.receiverSF = tag.GetSpreadingFactor();
            (*it).second.receiverTP = tag.GetReceivePower();

            CheckReceptionByAllGWsComplete(it);
        }
    }
}

void
InterferenceCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    NS_LOG_INFO("A packet was lost because of interference at gateway " << systemId);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it = packetTracker.find(packet);
    if ((*it).second.outcomes.size() > systemId - nDevices)
    {
        (*it).second.outcomes.at(systemId - nDevices) = _INTERFERED;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

void
NoMoreReceiversCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    NS_LOG_INFO("A packet was lost because there were no more receivers at gateway " << systemId);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it = packetTracker.find(packet);
    if ((*it).second.outcomes.size() > systemId - nDevices)
    {
        (*it).second.outcomes.at(systemId - nDevices) = _NO_MORE_RECEIVERS;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

void
UnderSensitivityCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    NS_LOG_INFO("A packet arrived at the gateway under sensitivity" << systemId);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it = packetTracker.find(packet);
    if ((*it).second.outcomes.size() > systemId - nDevices)
    {
        (*it).second.outcomes.at(systemId - nDevices) = _UNDER_SENSITIVITY;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

/**
 * Places the end devices according to the allocator object in the input file..
 * @param filename: arquivo de entrada
 * @return number of devices
 **/
void
EndDevicesPlacement(std::string filename)
{
    double edX = 0.0;
    double edY = 0.0;
    double edZ = 0.0;
    int nDev = 0;
    Ptr<ListPositionAllocator> allocatorED = CreateObject<ListPositionAllocator>();
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

    endDevices.Create(nDev);
    mobilityED.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityED.SetPositionAllocator(allocatorED);
    mobilityED.Install(endDevices);
}

void
PrintData()
{
    //        uint32_t nDevices = endDevices.GetN();
    // i:: device, j[0] sf j[1] data rate j[2] delay
    std::vector<std::vector<double>> deviceData;
    std::vector<std::vector<std::vector<double>>> deviceSimulatedData;
    double sumQos = 0.0;
    double mean_qos = 0.0;
    NS_LOG_UNCOND("Collecting package data!");

    deviceSimulatedData.reserve(nDevices);
    for (uint32_t i = 0; i < nDevices; ++i)
    {
        deviceSimulatedData.push_back({{}, {}, {}});
    }

    lorawan::LoraTag tag;
    std::cout << std::endl << "Devices Simulated Data" << std::endl;

    int numPackets = 0;
    int _lost = 0;
    int _received = 0;
    for (std::map<Ptr<const Packet>, myPacketStatus>::iterator p = packetTracker.begin();
         p != packetTracker.end();
         ++p)
    {
        numPackets += 1;
        (*p).second.packet->PeekPacketTag(tag);
        if ((*p).second.receiverId == 0)
        {
            _lost++;
        }
        else
        {
            _received++;
        }

        double size = (*p).second.packet->GetSize() * 8; // bits
        int devID = (*p).second.senderId;
        double sf;
        sf = ((*p).second.senderSF);
        double dk;
        dk = ((*p).second.receivedTime.GetSeconds() - (*p).second.sentTime.GetSeconds());
        double rk;
        rk = (size / ((*p).second.receivedTime.GetSeconds() - (*p).second.sentTime.GetSeconds()));
        deviceSimulatedData[devID][0].push_back(sf);
        deviceSimulatedData[devID][1].push_back(rk);
        deviceSimulatedData[devID][2].push_back(dk);
    }

    std::vector<std::vector<double>> deviceSummarizedData;
    sumQos = 0.0;
    for (uint32_t devID = 0; devID < nDevices; ++devID)
    {
        double dk = 0;
        double rk = 0;
        double sf = deviceSimulatedData[devID][0][0];
        double qos = 0;
        int qtd = 0;
        for (unsigned int i = 0; i < deviceSimulatedData[devID][0].size(); ++i)
        {
            if (deviceSimulatedData[devID][2].at(i) > 0.0)
            {
                rk += deviceSimulatedData[devID][1].at(i);
                dk += deviceSimulatedData[devID][2].at(i);
                qtd += 1;
            }
            else
            {
                deviceSimulatedData[devID][0].erase(deviceSimulatedData[devID][0].begin() + i);
                deviceSimulatedData[devID][1].erase(deviceSimulatedData[devID][1].begin() + i);
                deviceSimulatedData[devID][2].erase(deviceSimulatedData[devID][2].begin() + i);
            }
        }
        // QpS do device
        qos = (rk / qtd) / MAX_RK + (1 - ((dk / qtd) / MIN_RK));
        deviceSummarizedData.push_back({sf, rk / qtd, dk / qtd, qos});
        sumQos += qos;
    }
    // QoS médio da execução (todos devices por gateway)
    mean_qos = sumQos / nDevices;

    for (uint32_t i = 0; i < nDevices; ++i)
    {
        std::cout << i << " " << deviceSummarizedData[i][0] << " " << deviceSummarizedData[i][1]
                  << " " << deviceSummarizedData[i][2] << " " << deviceSummarizedData[i][3]
                  << std::endl;
    }
    if (isNaN(mean_qos))
    {
        std::cout << "Não atende os critérios de QoS" << std::endl;
    }
    else
    {
        std::cout << "QoS Simulado: " << mean_qos << std::endl;
    }

    std::cout << "Total: " << numPackets << " Sent: " << sent << " Lost: " << _lost
              << " Received: " << _received << " QoS: " << (isNaN(mean_qos) ? 0.0 : mean_qos)
              << std::endl;
}
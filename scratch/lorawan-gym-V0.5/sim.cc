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
/*
 * #####################################################################################
 *                              SIMULATION MODEL
 * #####################################################################################
 *                 LoRaWAN communication             5G sub6GHz communication
 *    © © © © ©                              X X                              ((( ^
 *     © © © © )))       NON3GPP         ((( |O| )))         3GPP                /X\
 *   ©© © © ©                                X X                                /XXX\
 * Devices Lorawan                           UAV                              BS 5G/B5G
 * #####################################################################################
 * */

#include "ns3/box.h"
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

// QoS, Data rate and Delay
#define MAX_RK 6835.94
#define MIN_RK 183.11


using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("LoRaWAN-OpenAIGym");

Ptr<OpenGymInterface> openGym;
Ptr<MobilityModel> mobility;
uint32_t nDevices = 0;
uint32_t nGateways = 0;
uint32_t env_action = 0;
uint32_t env_action_space_size = 4;
bool env_isGameOver = false;
double m_qos = 0.0;
Box area_bounds = Box(0.0, 10000.0, 0.0, 10000.0, 30.0, 30.0);
double movementStep = 1000.0;
double startXPosition = 0;
double startYPosition = 0;
double startZPosition = 0;
Vector uav_position;
double envStepTime = 600; // seconds, ns3gym env step time interval
bool verbose = false;
const int packetSize = 50;
double applicationStart = 0; // 0.1 second after simulation start
double applicationInterval = 10;
double applicationStop = 600; // 5 minutes
double simulationStop = 600*10*50;
bool impossible_movement = false;

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

NodeContainer endDevices;
NodeContainer gateways;
ApplicationContainer applicationContainer;

int pkt_noMoreReceivers = 0;
int pkt_interfered = 0;
int pkt_received = 0;
int pkt_underSensitivity = 0;
int pkt_transmitted = 0;

/***********************
 * Callback Functions  *
 **********************/
void CheckReceptionByAllGWsComplete(std::map<Ptr<const Packet>, myPacketStatus>::iterator it);
void TransmissionCallback(Ptr<const Packet> packet, uint32_t systemId);
void PacketReceptionCallback(Ptr<const Packet> packet, uint32_t systemId);
void InterferenceCallback(Ptr<const Packet> packet, uint32_t systemId);
void NoMoreReceiversCallback(Ptr<const Packet> packet, uint32_t systemId);
void UnderSensitivityCallback(Ptr<const Packet> packet, uint32_t systemId);
void CourseChangeDetection(std::string context, Ptr<const MobilityModel> model);

/**********************
 * Utility Functions  *
 **********************/
void ScheduleNextStateRead();
void ScheduleNextDataCollect();
void TrackersReset();
Ptr<ListPositionAllocator> NodesPlacement(std::string filename);
void DoSetInitialPositions();
void FindNewPosition(uint32_t action);
double PrintData();

/**********************
 * OPENGYM Functions  *
 **********************/
Ptr<OpenGymSpace> GetObservationSpace();
Ptr<OpenGymSpace> GetActionSpace();
bool ExecuteActions(Ptr<OpenGymDataContainer> action);
Ptr<OpenGymDataContainer> GetObservation();
std::string GetExtraInfo();
float GetReward();
bool GetGameOver();

int
main(int argc, char* argv[])
{
    uint32_t seed = 1;
    uint32_t simSeed = 1;
    double reward = 0.0;
    uint32_t openGymPort = 5555;
    bool up = true;

    CommandLine cmd;
    cmd.AddValue("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
    cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
    cmd.AddValue("nGateways", "Number of gateways to include in the simulation", nGateways);
    cmd.AddValue("verbose", "Whether to print output or not", verbose);
    cmd.AddValue("up", "Spread Factor UP", up);
    cmd.AddValue("simSeed", "Seed", simSeed);
    cmd.AddValue("reward", "Initial Reward", reward);
    cmd.AddValue("step", "UAVs movement step. Default:1000", movementStep);
    cmd.AddValue("startX", "UAVs X start position. Default:0", startXPosition);
    cmd.AddValue("startY", "UAVs Y start position. Default:0", startYPosition);
    cmd.AddValue("startZ", "UAVs Z start position. Default:0", startZPosition);

    cmd.Parse(argc, argv);
    env_action_space_size = 4 * nGateways;

    if(verbose)
    {
        NS_LOG_UNCOND("Ns3Env parameters:");
        NS_LOG_UNCOND("--simulationTime: " << simulationStop);
        NS_LOG_UNCOND("--openGymPort: " << openGymPort);
        NS_LOG_UNCOND("--envStepTime: " << envStepTime);
        NS_LOG_UNCOND("--seed: " << simSeed);
        NS_LOG_UNCOND("--envActionSpaceSize: " << env_action_space_size);
    }
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(simSeed);

    Config::SetDefault("ns3::EndDeviceLorawanMac::DRControl", BooleanValue(true));

    /************************************
     *  Logger settings                 *
     ************************************/
    if (verbose)
    {
        LogComponentEnable("LoRaWAN-OpenAIGym", ns3::LOG_LEVEL_ALL);
    }

    /************************
     *  Create the channel  *
     ************************/
    Ptr<LoraChannel> channel;
    MobilityHelper mobilityED;
    MobilityHelper mobilityGW;
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
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);
    LorawanMacHelper macHelper = LorawanMacHelper();
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    /************************
     *  Create End Devices  *
     ************************/
    NS_LOG_INFO("Creating end devices...");
    std::string cwd = get_current_dir_name();
    // Create a set of nodes
    Ptr<ListPositionAllocator> allocatorED =
        NodesPlacement(cwd + "/data/ed/endDevices_LNM_Placement_" +
                            std::to_string(seed) + "s+" + std::to_string(nDevices) + "d.dat");
    endDevices.Create(nDevices);
    mobilityED.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityED.SetPositionAllocator(allocatorED);
    mobilityED.Install(endDevices);

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

    //     Connect trace sources
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
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
    std::string filename = cwd + "/data/gw/optGPlacement_" +
                           std::to_string (seed) + "s_" + std::to_string (nGateways) + "x1Gv_" +
                           std::to_string (nDevices) + "D.dat";
    Ptr<ListPositionAllocator> gatewaysPositions = NodesPlacement(filename);
    mobilityGW.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGW.SetPositionAllocator(gatewaysPositions);
    mobilityGW.Install(gateways);

    // Create a net device for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    for (auto g = gateways.Begin(); g != gateways.End(); ++g)
    {
        Ptr<Node> object = *g;
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        mobility = netDevice->GetNode()->GetObject<MobilityModel>();
        uav_position = mobility->GetPosition();
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy()->GetObject<GatewayLoraPhy>();
        // Packets tracing callbacks
        gwPhy->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&PacketReceptionCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseInterference",
                                          MakeCallback(&InterferenceCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseNoMoreReceivers",
                                          MakeCallback(&NoMoreReceiversCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseUnderSensitivity",
                                          MakeCallback(&UnderSensitivityCallback));
        // Mobility CourseChange Callback
        std::ostringstream oss;
        oss.str("");
        oss << "/NodeList/" << object->GetId() << "/$ns3::MobilityModel/CourseChange";
        Config::Connect(oss.str(), MakeCallback(&CourseChangeDetection));
        NS_LOG_INFO("CallBack Connected on: " << oss.str());
    }

    /************************************
     *  Create the openGym environment  *
     ************************************/
    openGym = CreateObject<OpenGymInterface>(openGymPort);
    openGym->SetGetActionSpaceCb(MakeCallback(&GetActionSpace));
    openGym->SetGetObservationSpaceCb(MakeCallback(&GetObservationSpace));
    openGym->SetGetGameOverCb(MakeCallback(&GetGameOver));
    openGym->SetGetObservationCb(MakeCallback(&GetObservation));
    openGym->SetGetRewardCb(MakeCallback(&GetReward));
    openGym->SetGetExtraInfoCb(MakeCallback(&GetExtraInfo));
    openGym->SetExecuteActionsCb(MakeCallback(&ExecuteActions));

    // Force ADR
    ns3::lorawan::LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    /**************************
     *  Create Network Server  *'
     ***************************/
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
    Simulator::Schedule(Seconds(applicationStop + 10), &ScheduleNextStateRead);

    ScheduleNextDataCollect();
    NS_LOG_INFO("Completed configuration");

    /****************
     *  Simulation  *
     ****************/
    Simulator::Run();
    NS_LOG_INFO("Computing performance metrics...");
    openGym->NotifySimulationEnd();
    Simulator::Destroy();
    NS_LOG_INFO("Simulation finished");
}

/**
 * Places the end devices according to the allocator object in the input file..
 * @param filename: output filename
 * @return number of devices
 **/
Ptr<ListPositionAllocator>
NodesPlacement(std::string filename)
{
    double edX = 0.0;
    double edY = 0.0;
    double edZ = 0.0;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    const char* c = filename.c_str();
    // Get Devices position from File
    std::ifstream in_File(c);
    if (!in_File)
    {
        NS_LOG_INFO("Could not open the file - '" << filename << "'");
    }
    else
    {
        while (in_File >> edX >> edY >> edZ)
        {
            allocator->Add(Vector(edX, edY, edZ));
        }
        in_File.close();
    }
    return allocator;
}

/**
 * Find the new position of the node.
 * In case of area violation, a penalty is notified to the GYM
 */

void
FindNewPosition(uint32_t action, uint32_t uav_number)
{
    NS_LOG_FUNCTION("FindNewPosition");
    Ptr<MobilityModel> gwMob;
    gwMob = gateways.Get(uav_number)->GetObject<MobilityModel>();
    uav_position = gwMob->GetPosition();

    // Find a new position from the new state obtained from the GYM
    Vector new_pos = uav_position;

    // Check the possibility of executing the movement,
    // initially there will be no movements on the z-axis
    switch (action)
    {
    case 0: // UP
        if (uav_position.y + movementStep > area_bounds.yMax)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(uav_position.x, uav_position.y + movementStep, uav_position.z);
        }
        break;
    case 1: // DOWN
        if (uav_position.y - movementStep < area_bounds.yMin)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(uav_position.x, uav_position.y - movementStep, uav_position.z);
        }
        break;
    case 2: // LEFT
        if (uav_position.x - movementStep < area_bounds.xMin)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(uav_position.x - movementStep, uav_position.y, uav_position.z);
        }
        break;
    case 3: // RIGHT
        if (uav_position.x + movementStep > area_bounds.xMax)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(uav_position.x + movementStep, uav_position.y, uav_position.z);
        }
    }
    if (impossible_movement) // notify penalty to GYM
        NS_LOG_INFO("Movement is not allowed; the node position has not changed!");
    gwMob->SetPosition(new_pos); // the movement
    uav_position = new_pos;
}

double
PrintData()
{
    // Vector composition:
    // i:: device, j[0] sf j[1] data rate j[2] delay
    std::vector<std::vector<double>> deviceData;
    std::vector<std::vector<std::vector<double>>> deviceSimulatedData;
    double sumQos = 0.0;
    double mean_qos = 0.0;
    NS_LOG_INFO("Collecting package data!");

    deviceSimulatedData.reserve(nDevices);
    for (uint32_t i = 0; i < nDevices; ++i)
    {
        deviceSimulatedData.push_back({{}, {}, {}});
    }

    lorawan::LoraTag tag;
    NS_LOG_INFO("Devices Simulated Data...");

    int numPackets = 0;
    int lostPackets = 0;
    int receivedPackets = 0;
    for (auto p = packetTracker.begin();
         p != packetTracker.end();
         ++p)
    {
        numPackets += 1;
        (*p).second.packet->PeekPacketTag(tag);
        if ((*p).second.receiverId == 0)
        {
            lostPackets++;
        }
        else
        {
            receivedPackets++;
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
    if (numPackets == 0)
    {
        return -1;
    }
    std::vector<std::vector<double>> deviceSummarizedData;
    sumQos = 0.0;
    for (uint32_t devID = 0; devID < nDevices; ++devID)
    {
        if(deviceSimulatedData[devID][0].empty()) {
            continue;
        }
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
        // Device QoS
        qos = (rk / qtd) / MAX_RK + (1 - ((dk / qtd) / MIN_RK));
        deviceSummarizedData.push_back({sf, rk / qtd, dk / qtd, qos});
        sumQos += qos;
    }
    // Gateways QoS
    mean_qos = sumQos / nDevices;

    if (isNaN(mean_qos))
    {
        NS_LOG_INFO("Does not meet QoS criteria!");
    }
    else
    {
        for (uint32_t i = 0; i < nDevices; ++i)
        {
            NS_LOG_INFO(i << " " << deviceSummarizedData[i][0] << " " << deviceSummarizedData[i][1]
                          << " " << deviceSummarizedData[i][2] << " " << deviceSummarizedData[i][3]);
        }
        NS_LOG_INFO("Simulated QoS: " << mean_qos);
    }
    NS_LOG_INFO("Total: " << numPackets << " Sent: " << pkt_transmitted << " Lost: " << lostPackets
                          << " Received: " << receivedPackets
                          << " QoS: " << (isNaN(mean_qos) ? 0.0 : mean_qos));
    return mean_qos;
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
                pkt_received += 1;
                break;
            }
            case _UNDER_SENSITIVITY: {
                pkt_underSensitivity += 1;
                break;
            }
            case _NO_MORE_RECEIVERS: {
                pkt_noMoreReceivers += 1;
                break;
            }
            case _INTERFERED: {
                pkt_interfered += 1;
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
    LoraTag tag;
    packet->PeekPacketTag(tag);

    myPacketStatus status;
    status.packet = packet;
    status.senderId = systemId;
    status.sentTime = Simulator::Now();
    status.outcomeNumber = 0;
    status.outcomes = std::vector<enum PacketOutcome>(nGateways, _UNSET);
    status.senderSF = tag.GetSpreadingFactor();

    packetTracker.insert(std::pair<Ptr<const Packet>, myPacketStatus>(packet, status));
    pkt_transmitted += 1;
}

void
PacketReceptionCallback(Ptr<const Packet> packet, uint32_t systemId)
{
    // Remove the successfully received packet from the list of sent ones
    NS_LOG_INFO("A packet was successfully received at gateway " << systemId);
    LoraTag tag;
    packet->PeekPacketTag(tag);

    auto it = packetTracker.find(packet);

    if (it != packetTracker.end())
    {
        if ((*it).second.outcomes.size() > systemId - nDevices)
        {
            (*it).second.outcomes.at(systemId - nDevices) = _RECEIVED;
            (*it).second.outcomeNumber += 1;
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
    auto it = packetTracker.find(packet);
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
    auto it = packetTracker.find(packet);
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
    auto it = packetTracker.find(packet);
    if ((*it).second.outcomes.size() > systemId - nDevices)
    {
        (*it).second.outcomes.at(systemId - nDevices) = _UNDER_SENSITIVITY;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

void
CourseChangeDetection(std::string context, Ptr<const MobilityModel> model)
{
    uav_position = model->GetPosition();
    NS_LOG_INFO(context << " x = " << uav_position.x << ", y = " << uav_position.y);
    // openGym->NotifySimulationEnd();
}

/**
 * OPENGYM ENVIRONMENT FUNCTIONS
 * */

void
ScheduleNextStateRead()
{
    Simulator::Schedule(Seconds(applicationStop + 100), &ScheduleNextStateRead);
    applicationStart = applicationStop + 100;
    applicationStop = applicationStart + envStepTime;
    openGym->NotifyCurrentState();
}

void
ScheduleNextDataCollect()
{
    PeriodicSenderHelper periodicSenderHelper;
    periodicSenderHelper.SetPeriod(Seconds(applicationInterval));
    periodicSenderHelper.SetPacketSize(packetSize);

    ForwarderHelper forHelper = ForwarderHelper();
    forHelper.Install(gateways);
    applicationContainer = periodicSenderHelper.Install(endDevices);
    applicationContainer.Start(Seconds(applicationStart));
    applicationContainer.Stop(Seconds(applicationStop));

    TrackersReset();
}

void
TrackersReset()
{
    packetTracker.clear();
    pkt_transmitted = 0;
    pkt_received = 0;
    pkt_interfered = 0;
    pkt_noMoreReceivers = 0;
    pkt_underSensitivity = 0;
}

Ptr<OpenGymSpace>
GetActionSpace()
{
    /**
     * The action space contains four expected actions: move up,
     * move down, move left, and move right.
     * enum ActionMovements{
     *                      MOVE_UP,
     *                      MOVE_DOWN,
     *                      MOVE_RIGHT,
     *                      MOVE_LEFT
     * }
     * **/
    NS_LOG_FUNCTION("GetActionSpace");
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace>(env_action_space_size);
    NS_LOG_INFO("GetActionSpace: " << space);
    NS_LOG_INFO(Simulator::Now().GetSeconds());
    return space;
}

bool
GetGameOver()
{
    NS_LOG_INFO("MyGetGameOver: " << env_isGameOver);
    return env_isGameOver;
}

float
GetReward()
{
    if (env_action < env_action_space_size)
    {
        m_qos = PrintData();
        m_qos = (impossible_movement) ? -2 : ((isNaN(m_qos) || (m_qos < 0)) ? -1 : m_qos);
    }
    NS_LOG_INFO("MyGetReward: " << m_qos);
    return m_qos;
}

std::string
GetExtraInfo()
{
    std::string env_info = "";
    env_info = "[" + std::to_string(pkt_transmitted) +
               ", " + std::to_string(pkt_received) +
               ", " + std::to_string(applicationStart) +
               ", " + std::to_string(applicationStop) + "]";
    if (impossible_movement)
    {
        env_info += "[impossible movement]";
    }
    NS_LOG_INFO("MyGetExtraInfo: " << env_info);
    return env_info;
}

bool
ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    env_action = discrete->GetValue();
    uint32_t uav_number = env_action / 4;
    env_action = env_action % nGateways;
    impossible_movement = false;
    if (env_action < env_action_space_size)
    {
        FindNewPosition(env_action, uav_number);
        ScheduleNextDataCollect();
        NS_LOG_INFO("MyExecuteActions: " << action);
    }
    return true;
}

Ptr<OpenGymDataContainer>
GetObservation()
{
    uint32_t parameterNum = 2;
    std::vector<uint32_t> shape = {
        parameterNum,
    };

    Ptr<OpenGymBoxContainer<uint32_t>> box = CreateObject<OpenGymBoxContainer<uint32_t>>(shape);
    for (auto g = gateways.Begin(); g != gateways.End(); ++g)
    {
        Ptr<Node> object = *g;
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        mobility = netDevice->GetNode()->GetObject<MobilityModel>();
        uav_position = mobility->GetPosition();
        box->AddValue(uav_position.x);
        box->AddValue(uav_position.y);
        box->AddValue(uav_position.z);
    }
    NS_LOG_INFO("MyGetObservation: " << box);
    return box;
}

Ptr<OpenGymSpace>
GetObservationSpace()
{
    uint32_t parameterNum = 2;
    std::vector<uint32_t> shape = {
        parameterNum,
    };
    float low = 0.0;
    float high = 10000.0;
    std::string dtype = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    NS_LOG_INFO("MyGetObservationSpace: " << box);
    return box;
}

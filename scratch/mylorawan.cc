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
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-module.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <iomanip>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("DensityOrientedExperiment");

NodeContainer endDevices;
NodeContainer gateways;
Ptr<LoraChannel> channel;
MobilityHelper mobilityED;
MobilityHelper mobilityGW;

int nDevices = 0;
int nGateways = 0;

int noMoreReceivers = 0;
int interfered = 0;
int received = 0;
int underSensitivity = 0;

int sent = 0;

/**********************
 *  Global Callbacks  *
 **********************/

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
    int outcomeNumber;
    std::vector<enum PacketOutcome> outcomes;
};

std::map<Ptr<const Packet>, myPacketStatus> packetTracker;

void
CheckReceptionByAllGWsComplete(std::map<Ptr<const Packet>, myPacketStatus>::iterator it)
{
    // Check whether this packet is received by all gateways
    if ((*it).second.outcomeNumber == nGateways)
    {
        // Update the statistics
        myPacketStatus status = (*it).second;
        for (int j = 0; j < nGateways; j++)
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

int
main(int argc, char* argv[])
{
    double simulationTime = 1200;
    int appPeriodSeconds = 40;
    bool okumura = false;
    bool verbose = false;
    int seed = 1;
    bool up = true;
    int packetSize = 50;
    bool printRates = true;

    CommandLine cmd;
    cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
    cmd.AddValue("nGateways", "Number of gateways to include in the simulation", nGateways);
    cmd.AddValue("okumura", "Uses okumura-hate propagation mode", okumura);
    cmd.AddValue("verbose", "Whether to print output or not", verbose);
    cmd.AddValue("seed", "Whether to print result rates", seed);
    cmd.AddValue("up", "Spread Factor UP", up);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed);

    Config::SetDefault("ns3::EndDeviceLorawanMac::DRControl", BooleanValue(true));

    // Set up logging
    if (verbose)
    {
        LogComponentEnable("DensityOrientedExperiment", LOG_LEVEL_ALL);
        LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
        LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
        LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
        LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
        LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
        LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
        LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
        LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
        LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
        LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
        LogComponentEnable("NetworkServerHelper", LOG_LEVEL_ALL);
        LogComponentEnable("AdrComponent", LOG_LEVEL_ALL);
        LogComponentEnableAll(LOG_PREFIX_FUNC);
        LogComponentEnableAll(LOG_PREFIX_NODE);
        LogComponentEnableAll(LOG_PREFIX_TIME);
    }

    /************************
     *  Create the channel  *
     ************************/
    // Create the lora channel object
    // modelo de propagação (okumura ou logdistance)
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    if (okumura)
    {
        Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel>();
        channel = CreateObject<LoraChannel>(loss, delay);
    }
    else
    {
        Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
        loss->SetPathLossExponent(3.76);
        loss->SetReference(1, 10.0);
        channel = CreateObject<LoraChannel>(loss, delay);
    }

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

    Ptr<ListPositionAllocator> allocatorGW = CreateObject<ListPositionAllocator>();

    gateways.Create(nGateways);

    //    for (int i = 0; i < nGateways; i++)
    //    {
    allocatorGW->Add(Vector(5000, 5000, 30));
    //    allocatorGW->Add(Vector(0, 0, 30));
    //    allocatorGW->Add(Vector(10000, 10000, 30));
    //    }

    mobilityGW.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGW.SetPositionAllocator(allocatorGW);
    mobilityGW.Install(gateways);

    // Create a net device for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    for (NodeContainer::Iterator g = gateways.Begin(); g != gateways.End(); ++g)
    {
        Ptr<Node> object = *g;
        // Get the device
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy()->GetObject<GatewayLoraPhy>();
        gwPhy->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&PacketReceptionCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseInterference",
                                          MakeCallback(&InterferenceCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseNoMoreReceivers",
                                          MakeCallback(&NoMoreReceiversCallback));
        gwPhy->TraceConnectWithoutContext("LostPacketBecauseUnderSensitivity",
                                          MakeCallback(&UnderSensitivityCallback));
    }

    NS_LOG_DEBUG("Completed configuration");

    if (up)
    {
        ns3::lorawan::LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);
    }

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/

    Time appStopTime = Seconds(simulationTime); // ten minutes
    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(appPeriodSeconds)); // each 60 seconds
    appHelper.SetPacketSize(packetSize);
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    appContainer.Start(Seconds(0));
    appContainer.Stop(appStopTime);

    /**************************
     *  Create Network Server  *
     ***************************/

    // Create the NS node
    NodeContainer networkServer;
    networkServer.Create(1);
    // Create the NetworkServerHelper
    NetworkServerHelper nsHelper = NetworkServerHelper();
    // Create the ForwarderHelper
    ForwarderHelper forHelper = ForwarderHelper();

    // Create a NS for the network
    nsHelper.SetAdr("ns3::AdrComponent");
    nsHelper.EnableAdr(true);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.SetGateways(gateways);
    nsHelper.Install(networkServer);

    // Create a forwarder for each gateway
    forHelper.Install(gateways);

    /****************
     *  Simulation  *
     ****************/

    Simulator::Stop(appStopTime + Minutes(10));
    Simulator::Run();
    NS_LOG_INFO("Computing performance metrics...");

    if (printRates)
    {
        /**
         * Print PACKETS
         * **/

        LoraTag tag;

        int numPackets = 0;
        int lost = 0;
        int receiv = 0;
        for (std::map<Ptr<const Packet>, myPacketStatus>::iterator p = packetTracker.begin();
             p != packetTracker.end();
             ++p)
        {
            numPackets += 1;
            (*p).second.packet->PeekPacketTag(tag);
            if ((*p).second.receiverId == 0)
            {
                lost++;
            }
            else
            {
                receiv++;
            }
            std::cout << "Sender:" << (*p).second.senderId << " Receiver:" << (*p).second.receiverId
                      << " Sent:" << (*p).second.sentTime.GetSeconds()
                      << " Received:" << (*p).second.receivedTime.GetSeconds() << " Delay:"
                      << (*p).second.receivedTime.GetSeconds() - (*p).second.sentTime.GetSeconds()
                      << " SF:" << std::to_string((*p).second.senderSF)
                      << " Destr:" << std::to_string(tag.GetDestroyedBy())
                      << " Freq:" << std::to_string(tag.GetFrequency())
                      << " rxPower:" << std::to_string(tag.GetReceivePower())
                      << " packRx::" << std::to_string((*p).second.receiverTP)
                      << " SNR: " << std::to_string(tag.GetSnr()) << std::endl;
        }
        std::cout << "Total: " << numPackets << " Sent: " << sent << " Lost: " << lost
                  << " Received: " << receiv << std::endl;
    }

    Simulator::Destroy();
    return 0;
}
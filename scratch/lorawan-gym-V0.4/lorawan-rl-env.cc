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
 * Based on: <https://github.com/20kaushik02/TCP-RL> (c) Kaushik Narayan R
 */

#include "lorawan-rl-env.h"

#include "ns3/lora-tag.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ns3::LorawanGymEnv");
NS_OBJECT_ENSURE_REGISTERED(LorawanGymEnv);

LorawanGymEnv::LorawanGymEnv()
{
    NS_LOG_FUNCTION(this);
    SetOpenGymInterface(OpenGymInterface::Get());
}

LorawanGymEnv::~LorawanGymEnv()
{
    NS_LOG_FUNCTION(this);
}

void
LorawanGymEnv::ScheduleNextStateRead()
{
    NS_LOG_FUNCTION(this);
    Simulator::Schedule(Seconds(0), &LorawanGymEnv::ScheduleNextStateRead, this);
    Notify();
}

TypeId
LorawanGymEnv::GetTypeId(void)
{
    NS_LOG_UNCOND("LorawanGymEnv::GetTypeId");
    static TypeId tid =
        TypeId("ns3::LorawanGymEnv").SetParent<OpenGymEnv>().SetGroupName("OpenGym");
    return tid;
}

void
LorawanGymEnv::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

void
LorawanGymEnv::SetDuration(Time duration)
{
    NS_LOG_FUNCTION(this);
    m_duration = duration;
}

void
LorawanGymEnv::SetUAVMovementStep(double step)
{
    NS_LOG_FUNCTION(this);
    m_UAV_MovementStep = step;
}

void
LorawanGymEnv::SetReward(double reward)
{
    NS_LOG_FUNCTION(this);
    m_reward = reward;
}

/*
Define action space
*/
Ptr<OpenGymSpace>
LorawanGymEnv::GetActionSpace()
{
    /**
     * O espaço de ações contém quatro ações esperadas, mover para cima,
     * mover para baixo, mover para esquerda e mover para a direita.
     * **/
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace>(m_action_space);
    NS_LOG_UNCOND("GetActionSpace: " << space);
    return space;
}

/*
Define game over condition
*/
bool
LorawanGymEnv::GetGameOver()
{
    NS_LOG_UNCOND("MyGetGameOver: " << m_isGameOver);
    return m_isGameOver;
}

/*
Define reward function
*/
float
LorawanGymEnv::GetReward()
{
    NS_LOG_UNCOND("MyGetReward: " << m_envReward);
    return m_envReward;
}

/*
Define extra info. Optional
*/
std::string
LorawanGymEnv::GetExtraInfo()
{
    NS_LOG_UNCOND("MyGetExtraInfo: " << m_info);
    return m_info;
}

/*
Execute received actions
*/
bool
LorawanGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    m_action = discrete->GetValue();

    NS_LOG_UNCOND("MyExecuteActions: " << action);
    return true;
}

Ptr<OpenGymDataContainer>
LorawanGymEnv::GetObservation()
{
    uint32_t parameterNum = 7;
    std::vector<uint32_t> shape = {
        parameterNum,
    };
    Ptr<OpenGymBoxContainer<uint32_t>> box = CreateObject<OpenGymBoxContainer<uint32_t>>(shape);
    box->AddValue(m_uav_id);
    box->AddValue(m_uav_position.x);
    box->AddValue(m_uav_position.y);
    box->AddValue(m_uav_position.z);
    box->AddValue(m_pkt_transmitted);
    box->AddValue(m_pkt_received);
    box->AddValue(m_qos);

    NS_LOG_UNCOND("MyGetObservation: " << box);
    return box;
}

Ptr<OpenGymSpace>
LorawanGymEnv::GetObservationSpace()
{
    uint32_t parameterNum = 7;
    std::vector<uint32_t> shape = {
        parameterNum,
    };
    float low = 0.0;
    float high = 1000000000.0;
    std::string dtype = TypeNameGet<uint32_t>();

    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    NS_LOG_INFO("MyGetObservationSpace: " << box);
    return box;
}

/**
 * Packet tracing
 * */

void
LorawanGymEnv::PktTrace(Ptr<const Packet> packet, uint32_t systemId)
{
    NS_LOG_UNCOND("Packet Tracer!");
}

void
LorawanGymEnv::CheckReceptionByAllGWsComplete(
    std::map<Ptr<const Packet>, myPacketStatus>::iterator it)
{
    // Check whether this packet is received by all gateways
    if ((*it).second.outcomeNumber == m_number_of_gateways)
    {
        // Update the statistics
        myPacketStatus status = (*it).second;
        for (uint32_t j = 0; j < m_number_of_gateways; j++)
        {
            switch (status.outcomes.at(j))
            {
            case _RECEIVED: {
                m_pkt_received += 1;
                break;
            }
            case _UNDER_SENSITIVITY: {
                m_pkt_underSensitivity += 1;
                break;
            }
            case _NO_MORE_RECEIVERS: {
                m_pkt_noMoreReceivers += 1;
                break;
            }
            case _INTERFERED: {
                m_pkt_interfered += 1;
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
LorawanGymEnv::TransmissionCallback(Ptr<const Packet> packet,
                                    uint32_t systemId,
                                    Ptr<LorawanGymEnv> env)
{
    NS_LOG_INFO("Transmitted a packet from device " << systemId);
    // Coleta informações sobre o pacote (SF, TP, etc.)
    lorawan::LoraTag tag;
    packet->PeekPacketTag(tag);

    // Create a packetStatus
    myPacketStatus status;
    status.packet = packet;
    status.senderId = systemId;
    status.sentTime = Simulator::Now();
    status.outcomeNumber = 0;
    status.outcomes = std::vector<enum PacketOutcome>(m_number_of_gateways, _UNSET);
    status.senderSF = tag.GetSpreadingFactor();

    setMPacketTracker(std::pair<Ptr<const Packet>, myPacketStatus>(packet, status), env);
    m_pkt_transmitted += 1;
}

void
LorawanGymEnv::PacketReceptionCallback(Ptr<const Packet> packet,
                                       uint32_t systemId,
                                       Ptr<LorawanGymEnv> env)
{
    // Remove the successfully received packet from the list of sent ones
    NS_LOG_INFO("A packet was successfully received at gateway " << systemId);

    // Coleta informações sobre o pacote (SF, TP, etc.)
    lorawan::LoraTag tag;
    packet->PeekPacketTag(tag);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it =
        getMPacketTrackerIterator(env, packet);

    if (it != getMPacketTrackerIteratorEnd(env))
    {
        if ((*it).second.outcomes.size() > systemId - m_number_of_devices)
        {
            // lembre que o ID do gateways é enumerado após todos os devices, logo numero sequencial
            //  do GW é contado subtraindo o m_number_of_devices. Ainda que os pacotes originados do
            //  mesmo device no mesmo instante é recebido pelos gateways na área de alcance, <e que
            //  essas duplicidades são removidas pela CheckReceptionByAllGWsComplete> sem certeza
            //  nisso
            (*it).second.outcomes.at(systemId - m_number_of_devices) = _RECEIVED;
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
LorawanGymEnv::InterferenceCallback(Ptr<const Packet> packet,
                                    uint32_t systemId,
                                    Ptr<LorawanGymEnv> env)
{
    NS_LOG_INFO("A packet was lost because of interference at gateway " << systemId);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it =
        getMPacketTrackerIterator(env, packet);
    if ((*it).second.outcomes.size() > systemId - m_number_of_devices)
    {
        (*it).second.outcomes.at(systemId - m_number_of_devices) = _INTERFERED;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

void
LorawanGymEnv::NoMoreReceiversCallback(Ptr<const Packet> packet,
                                       uint32_t systemId,
                                       Ptr<LorawanGymEnv> env)
{
    NS_LOG_INFO("A packet was lost because there were no more receivers at gateway " << systemId);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it =
        getMPacketTrackerIterator(env, packet);
    if ((*it).second.outcomes.size() > systemId - m_number_of_devices)
    {
        (*it).second.outcomes.at(systemId - m_number_of_devices) = _NO_MORE_RECEIVERS;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

void
LorawanGymEnv::UnderSensitivityCallback(Ptr<const Packet> packet,
                                        uint32_t systemId,
                                        Ptr<LorawanGymEnv> env)
{
    NS_LOG_INFO("A packet arrived at the gateway under sensitivity" << systemId);

    std::map<Ptr<const Packet>, myPacketStatus>::iterator it =
        getMPacketTrackerIterator(env, packet);
    if ((*it).second.outcomes.size() > systemId - m_number_of_devices)
    {
        (*it).second.outcomes.at(systemId - m_number_of_devices) = _UNDER_SENSITIVITY;
        (*it).second.outcomeNumber += 1;
    }
    CheckReceptionByAllGWsComplete(it);
}

void
LorawanGymEnv::RecalcularTudo(Ptr<LorawanGymEnv> env, Ptr<const MobilityModel> mob)
{
    NS_LOG_UNCOND("DoPacketsTracking");
}

void
LorawanGymEnv::setMNumberOfDevices(uint32_t mNumberOfDevices)
{
    m_number_of_devices = mNumberOfDevices;
}

void
LorawanGymEnv::setMNumberOfGateways(uint32_t mNumberOfGateways)
{
    m_number_of_gateways = mNumberOfGateways;
}

void
LorawanGymEnv::setMPktInterfered(int mPktInterfered)
{
    m_pkt_interfered = mPktInterfered;
}

void
LorawanGymEnv::setMPktReceived(int mPktReceived)
{
    m_pkt_received = mPktReceived;
}

void
LorawanGymEnv::setMPktUnderSensitivity(int mPktUnderSensitivity)
{
    m_pkt_underSensitivity = mPktUnderSensitivity;
}

void
LorawanGymEnv::setMPktTransmitted(int mPktTransmitted)
{
    m_pkt_transmitted = mPktTransmitted;
}

void
LorawanGymEnv::setMPktNoMoreReceivers(int mPktNoMoreReceivers)
{
    m_pkt_noMoreReceivers = mPktNoMoreReceivers;
}

const std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus>::iterator
LorawanGymEnv::getMPacketTrackerIterator(Ptr<LorawanGymEnv> env, Ptr<const Packet> packet)
{
    return env->m_packetTracker.find(packet);
}

const std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus>::iterator
LorawanGymEnv::getMPacketTrackerIteratorEnd(Ptr<LorawanGymEnv> env)
{
    return env->m_packetTracker.end();
}

const std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus>::iterator
LorawanGymEnv::getMPacketTrackerIteratorBegin(Ptr<LorawanGymEnv> env)
{
    return env->m_packetTracker.begin();
}

void
LorawanGymEnv::setMPacketTracker(std::pair<Ptr<const Packet>, myPacketStatus> pair,
                                 Ptr<LorawanGymEnv> env)
{
    env->m_packetTracker.insert(pair);
}

} // namespace ns3
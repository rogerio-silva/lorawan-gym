/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
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
 * Author: Piotr Gawlowicz <gawlowicz@tkn.tu-berlin.de>
 */

#include "mygym.h"

#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/object.h"
#include "ns3/wifi-module.h"

#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MyGymEnv");

NS_OBJECT_ENSURE_REGISTERED(MyGymEnv);

MyGymEnv::MyGymEnv() {
    NS_LOG_FUNCTION (this);
}

MyGymEnv::MyGymEnv(NodeContainer uavs)
{
    NS_LOG_FUNCTION (this);
    m_uavs_number = uavs.GetN();

    m_area_dimensions = 3;
    m_action_space = 4;
    m_uavs = uavs;
    for (NodeContainer::Iterator g = uavs.Begin(); g != uavs.End(); ++g)
    {
        Ptr<MobilityModel> mobility2 = (*g)->GetObject<MobilityModel> ();
        Vector position = mobility2->GetPosition ();
        mobility2->SetPosition (Vector(999,999,999));
        m_uavs_positions.push_back(position);
        m_uavs_id.push_back(uavs.Get(0)->GetId());
    }
    m_current_uav = 0;
    m_current_uav_id = m_uavs_id.at(0);
    //    Simulator::Schedule (Seconds(0.0), &MyGymEnv::ScheduleNextStateRead, this);
}

MyGymEnv::~MyGymEnv()
{
    NS_LOG_FUNCTION(this);
}

void
MyGymEnv::ScheduleNextStateRead()
{
    NS_LOG_FUNCTION(this);
    Simulator::Schedule(m_interval, &MyGymEnv::ScheduleNextStateRead, this);
    Notify();
}

TypeId
MyGymEnv::GetTypeId()
{
    static TypeId tid = TypeId("MyGymEnv")
                            .SetParent<OpenGymEnv>()
                            .SetGroupName("OpenGym")
                            .AddConstructor<MyGymEnv>();
    return tid;
}

void
MyGymEnv::DoDispose()
{
    NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
MyGymEnv::GetActionSpace()
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

Ptr<OpenGymSpace>
MyGymEnv::GetObservationSpace()
{
    /**
     * O espaço de observação conterá as posições dos UAVs (x,y,z)
     * Inicialmente a posição z será fixa em 30 e o identificador do drone
     * que está que será posicionado.
     * <drone: int, position: {x,y} // ,z}
     * **/
    NS_LOG_FUNCTION(this);
    uint low = 0;
    uint high = H_POSITIONS;
    std::vector<uint32_t> shape = {
        m_area_dimensions,
    };
    std::string dType = TypeNameGet<uint32_t>();
    Ptr<OpenGymDiscreteSpace> uav = CreateObject<OpenGymDiscreteSpace>(m_uavs_number);
    Ptr<OpenGymBoxSpace> position = CreateObject<OpenGymBoxSpace>(low, high, shape, dType);
    Ptr<OpenGymDictSpace> space = CreateObject<OpenGymDictSpace>();
    space->Add("uav", uav);
    space->Add("position", position);

    NS_LOG_UNCOND("GetObservationSpace: " << space);
    return space;
}

Ptr<OpenGymDataContainer>
MyGymEnv::GetObservation()
{
    NS_LOG_FUNCTION(this);
//    uint low = 0;
//    uint high = H_POSITIONS;
    std::vector<uint32_t> shape = {
        m_area_dimensions,
    };
    std::string dType = TypeNameGet<uint32_t>();
    Ptr<OpenGymDiscreteContainer> uav = CreateObject<OpenGymDiscreteContainer>(m_uavs_number);
    Ptr<OpenGymBoxContainer<uint32_t>> box = CreateObject<OpenGymBoxContainer<uint32_t>>(shape);
    for(uint32_t i=0; i< m_uavs_number; ++i){
        uav->SetValue(m_uavs_id.at(i));
        box->AddValue(m_uavs_positions.at(i).x);
        box->AddValue(m_uavs_positions.at(i).y);
        box->AddValue(m_uavs_positions.at(i).z);
    }
    Ptr<OpenGymDictContainer> space = CreateObject<OpenGymDictContainer>();
    space->Add("uav", uav);
    space->Add("position", box);

    NS_LOG_UNCOND("MyGetObservation: " << space);
    return space;
}

bool
MyGymEnv::GetGameOver()
{
    /**
     * Hipótese de Game over:
     * 1. Alcançar o QoS pretendido
     * Para isso, o calculo do QoS deverá assegurar que todos os devices serão
     * atendidos, caso contrário Qos deve ser zero.
     * **/
    NS_LOG_FUNCTION(this);
    //    bool isGameOver = m_uav_qos >= QOS_BOUND;
    bool isGameOver = false;

    NS_LOG_UNCOND("MyGetGameOver: " << isGameOver);
    return isGameOver;
}

float
MyGymEnv::GetReward()
{
    NS_LOG_FUNCTION(this);
    /**
     * Computar a recompensa em função do QoS alcançado
     * 1. Calcula QoS dos devices
     *    - Todos os devices atendidos: +QoS
     *    - Há devices não atendidos: -1
     * 2. Calcula a QoS dos UAVs
     * 3. Calcula a QoS da época (De todos os UAVs implantados)
     * 4. Estima a recompensa em função da QoS da época
     * 4.1 Se alcançou a Qos: +QoS [OK] +10 por enquanto kkkk
     * 4.2 Se não alcançou a QoS: -1 [OK]
     * 4.3 Se houve colisão: -3 [OK]
     * 4.4 Se não atendeu a todos os devices: -1 [TODO]
     * 4.5 Executou movimento proibido: -2 [OK]
     * Avaliar a influência desses pesos e se for o caso ajustá-los
     * **/

    float reward;

    if (HasCollision(m_uavs_positions.at(m_current_uav),m_current_uav_id))
    {
        reward = -3.0;
    }
    else
    {
        if (!m_uav_impossible_movement)
        {
            if(m_uav_qos==0.0){ //Devices não atendidos
                reward = -2.0;
            }else {
                if (m_uav_qos > QOS_BOUND)
                {
                    reward = +10;
                }
                else // não alcançou a QoS
                {
                    reward = -1.0;
                }
            }
        }
        else
        {
            reward = -2.5;
        }
    }

    NS_LOG_UNCOND("MyGetReward: " << reward);
    return reward;
}

std::string
MyGymEnv::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    std::string g_QoS = "Uav_QoS: " + std::to_string(m_uav_qos);
    std::string myInfo = g_QoS;
    NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
    return myInfo;
}

bool
MyGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this);
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    uint32_t act = discrete->GetValue();
    std::pair<Vector3D, bool> new_pos = NextPos(m_uavs_positions.at(m_current_uav), m_current_uav, act);
    if (!new_pos.second)
    {
        m_uavs.Get(0)->GetObject<MobilityModel>()->SetPosition(new_pos.first);
        m_uavs_positions.at(m_current_uav) = new_pos.first;
    }
    m_uav_impossible_movement = new_pos.second;

    NS_LOG_UNCOND("New pos: " << new_pos.first << " Possible: " << !(new_pos.second));
    return true;
}

/**
 * Verifica colisões entre drones (drones ocuoando o mesmo espaço
 **/
bool
MyGymEnv::HasCollision(Vector3D new_position, uint32_t uav_id)
{
    // Verify new position occupation and returns drone collisions
    bool collision = false;
    for (NodeContainer::Iterator j = m_uavs.Begin(); j != m_uavs.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        Vector3D actual_position = position->GetPosition();
        if (uav_id != (*j)->GetId() && new_position == actual_position)
        {
            collision = true;
            break;
        }
    }
    return collision;
}

std::pair<Vector3D, bool>
MyGymEnv::NextPos(Vector3D actual_pos, uint32_t uav_id, uint32_t action)
{
    Vector3D new_pos = actual_pos;
    bool impossible_movement = false;
    switch (action)
    {
    case 0: // UP
        if (actual_pos.y + MOVEMENT_STEP > AREA_HEIGHT)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(actual_pos.x, actual_pos.y + MOVEMENT_STEP, actual_pos.z);
        }
        break;
    case 1: // DOWN
        if (actual_pos.y - MOVEMENT_STEP < 0)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(actual_pos.x, actual_pos.y - MOVEMENT_STEP, actual_pos.z);
        }
        break;
    case 2: // LEFT
        if (actual_pos.x - MOVEMENT_STEP < 0)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(actual_pos.x - MOVEMENT_STEP, actual_pos.y, actual_pos.z);
        }
        break;
    case 3: // RIGHT
        if (actual_pos.x + MOVEMENT_STEP > AREA_WIDTH)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(actual_pos.x + MOVEMENT_STEP, actual_pos.y, actual_pos.z);
        }
    }

    if (HasCollision(new_pos, uav_id))
    {
        impossible_movement = true;
    }

    return std::make_pair(new_pos, impossible_movement);
}

void
MyGymEnv::SetQoS(double qos)
{
    NS_LOG_INFO(Simulator::Now());
    m_uav_qos = qos;
}

// void CollectCommData(Ptr<MyGymEnv> entity, Ptr<const MobilityModel> mobility){
//void
//MyGymEnv::CollectCommData(Ptr<MyGymEnv> entity,
//                          NodeContainer endDevices,
//                          std::map<Ptr<const Packet>, myPacketStatus> packetTracker,
//                          int sent)
//{
//    /**
//     * Devices data
//     * **/
//    NS_LOG_INFO(Simulator::Now);
//    bool printRates = false;
//    double mean_qos = 0.0;
//    uint32_t nDevices = endDevices.GetN();
//    // i:: device, j[0] sf j[1] data rate j[2] delay
//    std::vector<std::vector<double>> deviceData;
//    std::vector<std::vector<std::vector<double>>> deviceSimulatedData;
//    double sumQos = 0.0;
//
//    NS_LOG_UNCOND("Collecting package data!");
//
//    deviceSimulatedData.reserve(nDevices);
//    for (uint32_t i = 0; i < nDevices; ++i)
//    {
//        deviceSimulatedData.push_back({{}, {}, {}});
//    }
//
//    lorawan::LoraTag tag;
//    if (printRates)
//    {
//        std::cout << std::endl << "Devices Simulated Data" << std::endl;
//    }
//    int numPackets = 0;
//    int _lost = 0;
//    int _received = 0;
//    for (std::map<Ptr<const Packet>, myPacketStatus>::iterator p = packetTracker.begin();
//         p != packetTracker.end();
//         ++p)
//    {
//        numPackets += 1;
//        (*p).second.packet->PeekPacketTag(tag);
//        if ((*p).second.receiverId == 0)
//        {
//            _lost++;
//        }
//        else
//        {
//            _received++;
//        }
//
//        double size = (*p).second.packet->GetSize() * 8; // bits
//        int devID = (*p).second.senderId;
//        double sf;
//        sf = ((*p).second.senderSF);
//        double dk;
//        dk = ((*p).second.receivedTime.GetSeconds() - (*p).second.sentTime.GetSeconds());
//        double rk;
//        rk = (size / ((*p).second.receivedTime.GetSeconds() - (*p).second.sentTime.GetSeconds()));
//        deviceSimulatedData[devID][0].push_back(sf);
//        deviceSimulatedData[devID][1].push_back(rk);
//        deviceSimulatedData[devID][2].push_back(dk);
//    }
//    std::vector<std::vector<double>> deviceSummarizedData;
//    sumQos = 0.0;
//    for (uint32_t devID = 0; devID < nDevices; ++devID)
//    {
//        double dk = 0;
//        double rk = 0;
//        double sf = deviceSimulatedData[devID][0][0];
//        double qos = 0;
//        int qtd = 0;
//        for (unsigned int i = 0; i < deviceSimulatedData[devID][0].size(); ++i)
//        {
//            if (deviceSimulatedData[devID][2].at(i) > 0.0)
//            {
//                rk += deviceSimulatedData[devID][1].at(i);
//                dk += deviceSimulatedData[devID][2].at(i);
//                qtd += 1;
//            }
//            else
//            {
//                deviceSimulatedData[devID][0].erase(deviceSimulatedData[devID][0].begin() + i);
//                deviceSimulatedData[devID][1].erase(deviceSimulatedData[devID][1].begin() + i);
//                deviceSimulatedData[devID][2].erase(deviceSimulatedData[devID][2].begin() + i);
//            }
//        }
//        // QpS do device
//        qos = (rk / qtd) / MAX_RK + (1 - ((dk / qtd) / MIN_RK));
//        deviceSummarizedData.push_back({sf, rk / qtd, dk / qtd, qos});
//        sumQos += qos;
//    }
//    // QoS médio da execução (todos devices por gateway)
//    mean_qos = sumQos / nDevices;
//
//    if (printRates)
//    {
//        for (uint32_t i = 0; i < nDevices; ++i)
//        {
//            std::cout << i << " " << deviceSummarizedData[i][0] << " " << deviceSummarizedData[i][1]
//                      << " " << deviceSummarizedData[i][2] << " " << deviceSummarizedData[i][3]
//                      << std::endl;
//        }
//        if (isNaN(mean_qos))
//        {
//            std::cout << "Não atende os critérios de QoS" << std::endl;
//        }
//        else
//        {
//            std::cout << "QoS Simulado: " << mean_qos << std::endl;
//        }
//
//        std::cout << "Total: " << numPackets << " Sent: " << sent << " Lost: " << _lost
//                  << " Received: " << _received << " QoS: " << (isNaN(mean_qos)?0.0:mean_qos) << std::endl;
//    }
//    entity->SetQoS(isNaN(mean_qos)?0.0:mean_qos);
//    entity->Notify();
//}

} // namespace ns3
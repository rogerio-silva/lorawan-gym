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

namespace ns3
{
namespace lorawan
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
LorawanGymEnv::ScheduleNextStateRead ()
{
    NS_LOG_FUNCTION (this);
    Simulator::Schedule (m_timeStep, &LorawanGymEnv::ScheduleNextStateRead, this);
    Notify();
}

TypeId
LorawanGymEnv::GetTypeId (void)
{
    NS_LOG_UNCOND("LorawanGymEnv::GetTypeId");
    static TypeId tid = TypeId ("ns3::TcpGymEnv")
                            .SetParent<OpenGymEnv> ()
                            .SetGroupName ("OpenGym")
        ;
    return tid;
}

void
LorawanGymEnv::DoDispose ()
{
    NS_LOG_FUNCTION (this);
}

void
LorawanGymEnv::SetDuration(Time value)
{
    NS_LOG_FUNCTION (this);
    m_duration = value;
}

void
LorawanGymEnv::SetTimeStep(Time value)
{
    NS_LOG_FUNCTION (this);
    m_timeStep = value;
}

void
LorawanGymEnv::SetReward(double value)
{
    NS_LOG_FUNCTION (this);
    m_reward = value;
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
    NS_LOG_UNCOND ("MyGetGameOver: " << m_isGameOver);
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

    NS_LOG_UNCOND ("MyExecuteActions: " << action);
    return true;
}

Ptr<OpenGymDataContainer>
LorawanGymEnv::GetObservation()
{
    uint32_t parameterNum = 7;
    std::vector<uint32_t> shape = {parameterNum,};
    Ptr<OpenGymBoxContainer<uint32_t> > box = CreateObject<OpenGymBoxContainer<uint32_t> >(shape);
    box->AddValue(m_uav_id);
    box->AddValue(m_uav_position.x);
    box->AddValue(m_uav_position.y);
    box->AddValue(m_uav_position.z);
    box->AddValue(m_packets_sent);
    box->AddValue(m_packets_received);
    box->AddValue(m_qos);

    NS_LOG_UNCOND ("MyGetObservation: " << box);
    return box;
}

Ptr<OpenGymSpace> GetObservationSpace(){
    uint32_t parameterNum = 7;
    float low = 0.0;
    float high = 1000000000.0;
    std::vector<uint32_t> shape = {parameterNum,};
    std::string dType = TypeNameGet<uint64_t> ();

    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dType);
    NS_LOG_UNCOND ("MyGetObservationSpace: " << box);
    return box;
}

} // namespace lorawan
} // namespace ns3
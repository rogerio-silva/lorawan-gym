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
#include "lorawan-rl.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("ns3::LorawanRl");
NS_OBJECT_ENSURE_REGISTERED(LorawanRl);

TypeId
LorawanRl::GetTypeId(void)
{
    NS_LOG_UNCOND("LorawanRl::GetTypeId");
    static TypeId tid = TypeId("ns3::LorawanRl")
                            .SetParent<NetworkStatus>()
                            .SetGroupName("lorawan")
                            .AddConstructor<LorawanRl>()
                            .AddAttribute("Reward",
                                          "Reward",
                                          DoubleValue(0.0),
                                          MakeDoubleAccessor(&LorawanRl::m_reward),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("Duration",
                                          "Simulation Duration. Default: 10000ms",
                                          TimeValue(MilliSeconds(10000)),
                                          MakeTimeAccessor(&LorawanRl::m_duration),
                                          MakeTimeChecker())
                            .AddAttribute("StepTime",
                                          "Step interval used in TCP env. Default: 100ms",
                                          TimeValue(MilliSeconds(100)),
                                          MakeTimeAccessor(&LorawanRl::m_timeStep),
                                          MakeTimeChecker());
    return tid;
}

TypeId
LorawanRl::GetInstanceTypeId() const
{
    NS_LOG_FUNCTION(this);
    return LorawanRl::GetTypeId();
}

LorawanRl::LorawanRl() : NetworkStatus()
{
    NS_LOG_FUNCTION(this);
    m_gym_env = nullptr;
    m_network_statuses = nullptr;
}

LorawanRl::~LorawanRl()
{
    NS_LOG_FUNCTION(this);
    m_gym_env = nullptr;
    m_network_statuses = nullptr;
}

std::string
LorawanRl::GetName() const
{
    NS_LOG_FUNCTION(this);
    return "LorawanRl";
}

void
LorawanRl::CreateGymEnv()
{
    NS_LOG_FUNCTION(this);
}

} // namespace lorawan
} // namespace ns3
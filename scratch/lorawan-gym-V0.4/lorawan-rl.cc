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

#include "lorawan-rl.h"
#include "lorawan-rl-env.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ns3::LorawanRlMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(LorawanRl);

TypeId
LorawanRl::GetTypeId(void)
{
    NS_LOG_UNCOND("LorawanRl::GetTypeId");
    static TypeId tid = TypeId("ns3::LorawanRlMobilityModel")
                            .SetParent<MobilityModel>()
                            .SetGroupName("Mobility")
                            .AddConstructor<LorawanRl>()
                            .AddAttribute("Reward",
                                          "Reward. Default: 0.0",
                                          DoubleValue(0),
                                          MakeDoubleAccessor(&LorawanRl::m_reward),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("Step",
                                          "Movement step. Default: 1000m",
                                          DoubleValue(1000.0),
                                          MakeDoubleAccessor(&LorawanRl::m_movement_step),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("Bounds",
                                          "Bounds of the area to cruise.",
                                          BoxValue(Box(-100.0, 100.0, -100.0, 100.0, 0.0, 100.0)),
                                          MakeBoxAccessor(&LorawanRl::m_bounds),
                                          MakeBoxChecker())
                            .AddAttribute("NumberOfDevices",
                                          "Number of devices.",
                                          UintegerValue(0),
                                          MakeUintegerAccessor(&LorawanRl::m_number_of_devices),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("NumberOfGateways",
                                          "Number of gateways.",
                                          UintegerValue(0),
                                          MakeUintegerAccessor(&LorawanRl::m_number_of_gateways),
                                          MakeUintegerChecker<uint32_t>())
                            .AddAttribute("SimulationTime",
                                          "Whether in simulation time.",
                                          BooleanValue(false),
                                          MakeBooleanAccessor(&LorawanRl::m_simulation_time),
                                          MakeBooleanChecker());

    return tid;
}

LorawanRl::LorawanRl()
    : MobilityModel()
{
    NS_LOG_FUNCTION(this);
}

LorawanRl::~LorawanRl()
{
    NS_LOG_FUNCTION(this);
}

void
LorawanRl::DoDispose()
{
    // chain up
    MobilityModel::DoDispose();
}

std::string
LorawanRl::GetName() const
{
    NS_LOG_FUNCTION(this);
    return "LorawanRl";
}

Vector
LorawanRl::DoGetPosition() const
{
    NS_LOG_FUNCTION(this);
    return m_position;
}

void
LorawanRl::DoSetPosition(const Vector& position)
{
    NS_LOG_FUNCTION(this);
    m_position = position;
    // Executar somente em "simulation time" NOT WORKING
    if(!m_lorawanGymEnv && Simulator::Now().GetSeconds() > 0){
        CreateGymEnv();
    }
    NotifyCourseChange();
}

/**
 * Considers that an RL algorithm determines the movement (repositioning).
 * This mobility model assumes that the nodes are static, the current
 * position does not change once it has been set, and until it is set
 * again to a new value through SetPosition() method.
 * @return  Vector(0,0,0) that means constant
 */

Vector
LorawanRl::DoGetVelocity() const
{
    NS_LOG_FUNCTION(this);
    return Vector(0.0, 0.0, 0.0);
}

void
LorawanRl::CreateGymEnv()
{
    NS_LOG_FUNCTION(this);
    Ptr<LorawanGymEnv> env = CreateObject<LorawanGymEnv>();
    env->SetReward(m_reward);
    env->SetUAVMovementStep(m_movement_step);
    m_lorawanGymEnv = env;
    m_lorawanGymEnv->setMPktNoMoreReceivers(0);
    m_lorawanGymEnv->setMPktInterfered(0);
    m_lorawanGymEnv->setMPktReceived(0);
    m_lorawanGymEnv->setMPktUnderSensitivity(0);
    m_lorawanGymEnv->setMPktTransmitted(0);
    m_lorawanGymEnv->setMNumberOfDevices(m_number_of_devices);
    m_lorawanGymEnv->setMNumberOfGateways(m_number_of_gateways);
    // cria um rl_packetTracker vazio e inicializa o m_packetTracker
    static std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus> rl_packetTracker;
    m_lorawanGymEnv->m_packetTracker = rl_packetTracker;
    GetDevices();
    GetGateways();
    ConnectTraceCallbacks();
}

/**
 * Find the new position of the node.
 * In case of area violation, a penalty is notified to the GYM
 */

void
LorawanRl::FindNewPosition(uint32_t action)
{
    NS_LOG_FUNCTION(this);
    // Find a new position from the new state obtained from the GYM
    bool impossible_movement = false;
    Vector new_pos = m_position;

    // Check the possibility of executing the movement,
    // initially there will be no movements on the z-axis
    switch (action)
    {
    case 0: // UP
        if (m_position.y + m_movement_step > m_bounds.yMax)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(m_position.x, m_position.y + m_movement_step, m_position.z);
        }
        break;
    case 1: // DOWN
        if (m_position.y - m_movement_step < 0)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(m_position.x, m_position.y - m_movement_step, m_position.z);
        }
        break;
    case 2: // LEFT
        if (m_position.x - m_movement_step < 0)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(m_position.x - m_movement_step, m_position.y, m_position.z);
        }
        break;
    case 3: // RIGHT
        if (m_position.x + m_movement_step > m_bounds.xMax)
        {
            impossible_movement = true;
        }
        else
        {
            new_pos = Vector(m_position.x + m_movement_step, m_position.y, m_position.z);
        }
    }
    if (impossible_movement) // notify penalty to GYM
        NS_LOG_INFO("Movement is not allowed; the node position has not changed!");
    DoMove(new_pos); // Do the movement
}

/**
 * Move the object to the new position (an RL algorithm determines the new position).
 */
void
LorawanRl::DoMove(Vector position)
{
    NS_LOG_FUNCTION(this);
    if (!m_lorawanGymEnv)
    {
        CreateGymEnv();
    }
    else
    {
        m_position = position;
        NotifyCourseChange();
    }
}

void
LorawanRl::GetDevices()
{
    NS_LOG_FUNCTION(this);
    for (uint i = 0; i < m_number_of_devices ; ++i)
    {
        m_devices.Add(NodeList::GetNode(i));
    }
}

void
LorawanRl::GetGateways()
{
    NS_LOG_FUNCTION(this);
    for (uint i = m_number_of_devices; i < m_number_of_devices + m_number_of_gateways; ++i)
    {
        m_gateways.Add(NodeList::GetNode(i));
    }
}

/**
 * Connect trace callback
 *
 * */
void
LorawanRl::ConnectTraceCallbacks()
{
    NS_LOG_FUNCTION(this);
    for (NodeContainer::Iterator g = m_gateways.Begin(); g != m_gateways.End(); ++g)
    {
        Ptr<Node> object = *g;
        // Get the device
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<MobilityModel> mobility = netDevice->GetObject<MobilityModel>();
        Ptr<lorawan::LoraNetDevice> loraNetDevice = netDevice->GetObject<lorawan::LoraNetDevice>();
        Ptr<lorawan::GatewayLoraPhy> gwPhy =
            loraNetDevice->GetPhy()->GetObject<lorawan::GatewayLoraPhy>();
        gwPhy->TraceConnectWithoutContext(
            "ReceivedPacket",
            MakeCallback(&LorawanGymEnv::PacketReceptionCallback, m_lorawanGymEnv));
        gwPhy->TraceConnectWithoutContext(
            "LostPacketBecauseInterference",
            MakeCallback(&LorawanGymEnv::InterferenceCallback, m_lorawanGymEnv));
        gwPhy->TraceConnectWithoutContext(
            "LostPacketBecauseNoMoreReceivers",
            MakeCallback(&LorawanGymEnv::NoMoreReceiversCallback, m_lorawanGymEnv));
        gwPhy->TraceConnectWithoutContext(
            "LostPacketBecauseUnderSensitivity",
            MakeCallback(&LorawanGymEnv::UnderSensitivityCallback, m_lorawanGymEnv));
        // CallBack CourseChange
        std::ostringstream oss;
        oss.str("");
        // Exemplo: "/NodeList/10/$ns3::MobilityModel/CourseChange"
        oss << "/NodeList/" << object->GetId() << "/$ns3::MobilityModel/CourseChange";
        std::cout << oss.str() << std::endl;
        // Quando o drone mudar de posição o ambiente deve recalcular
        Config::Connect(oss.str(),
                        MakeCallback(&LorawanGymEnv::RecalcularTudo, m_lorawanGymEnv));
    }

    for (NodeContainer::Iterator d = m_devices.Begin(); d != m_devices.End(); ++d)
    {
        Ptr<Node> node = *d;
        Ptr<lorawan::LoraNetDevice> loraNetDevice =
            node->GetDevice(0)->GetObject<lorawan::LoraNetDevice>();
        Ptr<lorawan::LoraPhy> phy = loraNetDevice->GetPhy();
        phy->TraceConnectWithoutContext(
            "StartSending",
            MakeCallback(&LorawanGymEnv::TransmissionCallback, m_lorawanGymEnv));
    }
}

} // namespace ns3
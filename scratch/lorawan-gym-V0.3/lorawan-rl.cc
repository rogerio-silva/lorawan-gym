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

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ns3::LorawanRl");
NS_OBJECT_ENSURE_REGISTERED(LorawanRl);

TypeId
LorawanRl::GetTypeId(void)
{
    NS_LOG_UNCOND("LorawanRl::GetTypeId");
    static TypeId tid = TypeId("ns3::LorawanRl")
                            .SetParent<MobilityModel>()
                            .SetGroupName("Mobility")
                            .AddConstructor<LorawanRl>();
    return tid;
}

LorawanRl::LorawanRl() : MobilityModel()
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
    return m_position;
}

void
LorawanRl::DoSetPosition(const Vector& position)
{
    m_position = position;
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
    return Vector(0.0,0.0,0.0);
}

/**
     * Initialize the model and calculate new velocity, direction, and pitch
 */

void
LorawanRl::Start()
{
    DoMove();
}

/**
     * Move the object to the new position (an RL algorithm determines the new position).
 */
void
LorawanRl::DoMove()
{
     NotifyCourseChange();
}


} // namespace ns3
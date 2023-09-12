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

#ifndef MY_GYM_ENTITY_H
#define MY_GYM_ENTITY_H

#include "ns3/lorawan-module.h"
#include "ns3/opengym-module.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/stats-module.h"

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

namespace ns3
{

class MyGymEnv : public OpenGymEnv
{
  public:
    MyGymEnv();
    MyGymEnv(NodeContainer uavs);
    void ScheduleNextStateRead ();
    virtual ~MyGymEnv();
    static TypeId GetTypeId(void);
    virtual void DoDispose();

    Ptr<OpenGymSpace> GetActionSpace();
    Ptr<OpenGymSpace> GetObservationSpace();
    bool GetGameOver();
    Ptr<OpenGymDataContainer> GetObservation();
    float GetReward();
    std::string GetExtraInfo();
    bool ExecuteActions(Ptr<OpenGymDataContainer> action);
    void SetQoS(double qos);//, Ptr<const MobilityModel> model);
    bool HasCollision(Vector3D new_position, uint32_t uav_id);
//    static void CollectCommData(Ptr<MyGymEnv> entity, NodeContainer endDevices, std::map<Ptr<const Packet>, myPacketStatus> packetTracker, int sent);

    std::pair<Vector3D, bool> NextPos(Vector3D actual_pos, uint32_t uav_id, uint32_t action);

  private:
    uint32_t m_uavs_number = 0;
    uint32_t m_current_uav;
    uint32_t m_current_uav_id;
    std::vector<uint32_t> m_uavs_id;
    uint32_t m_action_space = 4; //{Up, Down, Left, Right uav movements}
    uint32_t m_area_dimensions = 3; // {x, y coordinates - TODO z}
    std::vector<Vector3D> m_uavs_positions;
    bool m_uav_impossible_movement = false;
    double m_uav_qos = 0.0;
    NodeContainer m_uavs;
    Time m_interval = Seconds(0.5);
};
} // namespace ns3
#endif // MY_GYM_ENTITY_H

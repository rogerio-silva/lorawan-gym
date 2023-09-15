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

#ifndef LORAWAN_RL_ENV_H
#define LORAWAN_RL_ENV_H

#include "ns3/opengym-module.h"
#include "ns3/end-device-status.h"

namespace ns3{
namespace lorawan {

class EndDeviceStatus;

class LorawanGymEnv : public OpenGymEnv
{
  public:
    LorawanGymEnv();
    virtual ~LorawanGymEnv();
    static TypeId GetTypeId(void);
    virtual void DoDispose();

    // OpenGym interface
    virtual Ptr<OpenGymSpace> GetActionSpace();
    virtual bool GetGameOver();
    virtual float GetReward();
    virtual std::string GetExtraInfo();
    virtual bool ExecuteActions(Ptr<OpenGymDataContainer> action);

    virtual Ptr<OpenGymSpace> GetObservationSpace() = 0;
    virtual Ptr<OpenGymDataContainer> GetObservation() = 0;

    void SetDuration(Time value);
    void SetTimeStep(Time value);
    void SetReward(double value);
  private:
    void ScheduleNextStateRead();
    bool m_started {false};
    Time m_duration;
    Time m_timeStep;
    double m_reward;

    uint32_t m_uav_id;
    Vector3D m_uav_position;
    uint32_t m_packets_sent;
    uint32_t m_packets_received;
    double m_qos;

    uint32_t m_action_space = 4;
    bool m_isGameOver;
    float m_envReward;
    std::string m_info;
    uint32_t m_space;
    uint32_t m_action;

};

} //namespace lorawan
} //namespace ns3


#endif /* LORAWAN_RL_ENV_H */
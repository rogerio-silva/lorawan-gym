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

#include "ns3/constant-velocity-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/core-module.h"
#include "ns3/mobility-model.h"
#include "lorawan-rl-env.h"
#include "ns3/node-list.h"
#include "ns3/node-container.h"
#include "ns3/box.h"


#ifndef LORAWAN_RL_H
#define LORAWAN_RL_H

/*
 * ESTE CARA TEM QUE HERDAR DE MOBILITY MODEL (TALVEZ UMA MOB.MODEL ESPECÍFICA)
 * */

namespace ns3{

class MobilityModel;

class LorawanRl : public MobilityModel
{
  public:
    static TypeId GetTypeId ();
    void DoDispose() override;

    LorawanRl ();
    ~LorawanRl ();
    void GetGateways();
    void GetDevices();
    // From mobility model
    void FindNewPosition(uint32_t action);
    void DoMove(Vector position);
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;
    virtual std::string GetName () const;
  protected:
    virtual void CreateGymEnv();
    void ConnectTraceCallbacks();

    Ptr<LorawanGymEnv> m_lorawanGymEnv;
    Ptr<LorawanRl> m_model;

  private:
    Vector m_position;
    double m_movement_step = 1000.0; // step to a new position
    Box m_bounds; //!< bounding box
    double m_reward = 0.0;
    Time m_timeStep;
    NodeContainer m_gateways;
    NodeContainer m_devices;
    uint32_t m_number_of_devices = 0;
    uint32_t m_number_of_gateways = 0;
    bool m_simulation_time = false;

};

} /* namespace ns3 */
#endif /* LORAWAN_RL_H */
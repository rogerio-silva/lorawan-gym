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

#include "ns3/core-module.h"
#include "ns3/mobility-model.h"
#include "lorawan-rl-env.h"

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

    // From mobility model
    void Start();
    void DoMove();
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    virtual std::string GetName () const;

  private:
    Vector m_position;

};

} /* namespace ns3 */
#endif /* LORAWAN_RL_H */
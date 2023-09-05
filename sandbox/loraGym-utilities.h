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
#ifndef LORA_GYM_ENTITY_H
#define LORA_GYM_ENTITY_H



#include "ns3/position-allocator.h"
#include "ns3/mobility-model.h"
#include "ns3/double.h"
#include <fstream>
#include <cmath>

namespace ns3 {

Ptr<ListPositionAllocator> nodesPlacement (std::string filename);
double qos(double sf, double bandwidth, double packet_size);
std::vector<uint32_t> Serialize(Ptr<ListPositionAllocator> allocator);
Ptr<ListPositionAllocator> Deserialize(std::vector<uint32_t> positionVector);
uint32_t Linearize(Vector pos);
Vector Delinearize(uint32_t pos);

}
#endif
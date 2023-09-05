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

#include "loraGym-utilities.h"

namespace ns3
{

/**
 * Places the end devices according to the allocator object in the input file..
 * @param filename: arquivo de entrada
 * @return number of devices
 **/
Ptr<ListPositionAllocator>
nodesPlacement(std::string filename)
{
    double edX = 0.0;
    double edY = 0.0;
    double edZ = 0.0;
    //    int nDev = 0;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    const char* c = filename.c_str();
    // Get Devices position from File
    std::ifstream in_File(c);

    if (!in_File)
    {
        std::cout << "Could not open the file - '" << filename << "'" << std::endl;
    }
    else
    {
        while (in_File >> edX >> edY >> edZ)
        {
            allocator->Add(Vector(edX, edY, edZ));
            //            nDev++;
        }
        in_File.close();
    }

    return allocator;
}

double
qos(double sf, double bandwidth, double packet_size)
{
    double rk;
    double dk;
    rk = (sf * (bandwidth / (pow(2, sf)))) / MAX_RK;
    dk = (packet_size / rk) / (PACKET_SIZE / MIN_RK);
    return rk + (1 - dk);
}

/**
 * Serializes  ListPositionAllocator to a Vector
 **/
std::vector<uint32_t>
Serialize(Ptr<ListPositionAllocator> allocator)
{
    std::vector<uint32_t> positionVector;
    uint32_t nPos = allocator->GetSize();
    for (uint32_t i = 0; i < nPos; ++i)
    {
        positionVector.push_back(Linearize(allocator->GetNext()));
    }
    return positionVector;
}

/**
 * Deserializes  ListPositionAllocator to a Vector
 **/
Ptr<ListPositionAllocator>
Deserialize(std::vector<uint32_t> positionVector)
{
    Ptr<ListPositionAllocator> allocator;
    uint32_t nPos = positionVector.size();
    for (uint32_t i = 0; i < nPos; ++i)
    {
        allocator->Add(Delinearize(positionVector.at(i)));
    }
    return allocator;
}

/**
 * Linearize vector to a uint32 position
 **/
uint32_t
Linearize(Vector pos)
{
    return pos.y + (pos.x * H_POSITIONS);
}

/**
 * Delinearize uint32 to a Vector(x,y,z) position
 **/
Vector
Delinearize(uint32_t pos)
{
    return Vector(pos / H_POSITIONS, pos % H_POSITIONS, 30);
}





} // namespace ns3
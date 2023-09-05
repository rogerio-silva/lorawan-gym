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

#include "ns3/end-device-lora-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/lorawan-module.h"
#include "ns3/propagation-module.h"
#include <algorithm>
#include <iomanip>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("GatewaysDensityOrientedPlacement");

struct pos
{
  double v;
  int w;
  int h;
};

bool
pos_compare (const pos &p1, const pos &p2)
{
  return p1.v > p2.v;
}

int
main (int argc, char *argv[])
{
  int nGateways = 0;
  int nDevices = 0;
  int seed = 1;
  //  bool debug = false;

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("nGateways", "Number of gateways to include in the simulation", nGateways);
  cmd.AddValue ("seed", "Independent replications seed", seed);
  cmd.Parse (argc, argv);

  ns3::RngSeedManager::SetSeed (seed);

  //  LogComponentEnable ("DevicesDensityOrientedPlacement", LOG_LEVEL_ALL);
  //  LogComponentEnableAll (LOG_PREFIX_FUNC);
  //  LogComponentEnableAll (LOG_PREFIX_NODE);
  //  LogComponentEnableAll (LOG_PREFIX_TIME);
  //  LogComponentEnableAll (LOG_ERROR);

  NS_LOG_INFO ("Creating Gateways...");
  // Calculate density demand map
  // Generate the demand according to the following paper:
  // Dongheon Lee; Sheng Zhou; Xiaofeng Zhong; Zhisheng Niu; Xuan Zhou;
  // Honggang Zhang, "Spatial modeling of the traffic density in cellular
  // networks," in Wireless Communications, IEEE , vol.21, no.1, pp.80-88,
  // February 2014. doi: 10.1109/MWC.2014.6757900

  //  Some parameters
  //      %mu = 25.7015; % scaled
  //      mu = 18.93; % (DL-urban)
  //      %mu=17.7956; %(UL-urban), mu=12.572 (DL-rural), mu=11.573 (UL-rural)
  //      %sigma = 1.1929; % scaled
  //      %sigma = 2.3991; % (DL-urban)
  //      sigma=2.1188;% (UL-urban);%, sigma=2.7985 (DL-rural), sigma=2.3055 (UL-rural)
  //      %wMax = 0.001163; % test
  //      wMax = 0.011592; % (DL-urban)
  //      %wMax=0.012673;% (UL-urban);%, wMax=0.001163 (DL-rural), wMax=0.001202 (UL-rural)

  //  ** density is estimated against devices, not gateways **
  struct densityParams
  {
    double area_width = 10000; // Area width in meters
    double area_height = 10000; // Area height in meters
    double pixSideLen = 2000; // Cell height and width
    double mu = 17.7956;
    double sigma = 2.1188;
    double w_max = 0.012673;
  };

  densityParams dParams;
  double a, b;
  double angFreq_i[10], angFreq_j[10], phase_phi[10], phase_psi[10];

  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable> ();
  rd->SetAttribute ("Min", DoubleValue (0.0));
  rd->SetAttribute ("Max", DoubleValue (1.0));

  for (int i = 0; i < 10; ++i)
    {
      a = 0;
      b = dParams.w_max;
      angFreq_i[i] = a + (b - a) * rd->GetValue ();
      angFreq_j[i] = a + (b - a) * rd->GetValue ();
      a = 0;
      b = 2 * M_PI;
      phase_phi[i] = a + (b - a) * rd->GetValue ();
      phase_psi[i] = a + (b - a) * rd->GetValue ();
    }

  // Compute cell density
  int ppW = floor (dParams.area_width / dParams.pixSideLen);
  int ppH = floor (dParams.area_height / dParams.pixSideLen);
  double Xwh, Ywh, cum_pGwh = 0.0, totDens = 0.0;
  double pGwh[ppW][ppH];
  double p_wh[ppW][ppH];
  for (int w = 0; w < ppW; w++)
    {
      for (int h = 0; h < ppH; h++)
        {
          Xwh = w * dParams.pixSideLen - 0.5 * dParams.pixSideLen;
          Ywh = h * dParams.pixSideLen - 0.5 * dParams.pixSideLen;
          cum_pGwh = 0;
          for (int i = 0; i < 10; i++)
            {
              cum_pGwh = cum_pGwh + cos (angFreq_i[i] * Xwh + phase_phi[i]) *
                                        cos (angFreq_j[i] * Ywh + phase_psi[i]);
            }
          pGwh[w][h] = (2 / sqrt (10)) * cum_pGwh;
          p_wh[w][h] = exp (dParams.sigma * pGwh[w][h] + dParams.mu);
          totDens = totDens + p_wh[w][h];
        }
    }

  // Normalize cell values to nGateways
  double pN_wh[ppW][ppH]; // Percentage of devices per cell
  double nGat[ppW][ppH]; // Number of devices per cell
  std::vector<pos> vecPos;
  for (int w = 0; w < ppW; w++)
    {
      for (int h = 0; h < ppH; h++)
        {
          pN_wh[w][h] = (p_wh[w][h]) / totDens;
          nGat[w][h] = (pN_wh[w][h] * nGateways);
          vecPos.push_back ({nGat[w][h], w, h});
        }
    }

  // order the densest first
  std::sort (vecPos.begin (), vecPos.end (), pos_compare);
  int lastPos, nElements;
  for (lastPos = 0, nElements = 0; vecPos[lastPos].v != 0;
       lastPos++, nElements += vecPos[lastPos].v)
    ;

  // Put the gateways, one by one, in the respective cells
  NodeContainer gatewaysContainer;
  NodeContainer tmpGatewaysContainer[lastPos];
  MobilityHelper tmpGatewaysMobility[lastPos];
  int posFactor = dParams.area_height / ppH;
  double gatewayAltitude = 30;

  std::string path = "/home/rogerio/git/sim-res/datafile/density-oriented/placement/";
  std::string fileNS3 = path + "/densityOrientedPlacement_" + std::to_string (seed) + "s+" +
                        std::to_string (nDevices) + "d+" + std::to_string (nGateways) + "g.dat";
  const char *cN = fileNS3.c_str ();

  std::ofstream devicesNS3File;
  devicesNS3File.open (cN);

  int nGatLeft = nGateways;
  int newGat = 0;
  for (int x = 0; x < lastPos && nGatLeft > 0; x++)
    {
      //      if (vecPos[x].v >= 0.5)
      //        {
      newGat = vecPos[x].v > 4 ? 4 : vecPos[x].v;
      nGatLeft -= newGat;
      tmpGatewaysContainer[x].Create (newGat);
      tmpGatewaysMobility[x].SetPositionAllocator (
          "ns3::UniformDiscPositionAllocator", "rho", DoubleValue (posFactor / 2.0), "X",
          DoubleValue (vecPos[x].w * posFactor + (posFactor / 2.0)), "Y",
          DoubleValue (vecPos[x].h * posFactor + (posFactor / 2.0)));
      //          tmpGatewaysMobility[x].SetPositionAllocator (
      //              "ns3::RandomRectanglePositionAllocator", "X",
      //              PointerValue (CreateObjectWithAttributes<UniformRandomVariable> (
      //                  "Min", DoubleValue (vecPos[x].w * posFactor), "Max",
      //                  DoubleValue (vecPos[x].w * posFactor + posFactor))),
      //              "Y",
      //              PointerValue (CreateObjectWithAttributes<UniformRandomVariable> (
      //                  "Min", DoubleValue (vecPos[x].h * posFactor), "Max",
      //                  DoubleValue (vecPos[x].h * posFactor + posFactor))));
      tmpGatewaysMobility[x].SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      tmpGatewaysMobility[x].Install (tmpGatewaysContainer[x]);
      gatewaysContainer.Add (tmpGatewaysContainer[x]);
      //          devicesNS3File << posX << " " << posY << " " << gatewayAltitude << std::endl;
      //        }
    }

  for (NodeContainer::Iterator dv = gatewaysContainer.Begin (); dv != gatewaysContainer.End ();
       ++dv)
    {
      Ptr<MobilityModel> mobility2 = (*dv)->GetObject<MobilityModel> ();
      Vector position = mobility2->GetPosition ();
      position.z = gatewayAltitude;
      mobility2->SetPosition (position);
      devicesNS3File << position.x << " " << position.y << " " << position.z << std::endl;
    }

  devicesNS3File.close ();

  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop ();

  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}

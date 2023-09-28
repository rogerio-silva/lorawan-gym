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

#include "ns3/end-device-status.h"
#include "ns3/opengym-module.h"

namespace ns3
{

class LorawanGymEnv : public OpenGymEnv
{
  public:
    enum PacketOutcome
    {
        _RECEIVED,
        _INTERFERED,
        _NO_MORE_RECEIVERS,
        _UNDER_SENSITIVITY,
        _UNSET
    };

    struct myPacketStatus
    {
        Ptr<const Packet> packet;
        uint32_t senderId;
        uint32_t receiverId;
        Time sentTime;
        Time receivedTime;
        uint8_t senderSF;
        uint8_t receiverSF;
        double senderTP;
        double receiverTP;
        uint32_t outcomeNumber;
        std::vector<enum PacketOutcome> outcomes;
    };

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
    Ptr<OpenGymSpace> GetObservationSpace();
    Ptr<OpenGymDataContainer> GetObservation();

    void SetDuration(Time duration);
    void SetUAVMovementStep(double step);
    void SetReward(double reward);

    /**
     * Tracing
     * */
    virtual void PktTrace(Ptr<const Packet> packet, uint32_t systemId);
    virtual void CheckReceptionByAllGWsComplete(
        std::map<Ptr<const Packet>, myPacketStatus>::iterator it);
    virtual void TransmissionCallback(Ptr<const Packet> packet,
                                      uint32_t systemId,
                                      Ptr<LorawanGymEnv> env);
    virtual void PacketReceptionCallback(Ptr<const Packet> packet,
                                         uint32_t systemId,
                                         Ptr<LorawanGymEnv> env);
    virtual void InterferenceCallback(Ptr<const Packet> packet,
                                      uint32_t systemId,
                                      Ptr<LorawanGymEnv> env);
    virtual void NoMoreReceiversCallback(Ptr<const Packet> packet,
                                         uint32_t systemId,
                                         Ptr<LorawanGymEnv> env);
    virtual void UnderSensitivityCallback(Ptr<const Packet> packet,
                                          uint32_t systemId,
                                          Ptr<LorawanGymEnv> env);
    virtual void RecalcularTudo(Ptr<LorawanGymEnv> env, Ptr<const MobilityModel> mob);

    void setMNumberOfDevices(uint32_t mNumberOfDevices);
    void setMNumberOfGateways(uint32_t mNumberOfGateways);
    void setMPktInterfered(int mPktInterfered);
    void setMPktReceived(int mPktReceived);
    void setMPktUnderSensitivity(int mPktUnderSensitivity);
    void setMPktTransmitted(int mPktTransmitted);
    void setMPktNoMoreReceivers(int mPktNoMoreReceivers);
    static const std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus>::iterator
    getMPacketTrackerIterator(Ptr<LorawanGymEnv> env, Ptr<const Packet> packet);
    static const std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus>::iterator
    getMPacketTrackerIteratorBegin(Ptr<LorawanGymEnv> env);
    static const std::map<Ptr<const Packet>, LorawanGymEnv::myPacketStatus>::iterator
    getMPacketTrackerIteratorEnd(Ptr<LorawanGymEnv> env);

    static void setMPacketTracker(std::pair<Ptr<const Packet>, myPacketStatus> pair,
                                  Ptr<LorawanGymEnv> env);

    /**
     * Packet Tracing
     * */
    int m_pkt_interfered = 0;
    int m_pkt_received = 0;
    int m_pkt_underSensitivity = 0;
    int m_pkt_transmitted = 0;
    int m_pkt_noMoreReceivers = 0;
    std::map<Ptr<const Packet>, myPacketStatus> m_packetTracker;

  protected:
    void ScheduleNextStateRead();

    bool m_started{false};
    Time m_duration = Seconds(0);
    double m_UAV_MovementStep = 1000.0;
    double m_reward = 0.0;

    double m_qos = 0.0;

    uint32_t m_uav_id = 0;
    Vector3D m_uav_position = {0, 0, 0};

    uint32_t m_action_space = 4;
    bool m_isGameOver = false;
    float m_envReward = 0.0;
    std::string m_info = "none";
    uint32_t m_space = 0;
    uint32_t m_action = 0;
    uint32_t m_number_of_devices = 10;
    uint32_t m_number_of_gateways = 1;
};

} // namespace ns3

#endif /* LORAWAN_RL_ENV_H */
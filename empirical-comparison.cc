/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
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
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

#include "ns3/double.h"
#include "ns3/gnuplot.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Lab4Part1");

class NodeStatistics
{
public:
  NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas);

  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void SetPosition (Ptr<Node> node, Vector position);
  void AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime);
  Vector GetPosition (Ptr<Node> node);

private:
  uint32_t m_bytesTotal;
};

NodeStatistics::NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas)
{
  m_bytesTotal = 0;
}

void
NodeStatistics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize ();
}

void
NodeStatistics::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

Vector
NodeStatistics::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
NodeStatistics::AdvancePosition (Ptr<Node> node, int stepsTime, int idx)
{
  double x[] = { 15, 20, 25, 30, 35, 35, 35, 40, 45, 50, 55};
  double y[] = {  0,  0,  0,  0,  0,  5, 10, 10, 10, 10, 10};
  
  Vector pos = GetPosition (node);
  double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
  printf("%g\t(%g, %g, %g)\t%g\t%g\n", Simulator::Now().GetSeconds(), pos.x, pos.y, pos.z, CalculateDistance(pos, Vector3D(0.0,0.0,3.0)), mbs);
  if (idx < 11){
    pos.x = x[idx];
    pos.y = y[idx];
    pos.z = pos.z;
    SetPosition (node, pos);
    m_bytesTotal = 0;
    idx++;
    Simulator::Schedule (Seconds (stepsTime), &NodeStatistics::AdvancePosition, this, node, stepsTime, idx);
  }
}

void
RateChange (uint64_t oldVal, uint64_t newVal)
{
  cout << (Simulator::Now ()).GetSeconds () << ": Rate change from " << oldVal << " to " << newVal << endl;
}

static void
RxDrop (std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason){
  cout << (Simulator::Now ()).GetSeconds () << ": Packet dropped" << endl;
}

int main (int argc, char *argv[])
{
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 10;
  int sta1_y = 0;
  int steps = 12;
  int stepsTime = 1;
  std::string propagation = "Nakagami";

  LogComponentEnable ("Lab4Part1", LOG_LEVEL_INFO);

  CommandLine cmd (__FILE__);
  cmd.AddValue ("propagation", "Propagation loss model", propagation);
  cmd.Parse (argc, argv);

  int simuTime = steps * stepsTime;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (1);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  if (propagation == "Nakagami"){
    wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel", "Distance1", DoubleValue (35.0), "Distance2", DoubleValue (40.0), "m1", DoubleValue (0.25), "m2", DoubleValue (0.11));
  } else if (propagation == "TwoRayGround") {
    wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel");
  }
  wifiPhy.SetChannel (wifiChannel.Create ());

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211ac);
  WifiMacHelper wifiMac;

  //Configure the STA node
  wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");
  Ssid ssid = Ssid ("AP");
  wifiMac.SetType ("ns3::StaWifiMac",
                    "Ssid", SsidValue (ssid));
  wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get (0)));

  //Configure the AP node
  wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");
  ssid = Ssid ("AP");
  wifiMac.SetType ("ns3::ApWifiMac",
                    "Ssid", SsidValue (ssid));
  wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));

  wifiDevices.Add (wifiStaDevices);
  wifiDevices.Add (wifiApDevices);

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //Initial position of AP and STA
  positionAlloc->Add (Vector (ap1_x, ap1_y, 3.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 1.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes.Get (0));

  //Statistics counter
  NodeStatistics atpCounter = NodeStatistics (wifiApDevices, wifiStaDevices);

  //Move the STA
  printf("Time\tPos. (x, y, z)\tDist.\tGoodput\n");
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &atpCounter, wifiStaNodes.Get (0), stepsTime, 0);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  Ipv4Address sinkAddress = i.GetAddress (0);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("400Mb/s"), 1024);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (0.5 + simuTime)));
  ApplicationContainer apps_source = onoff.Install (wifiApNodes.Get (0));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (0.5 + simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Register packet receptions to calculate throughput
  Config::Connect ("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&NodeStatistics::RxCallback, &atpCounter));

  //Generate output each time a packet is dropped
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                   MakeCallback (&RxDrop));

  //Callbacks to print every change of rate
  //Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::MinstrelHtWifiManager/Rate",
  //                               MakeCallback (&RateChange));

  Simulator::Stop (Seconds (1.0 + simuTime));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

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

#include "ns3/ecc33-propagation-loss-model.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("EmpiricalPropagationLossComparison");

void
AdvancePosition (Ptr<Node> tx, Ptr<Node> rx, Ptr<PropagationLossModel> friis, Ptr<PropagationLossModel> ecc33, double stepsSize, double stepsTime)
{
  Ptr<MobilityModel> tx_mobility = tx->GetObject<MobilityModel> ();
  Ptr<MobilityModel> rx_mobility = rx->GetObject<MobilityModel> ();
  Vector pos = rx_mobility->GetPosition ();
  cout << "Distance: " << CalculateDistance(pos, tx_mobility->GetPosition()) << " m" << endl;
  cout << "Model: friis, Path loss: " << 47 - friis->CalcRxPower (47, tx_mobility, rx_mobility) << " dB" << endl;
  {
    cout << "Model: ecc33, Path loss: " << 47 - ecc33->CalcRxPower (47, tx_mobility, rx_mobility) << " dB" << endl;
  }
  pos.x += stepsSize;
  rx_mobility->SetPosition (pos);
  Simulator::Schedule (Seconds (stepsTime), &AdvancePosition, tx, rx, friis, ecc33, stepsSize, stepsTime);
}

int main (int argc, char *argv[])
{
  double ap1_z = 33.0;
  double sta1_z = 1.0;
  int steps = 75;
  double stepsSize = 10.0;
  double stepsTime = 1.0;
  double frequency = 900e6; 
  string env = "urban";

  CommandLine cmd (__FILE__);
  cmd.AddValue ("steps", "How many different distances to try", steps);
  cmd.AddValue ("stepsTime", "Time on each step", stepsTime);
  cmd.AddValue ("stepsSize", "Distance between steps", stepsSize);
  cmd.AddValue ("environment", "Environment type", env);
  cmd.Parse (argc, argv);

  if(env == "urban"){
    ap1_z = 33.0;
  } else if (env == "suburban") {
    ap1_z = 35.0;
  } else if (env == "rural") {
    ap1_z = 42.0;
  } else {
    cout << "Invaid environment type" << endl;
    return 1;
  }

  double simuTime = steps * stepsTime;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (1);

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //Initial position of AP and STA
  positionAlloc->Add (Vector (0.0, 0.0, ap1_z));
  positionAlloc->Add (Vector (80.0, 0.0, sta1_z));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes.Get (0));

  //Propagation Loss Models
  // Friis/Free space
  Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel> ();
  friis->SetFrequency (frequency);

  // Hata

  // Cost 231

  // ECC33
  Ptr<ECC33PropagationLossModel> ecc33 = CreateObject<ECC33PropagationLossModel> ();
  ecc33->SetTxAntennaHeight(ap1_z);
  ecc33->SetRxAntennaHeight(sta1_z);
  ecc33->SetFrequency(frequency);
  if(env == "urban"){
    ecc33->SetEnvironment(ns3::ECC33PropagationLossModel::Urban);
  } else if (env == "suburban") {
    ecc33->SetEnvironment(ns3::ECC33PropagationLossModel::Suburban);
  }

  Simulator::Schedule (Seconds (0.5 + stepsTime), &AdvancePosition, wifiApNodes.Get (0), wifiStaNodes.Get (0), friis, ecc33, stepsSize, stepsTime);

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
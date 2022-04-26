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
#include <ns3/double.h>
#include <ns3/enum.h>
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
#include <vector>
#include <iostream>
#include <fstream>

#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/cost231-propagation-loss-model.h"
#include "ns3/ecc33-propagation-loss-model.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("EmpiricalPropagationLossComparison");

void
AdvancePosition (Ptr<Node> tx, Ptr<Node> rx, vector<Ptr<PropagationLossModel>>  models, double stepsSize, double stepsTime, vector<Gnuplot2dDataset> output)
{
  Ptr<MobilityModel> tx_mobility = tx->GetObject<MobilityModel> ();
  Ptr<MobilityModel> rx_mobility = rx->GetObject<MobilityModel> ();
  Vector pos = rx_mobility->GetPosition ();
  double dist = CalculateDistance(pos, tx_mobility->GetPosition());
  // cout << dist;
  for(uint16_t i = 0; i < models.size(); i++){
    // cout << ", " << 47 - models.at(i)->CalcRxPower (47, tx_mobility, rx_mobility);
    output.at(i).Add(dist, 47 - models.at(i)->CalcRxPower (47, tx_mobility, rx_mobility));
  }
  // cout << endl;
  pos.x += stepsSize;
  rx_mobility->SetPosition (pos);
  Simulator::Schedule (Seconds (stepsTime), &AdvancePosition, tx, rx, models, stepsSize, stepsTime, output);
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

  // LogComponentEnable ("ECC33PropagationLossModel", LOG_LEVEL_INFO);

  if(env == "urban"){
    ap1_z = 33.0;
  } else if (env == "suburban") {
    ap1_z = 35.0;
  } else if (env == "rural") {
    ap1_z = 42.0;
  } else {
    cout << "Invaid environment type. Please enter 'urban', 'suburban', or 'rural.'" << endl;
    return 1;
  }

  double simuTime = steps * stepsTime;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  // Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (1);

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  // Initial position of AP and STA
  positionAlloc->Add (Vector (0.0, 0.0, ap1_z));
  positionAlloc->Add (Vector (80.0, 0.0, sta1_z));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes.Get (0));


  // Propagation Loss Models
  vector<Ptr<PropagationLossModel>>  models;
  vector<Gnuplot2dDataset> output;

  // Friis/Free space
  Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel> ();
  friis->SetFrequency (frequency);
  models.push_back(friis);
  output.push_back(Gnuplot2dDataset("Friis"));

  // Ericsson

  // Hata
  Ptr<OkumuraHataPropagationLossModel> hata = CreateObject<OkumuraHataPropagationLossModel> ();
  hata->SetAttribute ("Frequency", DoubleValue(frequency));
  if(env == "urban"){
    hata->SetAttribute ("Environment", EnumValue(ns3::UrbanEnvironment));
    hata->SetAttribute ("CitySize", EnumValue(ns3::LargeCity));
  } else if (env == "suburban") {
    hata->SetAttribute ("Environment", EnumValue(ns3::SubUrbanEnvironment));
    hata->SetAttribute ("CitySize", EnumValue(ns3::MediumCity));
  } else if (env == "rural"){
    hata->SetAttribute ("Environment", EnumValue(ns3::OpenAreasEnvironment));
    hata->SetAttribute ("CitySize", EnumValue(ns3::SmallCity));
  }
  models.push_back(hata);
  output.push_back(Gnuplot2dDataset("Hata"));

  // Cost-231 Hata
  Ptr<Cost231PropagationLossModel> cost231 = CreateObject<Cost231PropagationLossModel> ();
  cost231->SetLambda (3e8/frequency);
  cost231->SetBSAntennaHeight(ap1_z);
  cost231->SetSSAntennaHeight(sta1_z);
  if(env == "urban"){
    cost231->SetShadowing(3);
  } else {
    cost231->SetShadowing(0);
  }
  models.push_back(cost231);
  output.push_back(Gnuplot2dDataset("Cost-231"));

  // SUI

  // ECC-33
  Ptr<ECC33PropagationLossModel> ecc33 = CreateObject<ECC33PropagationLossModel> ();
  ecc33->SetTxAntennaHeight(ap1_z);
  ecc33->SetRxAntennaHeight(sta1_z);
  ecc33->SetFrequency(frequency);
  if(env == "urban"){
    ecc33->SetEnvironment(ns3::ECC33PropagationLossModel::Urban);
  } else if (env == "suburban") {
    ecc33->SetEnvironment(ns3::ECC33PropagationLossModel::Suburban);
  }
  if (env != "rural"){
    models.push_back(ecc33);
    output.push_back(Gnuplot2dDataset("ECC-33"));
  }


  Simulator::Schedule (Seconds (0.5 + stepsTime), &AdvancePosition, wifiApNodes.Get (0), wifiStaNodes.Get (0), models, stepsSize, stepsTime, output);

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  std::ofstream outfile ("propagation-loss-" + env + ".plt");
  Gnuplot gnuplot = Gnuplot ("propagation-loss-" + env + ".png");
  gnuplot.SetTerminal ("png");
  gnuplot.SetLegend ("Distance (m)", "Propagation Loss (dB)");
  env[0] = toupper(env[0]);
  gnuplot.SetTitle (env + " Propagation Loss vs Distance");
  for(uint16_t i = 0; i < output.size(); i++){
    gnuplot.AddDataset (output.at(i));
  }
  gnuplot.GenerateOutput (outfile);

  Simulator::Destroy ();

  return 0;
}
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
#include "ns3/ericsson-propagation-loss-model.h"
#include "ns3/sui-propagation-loss-model.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("EmpiricalPropagationLossComparison");

double urban[20][2] = { {80, 0}, {100, 0}, {150, 0}, {200, 0}, {250, 0}, {300, 0}, {350, 0}, {400, 0}, {450, 0}, {500, 0}, {550, 0}, {600, 0}, {650, 0}, {700, 0}, {750, 0}, {790, 0}, {790, 50}, {790, 100}, {790, 150}, {790, 150} };
double suburban[20][2] = { {56.57, 56.57}, {100, 100}, {125, 125}, {150, 150}, {150, 200}, {150, 250}, {150, 300}, {200, 300}, {250, 300},  {300, 300}, {300, 350}, {300, 400}, {350, 400}, {400, 400}, {400, 450},  {400, 500}, {400, 550}, {400, 600}, {400, 650}, {400, 700} };
double rural[20][2] = { {0, 80}, {0, 125}, {50, 125}, {60, 150}, {70, 200}, {100, 230}, {150, 250}, {200, 300}, {225, 315}, {250, 330}, {275, 335}, {300, 340}, {350, 370}, {400, 380}, {450, 390}, {500, 400}, {550, 400}, {600, 400}, {650, 400}, {700, 400} };

void
AdvancePosition (Ptr<Node> tx, Ptr<Node> rx, vector<Ptr<PropagationLossModel>>  models, int idx, double path[20][2], vector<Gnuplot2dDataset> output)
{
  Ptr<MobilityModel> tx_mobility = tx->GetObject<MobilityModel> ();
  Ptr<MobilityModel> rx_mobility = rx->GetObject<MobilityModel> ();
  Vector pos = rx_mobility->GetPosition ();
  double dist = CalculateDistance(pos, tx_mobility->GetPosition());
  for(uint16_t i = 0; i < models.size(); i++){
    output.at(i).Add(dist, 47 - models.at(i)->CalcRxPower (47, tx_mobility, rx_mobility));
    //output.at(i).Add(pos.x, pos.y, 47 - models.at(i)->CalcRxPower (47, tx_mobility, rx_mobility));
  }
  pos.x = path[idx][0];
  pos.y = path[idx][1];
  rx_mobility->SetPosition (pos);
  Simulator::Schedule (Seconds (1), &AdvancePosition, tx, rx, models, idx+1, path, output);
}

int main (int argc, char *argv[])
{
  double ap1_z = 33.0;
  double sta1_z = 1.0;
  double frequency = 900e6; 
  string env = "urban";
  double path[20][2]; 

  CommandLine cmd (__FILE__);
  cmd.AddValue ("environment", "Environment type", env);
  cmd.Parse (argc, argv);

  if(env == "urban"){
    ap1_z = 33.0;
    for(int i=0; i < 20; i++){
      path[i][0] = urban[i][0];
      path[i][1] = urban[i][1];
    }
  } else if (env == "suburban") {
    ap1_z = 35.0;
    for(int i=0; i < 20; i++){
      path[i][0] = suburban[i][0];
      path[i][1] = suburban[i][1];
    }
  } else if (env == "rural") {
    ap1_z = 42.0;
    for(int i=0; i < 20; i++){
      path[i][0] = rural[i][0];
      path[i][1] = rural[i][1];
    }
  } else {
    cout << "Invaid environment type. Please enter 'urban', 'suburban', or 'rural.'" << endl;
    return 1;
  }

  double simuTime = 20;

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
  Ptr<EricssonPropagationLossModel> ericsson = CreateObject<EricssonPropagationLossModel> ();
  ericsson->SetFrequency(frequency);
  ericsson->SetTxAntennaHeight(ap1_z);
  ericsson->SetRxAntennaHeight(sta1_z);
  if(env == "urban"){
    ericsson->SetEnvironment(ns3::EricssonPropagationLossModel::Urban);
  } else if (env == "suburban") {
    ericsson->SetEnvironment(ns3::EricssonPropagationLossModel::Suburban);
  } else if (env == "rural") {
    ericsson->SetEnvironment(ns3::EricssonPropagationLossModel::Rural);
  }
  models.push_back(ericsson);
  output.push_back(Gnuplot2dDataset("Ericsson"));

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
  Ptr<SUIPropagationLossModel> sui = CreateObject<SUIPropagationLossModel> ();
  sui->SetLambda(3e8/frequency);
  sui->SetBSAntennaHeight(ap1_z);
  sui->SetSSAntennaHeight(sta1_z);
  if(env == "urban"){
    sui->SetTerrain(ns3::SUIPropagationLossModel::A);
  } else if (env == "suburban"){
    sui->SetTerrain(ns3::SUIPropagationLossModel::B);
  } else if (env == "rural"){
    sui->SetTerrain(ns3::SUIPropagationLossModel::C);
  }
  models.push_back(sui);
  output.push_back(Gnuplot2dDataset("SUI"));

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


  Simulator::Schedule (Seconds (0.5 + 1), &AdvancePosition, wifiApNodes.Get (0), wifiStaNodes.Get (0), models, 0, path, output);

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  std::ofstream outfile ("propagation-loss-" + env + ".plt");
  Gnuplot gnuplot = Gnuplot ("propagation-loss-" + env + ".png");
  gnuplot.SetTerminal ("png");
  gnuplot.SetLegend ("Distance (m)", "Propagation Loss (dB)");
  env[0] = toupper(env[0]);
  gnuplot.SetTitle (env + " Propagation Loss vs Path");
  // gnuplot.AppendExtra ("set xlabel 'X Pos (m)' offset 0,0,-5");
  // gnuplot.AppendExtra ("set ylabel 'Y Pos (m)' offset 0,0,0");
  gnuplot.AppendExtra ("set zlabel 'Path loss (dB)' offset 0,0,0");
  gnuplot.AppendExtra ("set xrange [100:+800]");
  gnuplot.AppendExtra ("set yrange [20:+200]");
  //gnuplot.AppendExtra ("set zrange [0:+200]");
  //gnuplot.AppendExtra ("set ticslevel 0");

  for(uint16_t i = 0; i < output.size(); i++){
    //output.at(i).SetStyle("with boxes");
    output.at(i).SetStyle(ns3::Gnuplot2dDataset::Style::LINES);
    gnuplot.AddDataset (output.at(i));
  }
  gnuplot.GenerateOutput (outfile);

  Simulator::Destroy ();

  return 0;
}
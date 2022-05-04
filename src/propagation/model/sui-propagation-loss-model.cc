/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007,2008, 2009 INRIA, UDcast
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
 * Author: Mohamed Amine Ismail <amine.ismail@sophia.inria.fr>
 *                              <amine.ismail@udcast.com>
 */

#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include <cmath>
#include "sui-propagation-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SUIPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (SUIPropagationLossModel);

TypeId
SUIPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SUIPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<SUIPropagationLossModel> ()
    .AddAttribute ("Lambda",
                   "The wavelength  (default is 2.5 GHz at 300 000 km/s).",
                   DoubleValue (300000000.0 / 2.5e9),
                   MakeDoubleAccessor (&SUIPropagationLossModel::m_lambda),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Frequency",
                   "The Frequency  (default is 2.3 GHz).",
                   DoubleValue (2.3e9),
                   MakeDoubleAccessor (&SUIPropagationLossModel::m_frequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BSAntennaHeight",
                   "BS Antenna Height (default is 50m).",
                   DoubleValue (50.0),
                   MakeDoubleAccessor (&SUIPropagationLossModel::m_BSAntennaHeight),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SSAntennaHeight",
                   "SS Antenna Height (default is 3m).",
                   DoubleValue (3),
                   MakeDoubleAccessor (&SUIPropagationLossModel::m_SSAntennaHeight),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Terrain",
                   "Type of terrain (default is A) ",
                   EnumValue (A),
                   MakeEnumAccessor (&SUIPropagationLossModel::m_terrain),
                   MakeEnumChecker (A, "A", B, "B", C, "C"));
  return tid;
}

SUIPropagationLossModel::SUIPropagationLossModel ()
{
  m_shadowing = 10;
}

void
SUIPropagationLossModel::SetLambda (double frequency, double speed)
{
  m_lambda = speed / frequency;
  m_frequency = frequency;
}

double
SUIPropagationLossModel::GetShadowing (void)
{
  return m_shadowing;
}
void
SUIPropagationLossModel::SetShadowing (double shadowing)
{
  m_shadowing = shadowing;
}

void
SUIPropagationLossModel::SetLambda (double lambda)
{
  m_lambda = lambda;
  m_frequency = 300000000 / lambda;
}

double
SUIPropagationLossModel::GetLambda (void) const
{
  return m_lambda;
}

void
SUIPropagationLossModel::SetBSAntennaHeight (double height)
{
  m_BSAntennaHeight = height;
}

double
SUIPropagationLossModel::GetBSAntennaHeight (void) const
{
  return m_BSAntennaHeight;
}

void
SUIPropagationLossModel::SetSSAntennaHeight (double height)
{
  m_SSAntennaHeight = height;
}

double
SUIPropagationLossModel::GetSSAntennaHeight (void) const
{
  return m_SSAntennaHeight;
}

void
SUIPropagationLossModel::SetTerrain (Terrain terrain)
{
  m_terrain = terrain;
}

SUIPropagationLossModel::Terrain
SUIPropagationLossModel::GetTerrain (void) const
{
  return m_terrain;
}

double
SUIPropagationLossModel::GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{

  double distance = a->GetDistanceFrom (b);

  double frequency_MHz = m_frequency * 1e-6;

  double d0 = 100.0;

  double Xf = 6.0 * std::log10(frequency_MHz / 2000.0);
  double Xh = -10.8 * std::log10(m_SSAntennaHeight / 2000.0);
  double modparam_a = 4.6, modparam_b = 0.0075, modparam_c = 12.6;
  if (m_terrain == A) {
    modparam_a = 4.6;
    modparam_b = 0.0075;
    modparam_c = 12.6;
    Xh = -10.8 * std::log10(m_SSAntennaHeight / 2000.0);
  } else if (m_terrain == B) {
    modparam_a = 4.0;
    modparam_b = 0.0065;
    modparam_c = 17.1;
    Xh = -10.8 * std::log10(m_SSAntennaHeight / 2000.0);
  } else if (m_terrain == C) {
    modparam_a = 3.6;
    modparam_b = 0.005;
    modparam_c = 20.0;
    Xh = -20.0 * std::log10(m_SSAntennaHeight / 20000.0);
  }

  double param_A = 20 * log10(4 * M_PI * d0 / m_lambda);

  double gamma = modparam_a - modparam_b * m_BSAntennaHeight + (modparam_c / m_BSAntennaHeight);

  double loss_in_db = param_A + 10 * gamma * std::log10(distance / d0) + Xf + Xh + m_shadowing;

  NS_LOG_DEBUG ("dist =" << distance << ", Path Loss = " << loss_in_db);

  return (0 - loss_in_db);

}

double
SUIPropagationLossModel::DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
  return txPowerDbm + GetLoss (a, b);
}

int64_t
SUIPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

}

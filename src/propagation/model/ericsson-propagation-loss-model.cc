/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include <cmath>
#include "ericsson-propagation-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("EricssonPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (EricssonPropagationLossModel);

TypeId
EricssonPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EricssonPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<EricssonPropagationLossModel> ()
    .AddAttribute ("Frequency",
                   "The Frequency  (default is 2 GHz).",
                   DoubleValue (2e9),
                   MakeDoubleAccessor (&EricssonPropagationLossModel::m_frequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxAntennaHeight",
                   "TX Antenna Height (default is 50m).",
                   DoubleValue (50.0),
                   MakeDoubleAccessor (&EricssonPropagationLossModel::m_TxAntennaHeight),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxAntennaHeight",
                   "RX Antenna Height (default is 3m).",
                   DoubleValue (3),
                   MakeDoubleAccessor (&EricssonPropagationLossModel::m_RxAntennaHeight),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Environment",
                   "Type of environment (default is urban) ",
                   EnumValue (Urban),
                   MakeEnumAccessor (&EricssonPropagationLossModel::m_environment),
                   MakeEnumChecker (Urban, "Urban", Suburban, "Suburban", Rural, "Rural"));
  return tid;
}

EricssonPropagationLossModel::EricssonPropagationLossModel ()
{
}

void
EricssonPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
}

double
EricssonPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

void
EricssonPropagationLossModel::SetTxAntennaHeight (double height)
{
  m_TxAntennaHeight = height;
}

double
EricssonPropagationLossModel::GetTxAntennaHeight (void) const
{
  return m_TxAntennaHeight;
}

void
EricssonPropagationLossModel::SetRxAntennaHeight (double height)
{
  m_RxAntennaHeight = height;
}

double
EricssonPropagationLossModel::GetRxAntennaHeight (void) const
{
  return m_RxAntennaHeight;
}

void
EricssonPropagationLossModel::SetEnvironment (Environment environment)
{
  m_environment = environment;
}

EricssonPropagationLossModel::Environment
EricssonPropagationLossModel::GetEnvironment (void) const
{
  return m_environment;
}

double
EricssonPropagationLossModel::GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{

  double distance = (a->GetDistanceFrom (b)) / 1e3; // distance in km
  double frequency = m_frequency / 1e6;            //frequency in MHz

  double g_f = 44.49*log10(frequency) - 4.78*pow(log10(frequency), 2);
  double a0;
  double a1;
  double a2 = 12;
  double a3 = 0.1;
  if(m_environment == Urban){
    a0 = 36.2;
    a1 = 30.2;
  } else if (m_environment == Suburban){
    a0 = 43.2;
    a1 = 68.93;
  } else if (m_environment == Rural){
    a0 = 45.95;
    a1 = 100.6;
  }

  double loss_in_db = a0 + a1*log10(distance) + a2*log(m_RxAntennaHeight) + a3*log10(m_TxAntennaHeight)*log10(distance) - 3.2*pow(log10(11.75*m_RxAntennaHeight), 2) + g_f;

  NS_LOG_DEBUG ("dist =" << distance << ", Path Loss = " << loss_in_db << ", g_f = " << g_f << ", a0 = " << a0 << ", a1 = " << a1 << ", a2 = " << a2 << ", a3 = " << a3);

  return (0 - loss_in_db);

}

double
EricssonPropagationLossModel::DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
  return txPowerDbm + GetLoss (a, b);
}

int64_t
EricssonPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

}

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

 /*
  * ECC-33 Propagation Path Loss Model
  * A_fs - free space attenuation (dB)
  * A_bm - basic medium path loss (dB)
  * G_b  - base station/transmitter antenna height gain factor
  * G_r  - receiver antenna height gain factor
  *  L   - path loss (dB)
  *  f   - frequency (GhZ)
  *  d   - distance between base station (transmitter) and mobile antenna (receiver) (km)
  * h_b  - base station (transmitter) height (m)
  * h_r  - mobile antenna (receiver) height (m)
  *
  *  L   = A_fs + A_bm - G_b - G_r
  * A_fs = 92.4 + 20log(d) + log(f)
  * A_bm = 20.41 + 9.83log(d) + 7.894log(f) + 9.56[log(f)]^2
  * G_b  = log(h_b/200)(13.958 + 5.8log(d))^2
  * G_r suburban = [42.57 + 13.7log(f)][log(h_r)-0.585]
  * G_r urban    = 0.759h_r - 1.862
  */

#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include <cmath>
#include "ecc33-propagation-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ECC33PropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (ECC33PropagationLossModel);

TypeId
ECC33PropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ECC33PropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ECC33PropagationLossModel> ()
    .AddAttribute ("Frequency",
                   "The Frequency  (default is 2 GHz).",
                   DoubleValue (2e9),
                   MakeDoubleAccessor (&ECC33PropagationLossModel::m_frequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxAntennaHeight",
                   "Tx Antenna Height (default is 50m).",
                   DoubleValue (50.0),
                   MakeDoubleAccessor (&ECC33PropagationLossModel::m_TxAntennaHeight),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxAntennaHeight",
                   "Rx Antenna Height (default is 3m).",
                   DoubleValue (3),
                   MakeDoubleAccessor (&ECC33PropagationLossModel::m_RxAntennaHeight),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Environment",
                   "Type of environment (default is urban) ",
                   EnumValue (Urban),
                   MakeEnumAccessor (&ECC33PropagationLossModel::m_environment),
                   MakeEnumChecker (Urban, "Urban", Suburban, "Suburban"));
  return tid;
}

ECC33PropagationLossModel::ECC33PropagationLossModel ()
{
}

void
ECC33PropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
}

double
ECC33PropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

void
ECC33PropagationLossModel::SetTxAntennaHeight (double height)
{
  m_TxAntennaHeight = height;
}

double
ECC33PropagationLossModel::GetTxAntennaHeight (void) const
{
  return m_TxAntennaHeight;
}

void
ECC33PropagationLossModel::SetRxAntennaHeight (double height)
{
  m_RxAntennaHeight = height;
}

double
ECC33PropagationLossModel::GetRxAntennaHeight (void) const
{
  return m_RxAntennaHeight;
}

void
ECC33PropagationLossModel::SetEnvironment (Environment environment)
{
  m_environment = environment;
}

ECC33PropagationLossModel::Environment
ECC33PropagationLossModel::GetEnvironment (void) const
{
  return m_environment;
}

double
ECC33PropagationLossModel::GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
  double distance = a->GetDistanceFrom (b);
  distance = distance / 1000;
  double frequency = m_frequency/1e9;

  double A_fs = 92.4 + 20*std::log10(distance) + 20*std::log10(frequency);
  double A_bm = 20.41 + 9.83*std::log10(distance) + 7.89*std::log10(frequency) + 9.56*pow( std::log10(frequency), 2);
  double G_b = std::log10(m_TxAntennaHeight/200)*pow( (13.958 + 5.8*std::log10(distance)), 2);
  double G_r = 0.0;
  if (m_environment == Urban){
    G_r = 0.759*m_RxAntennaHeight - 1.862;
  } else {
    G_r = (42.57 + 13.7*std::log10(frequency))*(std::log10(m_RxAntennaHeight) - 0.585);
  }

  double loss_in_db = A_fs + A_bm - G_b - G_r;

  NS_LOG_DEBUG ("dist =" << distance << ", Path Loss = " << loss_in_db);

  return (0 - loss_in_db);
}

double
ECC33PropagationLossModel::DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
  return txPowerDbm + GetLoss (a, b);
}

int64_t
ECC33PropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

}
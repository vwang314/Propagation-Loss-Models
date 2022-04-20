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

#ifndef ECC33_PROPAGATION_LOSS_MODEL_H
#define ECC33_PROPAGATION_LOSS_MODEL_H

#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"

namespace ns3 {

class ECC33PropagationLossModel : public PropagationLossModel
{

public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  ECC33PropagationLossModel ();
  enum Environment
  {
    Suburban, Urban
  };

  /**
   * Get the propagation loss
   * \param a the mobility model of the source
   * \param b the mobility model of the destination
   * \returns the propagation loss (in dBm)
   */
  double GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;

  /**
   * Set the frequency
   * \param frequency frequency [Hz]
   */
  void SetFrequency (double frequency);
  /**
   * Get the frequency
   * \returns frequency [Hz]
   */
  double GetFrequency (void) const;
  /**
   * Set the Tx antenna height
   * \param height Tx antenna height [m]
   */
  void SetTxAntennaHeight (double height);
  /**
   * Get the Tx antenna height
   * \returns Tx antenna height [m]
   */
  double GetTxAntennaHeight (void) const;
  /**
   * Set the RX antenna height
   * \param height RX antenna height [m]
   */
  void SetRxAntennaHeight (double height);
  /**
   * Get the RX antenna height
   * \returns RX antenna height [m]
   */
  double GetRxAntennaHeight (void) const;
  /**
   * Set the environment
   * \param environment
   */
  void SetEnvironment (Environment environment);
  /**
   * Get the environment
   * \returns environment
   */
  Environment GetEnvironment (void) const;

private:
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  ECC33PropagationLossModel (const ECC33PropagationLossModel &);
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns
   */
  ECC33PropagationLossModel & operator = (const ECC33PropagationLossModel &);

  virtual double DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
  virtual int64_t DoAssignStreams (int64_t stream);

  double m_frequency; //!< frequency [Hz]
  double m_TxAntennaHeight; //!< Tx Antenna Height [m]
  double m_RxAntennaHeight; //!< Rx Antenna Height [m]
  Environment m_environment;

};

}

#endif /* ECC33PROPAGATIONMODEL_H */
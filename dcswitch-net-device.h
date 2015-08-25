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
 * Author: Gustavo Carneiro  <gjc@inescporto.pt>
 */
#ifndef DCSWITCH_NET_DEVICE_H
#define DCSWITCH_NET_DEVICE_H

#include "ns3/net-device.h"
#include "ns3/mac48-address.h"
#include "ns3/nstime.h"
#include "ns3/dcswitch-channel.h"
#include <stdint.h>
#include <string>
#include <map>

namespace ns3 {

class Node;

/**
 * \defgroup dcswitch Dcswitch Device
 * 
 * \brief a virtual net device that dcswitchs multiple LAN segments
 *
 * The DcswitchNetDevice object is a "virtual" netdevice that aggregates
 * multiple "real" netdevices and implements the data plane forwarding
 * part of IEEE 802.1D.  By adding a DcswitchNetDevice to a Node, it
 * will act as a "dcswitch", or "switch", to multiple LAN segments.
 * 
 * By default the dcswitch netdevice implements a "learning dcswitch"
 * algorithm (see 802.1D), where incoming unicast frames from one port
 * may occasionally be forwarded throughout all other ports, but
 * usually they are forwarded only to a single correct output port.
 *
 * \attention The Spanning Tree Protocol part of 802.1D is not
 * implemented.  Therefore, you have to be careful not to create
 * bridging loops, or else the network will collapse.
 *
 * \attention Bridging is designed to work only with NetDevices
 * modelling IEEE 802-style technologies, such as CsmaNetDevice and
 * WifiNetDevice.
 *
 * \attention If including a WifiNetDevice in a dcswitch, the wifi
 * device must be in Access Point mode.  Adhoc mode is not supported
 * with bridging.
 */

/**
 * \ingroup dcswitch
 * \brief a virtual net device that dcswitchs multiple LAN segments
 */
class DcswitchNetDevice : public NetDevice
{
public:
  static TypeId GetTypeId (void);
  DcswitchNetDevice ();
  virtual ~DcswitchNetDevice ();

  /** 
   * \brief Add a 'port' to a dcswitch device
   *
   * This method adds a new dcswitch port to a DcswitchNetDevice, so that
   * the new dcswitch port NetDevice becomes part of the dcswitch and L2
   * frames start being forwarded to/from this NetDevice.
   *
   * \param dcswitchPort NetDevice
   * \attention The netdevice that is being added as dcswitch port must
   * _not_ have an IP address.  In order to add IP connectivity to a
   * bridging node you must enable IP on the DcswitchNetDevice itself,
   * never on its port netdevices.
   */
  void AddDcswitchPort (Ptr<NetDevice> dcswitchPort);

  uint32_t GetNDcswitchPorts (void) const;

  Ptr<NetDevice> GetDcswitchPort (uint32_t n) const;

  // inherited from NetDevice base class.
  virtual void SetIfIndex (const uint32_t index);
  virtual uint32_t GetIfIndex (void) const;
  virtual Ptr<Channel> GetChannel (void) const;
  virtual void SetAddress (Address address);
  virtual Address GetAddress (void) const;
  virtual bool SetMtu (const uint16_t mtu);
  virtual uint16_t GetMtu (void) const;
  virtual bool IsLinkUp (void) const;
  virtual void AddLinkChangeCallback (Callback<void> callback);
  virtual bool IsBroadcast (void) const;
  virtual Address GetBroadcast (void) const;
  virtual bool IsMulticast (void) const;
  virtual Address GetMulticast (Ipv4Address multicastGroup) const;
  virtual bool IsPointToPoint (void) const;
  virtual bool IsBridge (void) const;
  virtual bool Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber);
  virtual bool SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber);
  virtual Ptr<Node> GetNode (void) const;
  virtual void SetNode (Ptr<Node> node);
  virtual bool NeedsArp (void) const;
  virtual void SetReceiveCallback (NetDevice::ReceiveCallback cb);
  virtual void SetPromiscReceiveCallback (NetDevice::PromiscReceiveCallback cb);
  virtual bool SupportsSendFrom () const;
  virtual Address GetMulticast (Ipv6Address addr) const;

protected:
  virtual void DoDispose (void);

  void ReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                          Address const &source, Address const &destination, PacketType packetType);
  void ForwardUnicast (Ptr<NetDevice> incomingPort, Ptr<const Packet> packet,
                       uint16_t protocol, Mac48Address src, Mac48Address dst);
  void ForwardBroadcast (Ptr<NetDevice> incomingPort, Ptr<const Packet> packet,
                         uint16_t protocol, Mac48Address src, Mac48Address dst);
  void Learn (Mac48Address source, Ptr<NetDevice> port);
  Ptr<NetDevice> GetLearnedState (Mac48Address source);

private:
  DcswitchNetDevice (const DcswitchNetDevice &);
  DcswitchNetDevice &operator = (const DcswitchNetDevice &);

  NetDevice::ReceiveCallback m_rxCallback;
  NetDevice::PromiscReceiveCallback m_promiscRxCallback;

  Mac48Address m_address;
  Time m_expirationTime; // time it takes for learned MAC state to expire
  struct LearnedState
  {
    Ptr<NetDevice> associatedPort;
    Time expirationTime;
  };
  std::map<Mac48Address, LearnedState> m_learnState;
  Ptr<Node> m_node;
  Ptr<DcswitchChannel> m_channel;
  std::vector< Ptr<NetDevice> > m_ports;
  uint32_t m_ifIndex;
  uint16_t m_mtu;
  bool m_enableLearning;
};

} // namespace ns3

#endif /* DCSWITCH_NET_DEVICE_H */

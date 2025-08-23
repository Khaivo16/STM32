#include "STM32Ethernet.h"

// DHCP Library v0.3 - April 25, 2009
// Author: Jordan Terrell - blog.jordanterrell.com

#include <string.h>
#include <stdlib.h>
#include "Dhcp.h"
#include "Arduino.h"
#include "utility/stm32_eth.h"

int DhcpClass::beginWithDHCP(uint8_t *mac, unsigned long timeout, unsigned long responseTimeout)
{
  UNUSED(responseTimeout);
  _timeout = timeout;
  _dhcp_lease_state = DHCP_CHECK_NONE;

  // zero out _dhcpMacAddr
  memset(_dhcpMacAddr, 0, 6);
  reset_DHCP_lease();

  memcpy((void*)_dhcpMacAddr, (void*)mac, 6);
  _dhcp_state = STATE_DHCP_START;
  stm32_set_DHCP_state(_dhcp_state);
  return request_DHCP_lease();
}

void DhcpClass::reset_DHCP_lease(){
  _dhcp_state = STATE_DHCP_RELEASE;
  stm32_set_DHCP_state(_dhcp_state);
  stm32_eth_scheduler();
  _dhcp_state = stm32_get_DHCP_state();
}

//return:0 on error, 1 if request is sent and response is received
int DhcpClass::request_DHCP_lease(){

  int result = 0;
  unsigned long startTime = millis();

  while(_dhcp_state != STATE_DHCP_LEASED)
  {
    stm32_eth_scheduler();
    _dhcp_state = stm32_get_DHCP_state();

    if(result != 1 && ((millis() - startTime) > _timeout)) {
      reset_DHCP_lease();
      break;
    }
  }

  if(_dhcp_state == STATE_DHCP_LEASED) {
    result = 1;
  }

  return result;
}

/*
    returns:
    0/DHCP_CHECK_NONE: nothing happened
    1/DHCP_CHECK_RENEW_FAIL: renew failed
    2/DHCP_CHECK_RENEW_OK: renew success
    3/DHCP_CHECK_REBIND_FAIL: rebind fail
    4/DHCP_CHECK_REBIND_OK: rebind success
*/
int DhcpClass::checkLease(){
    int rc = DHCP_CHECK_NONE;

    stm32_eth_scheduler();
    rc = stm32_get_DHCP_lease_state();

    if(rc != _dhcp_lease_state)
    {
      switch(_dhcp_lease_state) {
        case DHCP_CHECK_NONE:
          _dhcp_lease_state = rc;
          rc = DHCP_CHECK_NONE;
        break;

        case DHCP_CHECK_RENEW_OK:
          _dhcp_lease_state = rc;
          if(rc == DHCP_CHECK_NONE) {
            rc = DHCP_CHECK_RENEW_OK;
          } else {
            rc = DHCP_CHECK_RENEW_FAIL;
          }
        break;

        case DHCP_CHECK_REBIND_OK:
          _dhcp_lease_state = rc;
          if(rc == DHCP_CHECK_NONE) {
            rc = DHCP_CHECK_REBIND_OK;
          } else {
            rc = DHCP_CHECK_REBIND_FAIL;
          }
        break;

        default:
          _dhcp_lease_state = DHCP_CHECK_NONE;
        break;
      }
    }

    return rc;
}

IPAddress DhcpClass::getLocalIp()
{
  return IPAddress(stm32_eth_get_ipaddr());
}

IPAddress DhcpClass::getSubnetMask()
{
  return IPAddress(stm32_eth_get_netmaskaddr());
}

IPAddress DhcpClass::getGatewayIp()
{
  return IPAddress(stm32_eth_get_gwaddr());
}

IPAddress DhcpClass::getDhcpServerIp()
{
  return IPAddress(stm32_eth_get_dhcpaddr());
}

IPAddress DhcpClass::getDnsServerIp()
{
  return IPAddress(stm32_eth_get_dnsaddr());
}

int EthernetClass::begin(unsigned long timeout, unsigned long responseTimeout)
{
  static DhcpClass s_dhcp;
  _dhcp = &s_dhcp;
  stm32_eth_init(MACAddressDefault(), NULL, NULL, NULL);

  // Now try to get our config info from a DHCP server
  int ret = _dhcp->beginWithDHCP(mac_address, timeout, responseTimeout);
  if(ret == 1)
  {
    _dnsServerAddress = _dhcp->getDnsServerIp();
  }

  return ret;
}

void EthernetClass::begin(IPAddress local_ip)
{
  IPAddress subnet(255, 255, 255, 0);
  begin(local_ip, subnet);
}

void EthernetClass::begin(IPAddress local_ip, IPAddress subnet)
{
  // Assume the gateway will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress gateway = local_ip;
  gateway[3] = 1;
  begin(local_ip, subnet, gateway);
}

void EthernetClass::begin(IPAddress local_ip, IPAddress subnet, IPAddress gateway)
{
  // Assume the DNS server will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress dns_server = local_ip;
  dns_server[3] = 1;
  begin(local_ip, subnet, gateway, dns_server);
}

void EthernetClass::begin(IPAddress local_ip, IPAddress subnet, IPAddress gateway, IPAddress dns_server)
{
  stm32_eth_init(MACAddressDefault(), local_ip.raw_address(), gateway.raw_address(), subnet.raw_address());
  /* If there is a local DHCP informs it of our manual IP configuration to
  prevent IP conflict */
  stm32_DHCP_manual_config();
  _dnsServerAddress = dns_server;
}

int EthernetClass::begin(uint8_t *mac_address, unsigned long timeout, unsigned long responseTimeout)
{
  static DhcpClass s_dhcp;
  _dhcp = &s_dhcp;

  stm32_eth_init(mac_address, NULL, NULL, NULL);

  // Now try to get our config info from a DHCP server
  int ret = _dhcp->beginWithDHCP(mac_address, timeout, responseTimeout);
  if(ret == 1)
  {
    _dnsServerAddress = _dhcp->getDnsServerIp();
  }
  MACAddress(mac_address);
  return ret;
}

void EthernetClass::begin(uint8_t *mac_address, IPAddress local_ip)
{
  // Assume the DNS server will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress dns_server = local_ip;
  dns_server[3] = 1;
  begin(mac_address, local_ip, dns_server);
}

void EthernetClass::begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server)
{
  // Assume the gateway will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress gateway = local_ip;
  gateway[3] = 1;
  begin(mac_address, local_ip, dns_server, gateway);
}

void EthernetClass::begin(uint8_t *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
{
  IPAddress subnet(255, 255, 255, 0);
  begin(mac_address, local_ip, dns_server, gateway, subnet);
}

void EthernetClass::begin(uint8_t *mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
{
  stm32_eth_init(mac, local_ip.raw_address(), gateway.raw_address(), subnet.raw_address());
  /* If there is a local DHCP informs it of our manual IP configuration to
  prevent IP conflict */
  stm32_DHCP_manual_config();
  _dnsServerAddress = dns_server;
  MACAddress(mac);
}

EthernetLinkStatus EthernetClass::linkStatus()
{
  return (!stm32_eth_is_init()) ? Unknown : (stm32_eth_link_up() ? LinkON : LinkOFF);
}

int EthernetClass::maintain(){
  int rc = DHCP_CHECK_NONE;

  if(_dhcp != NULL){
    //we have a pointer to dhcp, use it
    rc = _dhcp->checkLease();
    switch ( rc ){
      case DHCP_CHECK_NONE:
        //nothing done
        break;
      case DHCP_CHECK_RENEW_OK:
      case DHCP_CHECK_REBIND_OK:
        _dnsServerAddress = _dhcp->getDnsServerIp();
        break;
      default:
        //this is actually a error, it will retry though
        break;
    }
  }
  return rc;
}

/*
 * This function updates the LwIP stack and can be called to be sure to update
 * the stack (e.g. in case of a long loop).
 */
void EthernetClass::schedule(void)
{
  stm32_eth_scheduler();
}

uint8_t * EthernetClass::MACAddressDefault(void)
{
  if ((mac_address[0] + mac_address[1] + mac_address[2] + mac_address[3] + mac_address[4] + mac_address[5]) == 0) {
    uint32_t baseUID = *(uint32_t *)UID_BASE;
    mac_address[0] = 0x00;
    mac_address[1] = 0x80;
    mac_address[2] = 0xE1;
    mac_address[3] = (baseUID & 0x00FF0000) >> 16;
    mac_address[4] = (baseUID & 0x0000FF00) >> 8;
    mac_address[5] = (baseUID & 0x000000FF);
  }
  return mac_address;
}

void EthernetClass::MACAddress(uint8_t *mac) {
  mac_address[0] = mac[0];
  mac_address[1] = mac[1];
  mac_address[2] = mac[2];
  mac_address[3] = mac[3];
  mac_address[4] = mac[4];
  mac_address[5] = mac[5];
}

uint8_t * EthernetClass::MACAddress(void) {
  return mac_address;
}

IPAddress EthernetClass::localIP()
{
  return IPAddress(stm32_eth_get_ipaddr());
}

IPAddress EthernetClass::subnetMask()
{
  return IPAddress(stm32_eth_get_netmaskaddr());
}

IPAddress EthernetClass::gatewayIP()
{
  return IPAddress(stm32_eth_get_gwaddr());
}

IPAddress EthernetClass::dnsServerIP()
{
  return _dnsServerAddress;
}

EthernetClass Ethernet;

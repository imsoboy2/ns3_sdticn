
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/header.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-interface-address.h"
// #include "ns3/netanim-module.h"
#include "ns3/csma-module.h"

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <map>
#include <vector>
#include <queue>

#define INF 10000
using namespace ns3;

std::ofstream fout_t ("throughput1.txt");
NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhoc");

typedef struct SwitchComponent {
  Ipv4Address switch_addr;
  Ipv4Address targetAddr;
  int count;
}SwitchComponent;

typedef struct RControllerComponent{
  Ipv4Address controlleraddr;
  std::vector<SwitchComponent> switch_list;
}RControllerComponent;

//       ContollerAddr
std::vector<RControllerComponent> ControllerTable;
uint32_t totalnum_switch;

std::map<uint32_t, int> nodeIdTable; // ip -> nodeid
std::map<int, uint32_t> ipTable; // nodeid -> ip
std::map<uint32_t, uint64_t> macTable;
std::map<uint32_t, bool> controllerIpTable;
std::map<uint32_t, bool> switchIpTable;

// class Tier3ControllerReceiver {
//   public: 
//   void ReceivePacketInTier3Controller (Ptr<Socket> socket);
//   private:
//   // private variables
// };


int throughput_B = 0;
int throughput_C = 0;
int throughput_D = 0;
int throughput_E = 0;
int throughput_F = 0;
int throughput_G = 0;
int throughput_H = 0;
int throughput_I = 0;
int throughput_J = 0;
int throughput_K = 0;

int PktCount_Forwarding = 500;

const int numOfTier1Switch = 7;
const int numOfTier2Controller = 6;
const int E = 10;
int presetted_count= 50;
int nodeindex = 0;

std::map<uint32_t, int> heartbeatTable[numOfTier1Switch * numOfTier2Controller + numOfTier2Controller + 1];
std::map<uint32_t, uint32_t> ruleTable[numOfTier1Switch * numOfTier2Controller + numOfTier2Controller];

int map[2][100][100];
bool link[2][100][100];
int from[2][100][100];

int distance[100];
int visited[100];
int cost = 10;
int nodesize = 0;
int nodesize_temp = 0;
Ipv4Address Replaced_nodeIp;

void SendStuff (Ptr<Socket> sock, Ipv4Address dstaddr, uint16_t port);
void TestForwarding (Ptr<Socket> sock, Ipv4Address srcaddr);
void TestForwardingB (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingC (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingD (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingE (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingF (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingG (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingH (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingI (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingJ (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
void TestForwardingK (Ptr<Socket> sock, Ipv4Address srcaddr, int PktCount_Forwarding);
static void forwardPacket(Ptr<Socket> socket, Ptr<Packet> pkt);
void ControllerProfile(Ptr<Socket> sock, Ipv4Address ipAddr);

void CalculateThroughput()
{
  Time t = Simulator::Now();
  NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "\t" << throughput_B << "\t" << throughput_C << "\t" << throughput_D << "\t" << throughput_E << "\t" << throughput_F << "\t" << throughput_G << "\t" << throughput_H << "\t" << throughput_I << "\t" <<throughput_J << "\t" <<throughput_K <<std::endl);
  fout_t << Simulator::Now().GetSeconds() << "\t" << throughput_B << "\t" << throughput_C << "\t" << throughput_D << "\t" << throughput_E << "\t" << throughput_F << "\t" << throughput_G << "\t" << throughput_H << "\t" << throughput_I << "\t" <<throughput_J << "\t" <<throughput_K <<std::endl;
  throughput_B = 0;
  throughput_C = 0;
  throughput_D = 0;
  throughput_E = 0;
  throughput_F = 0;
  throughput_G = 0;
  throughput_H = 0;
  throughput_I = 0;
  throughput_J = 0;
  throughput_K = 0;
  Simulator::Schedule (Seconds(0.5), &CalculateThroughput);
}


void dijkstra(int start, int tier) {
  for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) 
    distance[i] = map[tier][start][i], visited[i] = false, from[tier][start][i] = i;
  visited[start] = true;
  distance[start] = 0;
  for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) {
    int u = 0, minC = INF;
    for (int j = 0; j < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; j++) {
      if (!visited[j] && distance[j] < minC)
        u = j, minC = distance[j];
    }
    visited[u] = true;
    for (int j = 0; j < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; j++) {
         if (!visited[j] && distance[u] + map[tier][u][j] < distance[j]) 
            distance[j] = distance[u] + map[tier][u][j], from[tier][start][j] = u;
      }
  }
  for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) {
    map[tier][start][i] = distance[i];
    from[tier][start][i] = (map[tier][start][i] < INF) ? from[tier][start][i] : INF;
  }
}

void printRoutingTable(int tier)
{
  // printf("----- min cost table -----\n");
  for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) {
    for (int j = 0; j < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; j++) {
      if (map[tier][i][j] >= INF)
        continue;
        // printf("  X");
      else
        // printf(" %2d", map[tier][i][j]);
        continue;
    }
    // printf("\n");
  }
  // printf("----- next hop table -----\n");
  for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) {
    for (int j = 0; j < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; j++) {
      if (from[tier][i][j] >= INF)
      continue;
        // printf("  X");
      else
        // printf(" %2d", from[tier][i][j]);
        continue;
    }
    // printf("\n");
  }
}
void initController()
{
  // routing table init
  std::fill(&map[0][0][0], &map[1][99][100], INF);
  std::fill(&link[0][0][0], &link[1][99][100], false);
  std::fill(&from[0][0][0], &from[1][99][100], INF);
  // for (int i = 0; i < numOfSwitch; i++) {
  //   for (int j = 0; j < numOfSwitch; j++) {
  //     if (map[i][j] != INF) printf("%d ", map[i][j]);
  //     else printf("X  ");
  //   }
  //   printf("\n");
  // }

  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) 
      dijkstra(i, j);
    // printRoutingTable(j);
  }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval, Ipv4Address src, Ipv4Address dest )
{
  if (pktCount > 0)
    {
      Ptr <Packet> pkt;
      Ipv4Header header;
      if (controllerIpTable.find(src.Get()) != controllerIpTable.end()) // I N  C O N T R O L L E R
      {
        if (pktSize == 10) 
        { // heartbeat msg
          header.SetSource(src);
          header.SetDestination(dest);
          header.SetProtocol(156); // heartbeat send msg
          pkt = Create<Packet> (pktSize);
          pkt->AddHeader(header);
          socket->SetPriority(2);
          socket->Send (pkt);
          Simulator::Schedule (pktInterval, &GenerateTraffic,
                              socket, pktSize,pktCount - 1, pktInterval, src, dest);
        }
        else 
        { // rule msg
          uint8_t *msg = new uint8_t[12];
          uint16_t packetSize = 12; // [srcip][dstip][fwdip]
          header.SetSource(src);
          header.SetProtocol(157); // rule msg
          for (int i = 0; i < numOfTier1Switch; i++) {
            for (int j = 0; j < numOfTier1Switch; j++) { // if pkt ip == dst ip -> switch need to forward packet to from[i][j]
              msg[0] = ipTable[i] >> 24; msg[1] = (ipTable[i] >> 16) % 256; msg[2] = (ipTable[i] >> 8) % 256; msg[3] = ipTable[i] % 256;
              msg[4] = ipTable[j] >> 24; msg[5] = (ipTable[j] >> 16) % 256; msg[6] = (ipTable[j] >> 8) % 256; msg[7] = ipTable[j] % 256;
              header.SetDestination(Ipv4Address(ipTable[i]));
              if (map[0][i][j] != 0 && map[0][i][j] < INF) // forward
              {
                int fwd = from[0][i][j];
                // printf("fwd = %d\n", fwd);
                msg[8] = ipTable[fwd] >> 24; msg[9] = (ipTable[fwd] >> 16) % 256; msg[10] = (ipTable[fwd] >> 8) % 256; msg[11] = ipTable[fwd] % 256;
              }
              else msg[8] = msg[9] = msg[10] = msg[11] = 0; // drop
              pkt = Create<Packet> (msg, packetSize);
              pkt->AddHeader(header);
              socket->SetPriority(1);
              socket->Send (pkt);
              Simulator::Schedule (pktInterval, &GenerateTraffic,
                              socket, pktSize,pktCount - 1, pktInterval, src, Ipv4Address(ipTable[i]));
            }
          }
        }
      }
      else // I N  S W I T C H
      {
        header.SetSource(src);
        header.SetDestination(dest);
        if (pktSize == 10) // heartbeat response msg
        {
          header.SetProtocol(158); 
          socket->SetPriority(2);
          uint8_t *msg = new uint8_t[heartbeatTable[socket->GetNode()->GetId()].size() * 5];
          uint16_t packetSize = heartbeatTable[socket->GetNode()->GetId()].size() * 5; // [srcip]

          int i = 0;
          for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end(); it++) {
            msg[i] = it->first >> 24; msg[i + 1] = (it->first >> 16) % 256; msg[i + 2] = (it->first >> 8) % 256; msg[i + 3] = it->first % 256;
            msg[i + 4] = (it->second < 0) ? 0 : 1; // link down ? 0 : 1 
            i += 5;
          }
          pkt = Create<Packet> (msg, packetSize);
          pktSize = packetSize;
        }
        else // packet forwarding with dscp
        {
          socket->SetPriority(1);
          pkt = Create<Packet> (pktSize);
        }
        pkt->AddHeader(header);
        socket->Send (pkt);
        Simulator::Schedule (pktInterval, &GenerateTraffic,
                            socket, pktSize, pktCount - 1, pktInterval, src, dest);
      }
    }
}

void TestForwarding (Ptr<Socket> socket, Ipv4Address srcaddr)
{
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.8"));

  int pktSize = 7;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.16").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.12" << "), forwarding in Tier 2 & 3\n";
  return;
}
void TestForwardingB (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.5"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.17").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingB, socket, srcaddr, PktCount_Forwarding-1);
  // std::cout << PktCount_Forwarding << std::endl;
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingC (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.7"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.19").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingC, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingD (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.11"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.23").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingD, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingE (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.13"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.5").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingE, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingF (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.17"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.7").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingF, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingG (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.19"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.11").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingG, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingH (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.23"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.13").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingH, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingI (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.5"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.17").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingI, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingJ (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.7"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.19").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingJ, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}

void TestForwardingK (Ptr<Socket> socket, Ipv4Address srcaddr, int PktCount_Forwarding)
{
  if (PktCount_Forwarding > 0) {
  Ipv4Header header = Ipv4Header();
  header.SetProtocol(6); 
  header.SetSource(srcaddr);
  header.SetDestination(Ipv4Address ("10.1.1.11"));

  int pktSize = 1500;
  uint8_t *payload = new uint8_t[pktSize];
  payload[6] = (uint8_t) 1;
  uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.23").Get()];
  payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
  payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
  Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
  pkt->AddHeader(header);

  socket->SetPriority(1);
  socket->Send (pkt);

  // std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.11" << ", forwarding in Tier 1 & 2\n";
  
  Simulator::Schedule (Seconds(0.001), &TestForwardingK, socket, srcaddr, PktCount_Forwarding-1);
  // Simulator::Stop (Seconds (10));
}
  else {
  return;}
}




void SendPacketInTier3Controller (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval, Ipv4Address src, Ipv4Address dest)
{
  if (pktCount > 0)
    {
      Ptr <Packet> pkt;
      Ipv4Header header;
      if (pktSize == 10) 
      { // heartbeat msg
        header.SetSource(src);
        header.SetDestination(dest);
        header.SetProtocol(160); // heartbeat send msg (tier3 controller)
        pkt = Create<Packet> (pktSize);
        pkt->AddHeader(header);
        socket->SetPriority(2);
        socket->Send (pkt);
        Simulator::Schedule (pktInterval, &SendPacketInTier3Controller,
                             socket, pktSize,pktCount - 1, pktInterval, src, dest);
      }
      else 
      { // rule msg
        uint8_t *msg = new uint8_t[12];
        uint16_t packetSize = 12; // [srcip][dstip][fwdip]
        header.SetSource(src);
        header.SetProtocol(161); // tier3 controller rule msg
        for (int i = 0; i < numOfTier1Switch; i++) {
          for (int j = 0; j < numOfTier1Switch; j++) { // if pkt ip == dst ip -> switch need to forward packet to from[i][j]
            msg[0] = ipTable[i] >> 24; msg[1] = (ipTable[i] >> 16) % 256; msg[2] = (ipTable[i] >> 8) % 256; msg[3] = ipTable[i] % 256;
            msg[4] = ipTable[j] >> 24; msg[5] = (ipTable[j] >> 16) % 256; msg[6] = (ipTable[j] >> 8) % 256; msg[7] = ipTable[j] % 256;
            header.SetDestination(Ipv4Address(ipTable[i]));
            if (map[1][i][j] != 0 && map[1][i][j] < INF) // forward
            {
              int fwd = from[1][i][j];
              msg[8] = ipTable[fwd] >> 24; msg[9] = (ipTable[fwd] >> 16) % 256; msg[10] = (ipTable[fwd] >> 8) % 256; msg[11] = ipTable[fwd] % 256;
            }
            else msg[8] = msg[9] = msg[10] = msg[11] = 0; // drop
            pkt = Create<Packet> (msg, packetSize);
            pkt->AddHeader(header);
            socket->SetPriority(1);
            socket->Send (pkt);
            Simulator::Schedule (pktInterval, &SendPacketInTier3Controller,
                            socket, pktSize,pktCount - 1, pktInterval, src, Ipv4Address(ipTable[i]));
          }
        }
      }
    }
}




void ReceivePacketInTier3Controller (Ptr<Socket> socket)
{
  Ptr <Packet> received;
  Ptr<Ipv4> ipv4 = socket->GetNode()->GetObject<Ipv4>();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0); 
  Ipv4Address ipAddr = iaddr.GetLocal ();
  while (received = socket->Recv ())
    {
      bool stateChanged = false;
      Ipv4Header hdr;
      received->RemoveHeader(hdr);
      if (hdr.GetProtocol() == 162) // get heartbeat response 
      {
        if (heartbeatTable[socket->GetNode()->GetId()].find(hdr.GetSource().Get()) == heartbeatTable[socket->GetNode()->GetId()].end()) { // Packet-In 
          heartbeatTable[socket->GetNode()->GetId()].insert({hdr.GetSource().Get(), 20}); // sourceIp, cnt
        }
        else {
          heartbeatTable[socket->GetNode()->GetId()][hdr.GetSource().Get()] = 20;
          stateChanged = true;
        }

        uint8_t *buffer = new uint8_t[received->GetSize()];
        received->CopyData(buffer, received->GetSize());
        uint32_t switchLink;

        for (int i = 0; i < ((int)received->GetSize() / 5); i++) {
          switchLink = (buffer[i * 5] << 24) + (buffer[i * 5 + 1] << 16) + (buffer[i * 5 + 2] << 8) + buffer[i * 5 + 3];
          bool isLived = (buffer[i * 5 + 4] == 0) ? false : true;
          if (switchLink == ipAddr.Get()) continue;
          if ((!isLived && link[1][nodeIdTable[hdr.GetSource().Get()]][nodeIdTable[switchLink]]) 
              || (isLived && !link[1][nodeIdTable[hdr.GetSource().Get()]][nodeIdTable[switchLink]])) {
            link[1][nodeIdTable[hdr.GetSource().Get()]][nodeIdTable[switchLink]] = isLived;
            stateChanged = true;
          }
        }
        
        for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end();) {
          if (it->second < 0) { 
            int nodeId = nodeIdTable[it->first];
            for (auto it2 = heartbeatTable[nodeId].begin(); it2 != heartbeatTable[nodeId].end(); it2++)
            {
                  int nodeId2 = nodeIdTable[it2->first];
                  nodesize_temp = heartbeatTable[nodeId2].size();             
                  
                  if (nodesize_temp > nodesize)
                  {
                    nodesize = nodesize_temp;
                    Replaced_nodeIp = Ipv4Address(ipTable[nodeIdTable[it2->first]]);

                  }
                  nodesize_temp = 0;
            }
            nodeindex = (Replaced_nodeIp.Get() % 256)-1 ;
            
            heartbeatTable[socket->GetNode()->GetId()].erase(it++);
            stateChanged = true;
            for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++)
              link[1][nodeId][i] = link[1][i][nodeId] = false, from[0][nodeId][i] = from[0][i][nodeId] = INF;
          } else {
 

            ++it;
          }
        }
        if (stateChanged) {
          std::fill(&map[1][0][0], &map[1][99][100], INF);
          std::fill(&from[1][0][0], &from[1][99][100], INF);
          for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) 
            for (int j = 0; j < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; j++) 
              if (link[1][i][j]) map[1][i][j] = map[1][j][i] = cost;
          for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++)
            dijkstra(i, 1);
          printRoutingTable(1);
          
          Simulator::Schedule (Seconds(0.01), &SendPacketInTier3Controller,
                                socket, 1, 1, Seconds(0.01), ipAddr, Ipv4Address("255.255.255.255"));
          stateChanged = false;
        }
      }
    }
}

// typedef struct SwitchComponent {
//   Ipv4Address switch_addr;
//   int count;
// };

// typedef struct RControllerComponent{
//   Ipv4addr controlleraddr;
//   std::vector<SwitchComponent> switch_list;
// };

// //       ContollerAddr
// std::vector<RControllerComponent> ControllerTable;

void RegisterControllerTable(std::vector< RControllerComponent> &ControllerTable, Ipv4Address conaddr, Ipv4Address switchaddr, Ipv4Address targetAddr)
{

  if (targetAddr != conaddr){
    // NS_LOG_UNCOND(targetAddr);

  for (std::vector<RControllerComponent>::iterator iter = ControllerTable.begin(); iter != ControllerTable.end(); ++iter){
      for (std::vector<SwitchComponent>::iterator iter2 = iter->switch_list.begin(); iter2 != iter->switch_list.end(); ++iter2){

          iter2->count = iter2->count - 1;
          // std::cout<<"Controller_addr : "<< iter->controlleraddr << " with Switch " <<iter2->switch_addr << " :::::: " << iter2->count << " target_addr : " << iter2->targetAddr <<std::endl;
      }
    }
  
  for (std::vector<RControllerComponent>::iterator iter = ControllerTable.begin(); iter != ControllerTable.end(); ++iter){

      for (std::vector<SwitchComponent>::iterator iter2 = iter->switch_list.begin(); iter2 != iter->switch_list.end(); ){
        if (iter2->count == 0){
          iter2 = iter->switch_list.erase(iter2);
          // std::cout<<"erase"<<std::endl;
        }
        else{
          iter2++;
        }
      }
    }

  bool find_controller = false;
 
  for (std::vector<RControllerComponent>::iterator iter = ControllerTable.begin(); iter != ControllerTable.end(); ++iter){
    bool find_switch = false;
    if (iter->controlleraddr == conaddr){
      
      for (std::vector<SwitchComponent>::iterator iter2 = iter->switch_list.begin(); iter2 != iter->switch_list.end(); ++iter2){
        if (iter2->switch_addr == switchaddr){
          find_controller = true;
          iter2->count = presetted_count;
          iter2->targetAddr = targetAddr;
        }
      }
      if (find_switch == false){
        SwitchComponent switch_temp;
        switch_temp.count = presetted_count;
        switch_temp.switch_addr = switchaddr;
        switch_temp.targetAddr = targetAddr;
        iter->switch_list.push_back(switch_temp);
      }
    }
  }
  if (find_controller == false)
  {
  RControllerComponent concomp_temp;
  SwitchComponent switch_temp;
  switch_temp.count = presetted_count;
  switch_temp.switch_addr = switchaddr;
  switch_temp.targetAddr = targetAddr;
  concomp_temp.controlleraddr = conaddr;
  concomp_temp.switch_list.push_back(switch_temp);  
  ControllerTable.push_back(concomp_temp);
  }
  }
}

void ReceivePacketInController (Ptr<Socket> socket)
{
  Ptr <Packet> received;
  Ptr<Ipv4> ipv4 = socket->GetNode()->GetObject<Ipv4>();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0); 
  Ipv4Address ipAddr = iaddr.GetLocal ();
  srand((socket->GetNode()->GetId()) * 2);
  double jitter = rand() % 100 / 10000.0;
  while (received = socket->Recv ())
    {
      bool stateChanged = false;
      Ipv4Header hdr;
      received->RemoveHeader(hdr);
      if (hdr.GetProtocol() == 200)
      {
      Ptr<Ipv4> ipv4 = socket->GetNode()->GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0); 
      Ipv4Address ipAddr = iaddr.GetLocal ();
      Ipv4Address targetAddr = Ipv4Address(hdr.GetSource().Get() - (hdr.GetSource().Get() % 256) % (numOfTier1Switch +1) + numOfTier1Switch + 1);
      RegisterControllerTable(ControllerTable, ipAddr, hdr.GetSource(), targetAddr);
      // std::cout << targetAddr << std::endl;

    //print the list
    for (std::vector<RControllerComponent>::iterator iter = ControllerTable.begin(); iter != ControllerTable.end(); ++iter){
      // std::cout<<iter->controlleraddr;

      for (std::vector<SwitchComponent>::iterator iter2 = iter->switch_list.begin(); iter2 != iter->switch_list.end(); ++iter2){
          // std::cout<<iter2->switch_addr;
      }
      }
      }
      if (hdr.GetProtocol() == 158) // get heartbeat response 
      {
        if (heartbeatTable[socket->GetNode()->GetId()].find(hdr.GetSource().Get()) == heartbeatTable[socket->GetNode()->GetId()].end()) { // Packet-In 
          heartbeatTable[socket->GetNode()->GetId()].insert({hdr.GetSource().Get(), 20}); // sourceIp, cnt
        }
        else {
          heartbeatTable[socket->GetNode()->GetId()][hdr.GetSource().Get()] = 20;
          stateChanged = true;
        }

        uint8_t *buffer = new uint8_t[received->GetSize()];
        received->CopyData(buffer, received->GetSize());
        uint32_t switchLink;
        for (int i = 0; i < ((int)received->GetSize() / 5); i++) {
          switchLink = (buffer[i * 5] << 24) + (buffer[i * 5 + 1] << 16) + (buffer[i * 5 + 2] << 8) + buffer[i * 5 + 3];
          bool isLived = (buffer[i * 5 + 4] == 0) ? false : true;
          if (switchLink == ipAddr.Get()) continue;
          if ((!isLived && link[0][nodeIdTable[hdr.GetSource().Get()]][nodeIdTable[switchLink]]) 
              || (isLived && !link[0][nodeIdTable[hdr.GetSource().Get()]][nodeIdTable[switchLink]])) {
            link[0][nodeIdTable[hdr.GetSource().Get()]][nodeIdTable[switchLink]] = isLived;
            stateChanged = true;
          }
        }
        
        for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end();) {
          if (it->second < 0) {
            int nodeId = nodeIdTable[it->first];
            heartbeatTable[socket->GetNode()->GetId()].erase(it++);
            stateChanged = true;
            for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++)
              link[0][nodeId][i] = link[0][i][nodeId] = false, from[0][nodeId][i] = from[0][i][nodeId] = INF;
          } else {
            // std::cout<<Ipv4Address(ipTable[nodeIdTable[it->first]])<<" HelpController"<<std::endl;

            ++it;
          }
        }
        if (stateChanged) {
          std::fill(&map[0][0][0], &map[0][99][100], INF);
          std::fill(&from[0][0][0], &from[0][99][100], INF);
          for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++) 
            for (int j = 0; j < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; j++) 
              if (link[0][i][j]) map[0][i][j] = map[0][j][i] = cost;
          for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++)
            dijkstra(i, 0);
          printRoutingTable(0);
          
          Simulator::Schedule (Seconds(0.01), &GenerateTraffic,
                                socket, 1, 1, Seconds(0.1), ipAddr, Ipv4Address("255.255.255.255"));
          stateChanged = false;
        }
      } 
      else if (hdr.GetProtocol() == 160) { // get heartbeat request message
        if (nodeIdTable.find(ipAddr.Get()) == nodeIdTable.end()) // not exist
        {
          nodeIdTable.insert({ipAddr.Get(), socket->GetNode()->GetId() - 1}); // sourceIp, nodeId
          ipTable.insert({socket->GetNode()->GetId() - 1, ipAddr.Get()});
        }
        else
          nodeIdTable[ipAddr.Get()] = socket->GetNode()->GetId() - 1;

        InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
        socket->SetAllowBroadcast (true);
        socket->Connect (remote);
        Ptr <Packet> pkt;
        Ipv4Header header;
        header.SetSource(ipAddr);
        header.SetDestination(Ipv4Address("255.255.255.255"));
        header.SetProtocol(162); // heartbeat send msg

        uint8_t *msg = new uint8_t[heartbeatTable[socket->GetNode()->GetId()].size() * 5];
        uint16_t packetSize = heartbeatTable[socket->GetNode()->GetId()].size() * 5; // [srcip]
        int i = 0;
        for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end(); it++) {
          msg[i] = it->first >> 24; msg[i + 1] = (it->first >> 16) % 256; msg[i + 2] = (it->first >> 8) % 256; msg[i + 3] = it->first % 256;
          msg[i + 4] = (it->second < 0) ? 0 : 1; // link down ? 0 : 1 
          i += 5;
        }
        pkt = Create<Packet> (msg, packetSize);
        pkt->AddHeader(header);
        socket->SetPriority(2);
        socket->Send (pkt);
      } else if (hdr.GetProtocol() == 162) { // heartbeat response message -> link state update
        if (heartbeatTable[socket->GetNode()->GetId()].find(hdr.GetSource().Get()) == heartbeatTable[socket->GetNode()->GetId()].end()) { // Packet-In 
          heartbeatTable[socket->GetNode()->GetId()].insert({hdr.GetSource().Get(), 20}); // sourceIp, cnt
        }
        else
          heartbeatTable[socket->GetNode()->GetId()][hdr.GetSource().Get()] = 20;
        
        for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end(); it++) {
          if (it->second < 0) {
            it->second = -1;
          }
        }
      } else if (controllerIpTable.find(hdr.GetSource().Get()) != controllerIpTable.end() && hdr.GetProtocol() == 161)
      { // rule message from tier3 controller
        uint8_t *buffer = new uint8_t[received->GetSize()];
        received->CopyData(buffer, received->GetSize());
        uint32_t src = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
        if (src != ipAddr.Get()) return;
        uint32_t dest = (buffer[4] << 24) + (buffer[5] << 16) + (buffer[6] << 8) + buffer[7];
        uint32_t toFwd = (buffer[8] << 24) + (buffer[9] << 16) + (buffer[10] << 8) + buffer[11];
        if (ruleTable[socket->GetNode()->GetId() - 1].find(Ipv4Address(dest).Get()) == ruleTable[socket->GetNode()->GetId() - 1].end()) {
          ruleTable[socket->GetNode()->GetId() - 1].insert({dest, toFwd});
        }
        else
          ruleTable[socket->GetNode()->GetId() - 1][dest] = toFwd;
      } else if (controllerIpTable.find(hdr.GetSource().Get()) == controllerIpTable.end()) { // if not my layer packet, drop
        return;
      } else if (controllerIpTable.find(hdr.GetDestination().Get()) != controllerIpTable.end()) { // packet forward
        if (hdr.GetDestination().Get() == ipAddr.Get()) if (hdr.GetDestination().Get() == ipAddr.Get()) {
          // std::cout << "Forwarding Complete dest ip : " << hdr.GetDestination() << std::endl; 
        if (hdr.GetDestination() == "10.1.1.5")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_B += (bTotal) / 1000000; 

          }
          else if (hdr.GetDestination() == "10.1.1.7")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_C += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.11")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_D += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.13")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_E += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.17")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_F += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.19")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_G += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.23")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_H += (bTotal) / 1000000;
          }
          // else if (hdr.GetDestination() == "10.1.1.5")
          // {
          //   int packets_now = received->GetSize();
          //   int bTotal = packets_now * 1500;
          //   throughput_I += (bTotal) / 1000000;
          // }
          // else if (hdr.GetDestination() == "10.1.1.7")
          // {
          //   int packets_now = received->GetSize();
          //   int bTotal = packets_now * 1500;
          //   throughput_J += (bTotal) / 1000000;
          // }
          // else if (hdr.GetDestination() == "10.1.1.11")
          // {
          //   int packets_now = received->GetSize();
          //   int bTotal = packets_now * 1500;
          //   throughput_K += (bTotal) / 1000000;
          // }
          return;


        }// packet received in dst node;// packet received in dst node
        if (hdr.GetProtocol() == 6) {
          uint8_t *buffer = new uint8_t[received->GetSize()];
          received->CopyData(buffer, received->GetSize());
          uint64_t dst = ((uint64_t) buffer[0] << 40) + ((uint64_t) buffer[1] << 32) + ((uint64_t) buffer[2] << 24) + ((uint64_t) buffer[3] << 16) + ((uint64_t) buffer[4] << 8) + buffer[5];
          if (macTable[ipAddr.Get()] != dst) return;
          if (ruleTable[socket->GetNode()->GetId() - 1].find(hdr.GetDestination().Get()) != ruleTable[socket->GetNode()->GetId() - 1].end()
                && ruleTable[socket->GetNode()->GetId() - 1][hdr.GetDestination().Get()] != 0) {
            uint32_t nextHopIp = ruleTable[socket->GetNode()->GetId() - 1][hdr.GetDestination().Get()];
            uint64_t nextHopMac = macTable[nextHopIp];
            buffer[0] = (uint8_t) (nextHopMac >> 40); buffer[1] = (uint8_t) ((nextHopMac >> 32) % 256); buffer[2] = (uint8_t) ((nextHopMac >> 24) % 256);
            buffer[3] = (uint8_t) ((nextHopMac >> 16) % 256); buffer[4] = (uint8_t) ((nextHopMac >> 8) % 256); buffer[5] = (uint8_t) (nextHopMac % 256);
            Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
            pkt->AddHeader(hdr);
            std::cout << "FORWARD PACKET: src(" << hdr.GetSource() << ")->dst(" << hdr.GetDestination() << ") in " << ipAddr << "\n";
            if (buffer[6] == (uint8_t)0) {
              printf("CRITICAL PACKET\n");
              printf("GO TO SATELITE\n");
              socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address ("10.1.2.1"), 12345));
            }
            else
              Simulator::Schedule (Seconds(jitter), &forwardPacket, socket, pkt);
          } else { // forwarding fail
            uint64_t nextHopMac = macTable[hdr.GetDestination().Get()];
            buffer[0] = (uint8_t) (nextHopMac >> 40); buffer[1] = (uint8_t) ((nextHopMac >> 32) % 256); buffer[2] = (uint8_t) ((nextHopMac >> 24) % 256);
            buffer[3] = (uint8_t) ((nextHopMac >> 16) % 256); buffer[4] = (uint8_t) ((nextHopMac >> 8) % 256); buffer[5] = (uint8_t) (nextHopMac % 256);
            Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
            pkt->AddHeader(hdr);
            printf("FORWARDING FAILURE TIER 32\n");
            printf("GO TO SATELITE TIER 32\n");
            socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address ("10.1.2.1"), 12345));
          }
        } else {
            Simulator::Schedule (Seconds(jitter), &GenerateTraffic, socket, received->GetSize(), 1, 
                                Seconds(0.1), hdr.GetSource(), hdr.GetDestination());
        }
      }
    }
}

void SendPacket (Ptr<Socket> socket, uint32_t param)
{
  if (socket->GetPriority() == ((uint8_t) 2)) {
    for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end(); it++) {
      heartbeatTable[socket->GetNode()->GetId()][it->first] = (it->second) - 1;
    }
  }
}

static void forwardPacket(Ptr<Socket> socket, Ptr<Packet> pkt)
{
  socket->SetPriority(1);
  socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address ("255.255.255.255"), 12345));
  NS_LOG_UNCOND("asdfasfd");
}

bool test = true;
void ReceivePacketInSwitch (Ptr<Socket> socket)
{
  Ptr<Packet> received;
  while (received = socket->Recv ())
    {
      srand((socket->GetNode()->GetId()) * 2);
      double jitter = rand() % 100 / 10000.0;
      Ipv4Header hdr;
      received->RemoveHeader(hdr);
      Ptr<Ipv4> ipv4 = socket->GetNode()->GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0); 
      Ipv4Address ipAddr = iaddr.GetLocal ();
      if (hdr.GetProtocol() == 156) 
      { // heartbeat request message
        if (nodeIdTable.find(ipAddr.Get()) == nodeIdTable.end()) // not exist
        {
          nodeIdTable.insert({ipAddr.Get(), socket->GetNode()->GetId() - 1}); // sourceIp, nodeId
          ipTable.insert({socket->GetNode()->GetId() - 1, ipAddr.Get()});
        }
        else
          nodeIdTable[ipAddr.Get()] = socket->GetNode()->GetId() - 1;

        InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
        socket->SetAllowBroadcast (true);
        socket->Connect (remote);
        Simulator::Schedule (Seconds(jitter), &GenerateTraffic, socket, 10, 1, Seconds(0.1), ipAddr, Ipv4Address ("255.255.255.255"));
        Simulator::Schedule(Seconds(jitter), &ControllerProfile, socket, ipAddr);
      }
      else if (hdr.GetProtocol() == 158)
      { // heartbeat response message -> link state update
        if (heartbeatTable[socket->GetNode()->GetId()].find(hdr.GetSource().Get()) == heartbeatTable[socket->GetNode()->GetId()].end()) { // Packet-In 
          heartbeatTable[socket->GetNode()->GetId()].insert({hdr.GetSource().Get(), 20}); // sourceIp, cnt
        }
        else
          heartbeatTable[socket->GetNode()->GetId()][hdr.GetSource().Get()] = 20;
        
        for(auto it = heartbeatTable[socket->GetNode()->GetId()].begin(); it != heartbeatTable[socket->GetNode()->GetId()].end(); it++) {
          if (it->second < 0) {
            it->second = -1;
          }
        }
      }
      else if (controllerIpTable.find(hdr.GetSource().Get()) != controllerIpTable.end() && hdr.GetProtocol() == 157)
      { // rule message
        uint8_t *buffer = new uint8_t[received->GetSize()];
        received->CopyData(buffer, received->GetSize());
        uint32_t src = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
        if (src != ipAddr.Get()) return;
        uint32_t dest = (buffer[4] << 24) + (buffer[5] << 16) + (buffer[6] << 8) + buffer[7];
        uint32_t toFwd = (buffer[8] << 24) + (buffer[9] << 16) + (buffer[10] << 8) + buffer[11];
        if (ruleTable[socket->GetNode()->GetId() - 1].find(Ipv4Address(dest).Get()) == ruleTable[socket->GetNode()->GetId() - 1].end()) {
          ruleTable[socket->GetNode()->GetId() - 1].insert({dest, toFwd});
        }
        else
          ruleTable[socket->GetNode()->GetId() - 1][dest] = toFwd;
      }
      else if (switchIpTable.find(hdr.GetSource().Get()) == switchIpTable.end()) { // if not my layer packet, drop
        return;
      } else {
        // if (hdr.GetDestination().Get() == ipAddr.Get()) return;// packet received in dst node
        if (hdr.GetDestination().Get() == ipAddr.Get())
        if (hdr.GetProtocol() == 6) {
          uint8_t *buffer = new uint8_t[received->GetSize()];
          received->CopyData(buffer, received->GetSize());
          uint64_t dst = ((uint64_t) buffer[0] << 40) + ((uint64_t) buffer[1] << 32) + ((uint64_t) buffer[2] << 24) + ((uint64_t) buffer[3] << 16) + ((uint64_t) buffer[4] << 8) + buffer[5];
          if (macTable[ipAddr.Get()] != dst) return;
          // std::cout << "Forwarding Complete dest ip : " << hdr.GetDestination() << std::endl;
        if (hdr.GetDestination() == "10.1.1.5")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_B += (bTotal) / 1000000;

          }
          else if (hdr.GetDestination() == "10.1.1.7")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_C += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.11")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_D += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.13")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_E += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.17")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_F += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.19")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_G += (bTotal) / 1000000;
          }
          else if (hdr.GetDestination() == "10.1.1.23")
          {
            int packets_now = received->GetSize();
            int bTotal = packets_now * 1500;
            throughput_H += (bTotal) / 1000000;
          }

        // packet received in dst node
          if (ruleTable[socket->GetNode()->GetId() - 1].find(hdr.GetDestination().Get()) != ruleTable[socket->GetNode()->GetId() - 1].end()
                && ruleTable[socket->GetNode()->GetId() - 1][hdr.GetDestination().Get()] != 0) {
            uint32_t nextHopIp = ruleTable[socket->GetNode()->GetId() - 1][hdr.GetDestination().Get()];
            uint64_t nextHopMac = macTable[nextHopIp];
            buffer[0] = (uint8_t) (nextHopMac >> 40); buffer[1] = (uint8_t) ((nextHopMac >> 32) % 256); buffer[2] = (uint8_t) ((nextHopMac >> 24) % 256);
            buffer[3] = (uint8_t) ((nextHopMac >> 16) % 256); buffer[4] = (uint8_t) ((nextHopMac >> 8) % 256); buffer[5] = (uint8_t) (nextHopMac % 256);
            Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
            pkt->AddHeader(hdr);
            std::cout << "FORWARD PACKET: src(" << hdr.GetSource() << ")->dst(" << hdr.GetDestination() << ") in " << ipAddr << "\n";
            if (buffer[6] == (uint8_t)0) {

              printf("CRITICIAL PACKET TIER 12\n");
              printf("GO TO SATELITE TIER 12\n");
              socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address ("10.1.2.1"), 12345));
            }
            else
            {
              Simulator::Schedule (Seconds(jitter), &forwardPacket, socket, pkt);
            }
          }
          
            else {
            // std::cout<<"failfail"<<std::endl;
               // forwarding fail
            bool Target_Terminal_Switch_exist = false;
            bool Controller_Terminal_Switch_exist = false;
            Ipv4Address SourceController_IP = Ipv4Address(hdr.GetSource().Get() - (hdr.GetSource().Get() % 256) % (numOfTier1Switch +1) + numOfTier1Switch + 1);
            Ipv4Address targetAddr_temp = Ipv4Address(hdr.GetDestination().Get() - (hdr.GetDestination().Get() % 256) % (numOfTier1Switch +1) + numOfTier1Switch + 1);
            Ipv4Address Target_Terminal_IP;
            Ipv4Address Controller_Terminal_IP;
            
            for (std::vector<RControllerComponent>::iterator iter = ControllerTable.begin(); iter != ControllerTable.end(); ++iter){
              for (std::vector<SwitchComponent>::iterator iter2 = iter->switch_list.begin(); iter2 != iter->switch_list.end(); ++iter2){
                if (iter->controlleraddr == SourceController_IP && iter2->targetAddr == targetAddr_temp) {
                  Target_Terminal_IP = iter2->switch_addr;
                  Target_Terminal_Switch_exist = true;
                  }
              }
            }
            for (std::vector<RControllerComponent>::iterator iter = ControllerTable.begin(); iter != ControllerTable.end(); ++iter){
              for (std::vector<SwitchComponent>::iterator iter2 = iter->switch_list.begin(); iter2 != iter->switch_list.end(); ++iter2){
                if (iter->controlleraddr == targetAddr_temp && iter2->targetAddr == SourceController_IP) {
                  Controller_Terminal_IP = iter2->switch_addr;
                  Controller_Terminal_Switch_exist = true;
                  }
              }
            }
            // std::cout<<Target_Terminal_Switch_exist<<"Target Terminal switch exist"<<std::endl;
            // std::cout<<Controller_Terminal_Switch_exist<<"Controller Terminal switch exist"<<std::endl;
            if (Target_Terminal_Switch_exist && Controller_Terminal_Switch_exist) {
              if (dst != macTable[Controller_Terminal_IP.Get()]) {
                  printf("FORWARDING PACKET TO CONTROLLER_TERMINAL_IP _ TIER 12\n");
                  uint64_t nextHopMac = macTable[Controller_Terminal_IP.Get()];
                  // std::cout << " next hop mac  : " << uint64_t(nextHopMac) << std::endl; 
                  buffer[0] = (uint8_t) (nextHopMac >> 40); buffer[1] = (uint8_t) ((nextHopMac >> 32) % 256); buffer[2] = (uint8_t) ((nextHopMac >> 24) % 256);
                  buffer[3] = (uint8_t) ((nextHopMac >> 16) % 256); buffer[4] = (uint8_t) ((nextHopMac >> 8) % 256); buffer[5] = (uint8_t) (nextHopMac % 256);
                  // Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
                  Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
                  pkt->AddHeader(hdr);
                  socket->SendTo (pkt, 0, InetSocketAddress (Controller_Terminal_IP, 12345));
                  return;
              }

              else if (dst == macTable[Controller_Terminal_IP.Get()]) {
                  printf("FORWARDING PACKET TO TARGET_TERMINAL_IP _ TIER 12\n");
                  uint64_t nextHopMac = macTable[Target_Terminal_IP.Get()];
                  // std::cout << " next hop mac  : " << uint64_t(nextHopMac) << std::endl; 
                  buffer[0] = (uint8_t) (nextHopMac >> 40); buffer[1] = (uint8_t) ((nextHopMac >> 32) % 256); buffer[2] = (uint8_t) ((nextHopMac >> 24) % 256);
                  buffer[3] = (uint8_t) ((nextHopMac >> 16) % 256); buffer[4] = (uint8_t) ((nextHopMac >> 8) % 256); buffer[5] = (uint8_t) (nextHopMac % 256);
                  // Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
                  Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
                  pkt->AddHeader(hdr);
                  socket->SendTo (pkt, 0, InetSocketAddress (Target_Terminal_IP, 12345));
                  return;
              }
            }


             // forwarding fail
            uint64_t nextHopMac = macTable[hdr.GetDestination().Get()];
            buffer[0] = (uint8_t) (nextHopMac >> 40); buffer[1] = (uint8_t) ((nextHopMac >> 32) % 256); buffer[2] = (uint8_t) ((nextHopMac >> 24) % 256);
            buffer[3] = (uint8_t) ((nextHopMac >> 16) % 256); buffer[4] = (uint8_t) ((nextHopMac >> 8) % 256); buffer[5] = (uint8_t) (nextHopMac % 256);
            Ptr <Packet> pkt = Create<Packet> (buffer, received->GetSize());
            pkt->AddHeader(hdr);
            // printf("GO TO SATELITE FORWARDING FAILURE\n");
            socket->SendTo (pkt, 0, InetSocketAddress (Ipv4Address ("10.1.2.1"), 12345));
          }
        }
        else {
            Simulator::Schedule (Seconds(jitter), &GenerateTraffic, socket, received->GetSize(), 1, 
                                Seconds(0.1), hdr.GetSource(), hdr.GetDestination());
        }
      }
    }
}

void ForwardPacketToOtherDomain (Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> packet = socket->RecvFrom (from);
  InetSocketAddress address = InetSocketAddress::ConvertFrom (from);
  Ipv4Header hdr;
  packet->PeekHeader(hdr);
  Ipv4Address dstAddress = Ipv4Address(hdr.GetDestination().Get() + 256);
  socket->SendTo (packet, 0, InetSocketAddress (dstAddress, address.GetPort()));
  NS_LOG_UNCOND ("GET PACKET IN SATELITE:: Destination Received " << packet->GetSize () << " bytes from " << address.GetIpv4 ());
  NS_LOG_UNCOND ("PACKET INFO " << hdr.GetSource() << " -> " << hdr.GetDestination());
}

void ControllerProfile(Ptr<Socket> sock, Ipv4Address srcAddr)
{
  Ipv4Header header;
  header.SetProtocol(200); // Controller_switch table msg
  header.SetSource(srcAddr);
  Ptr<Packet> p = Create<Packet> (2525);
  p->AddHeader(header);
  // NS_LOG_UNCOND(srcAddr<<std::endl);
  sock->Send (p);
  return;
}

// void SendpacketinSwitch (Ptr<Socket> socket, Ipv4Address srcaddr)
// {
//   Ipv4Header header = Ipv4Header();
//   header.SetProtocol(6); 
//   header.SetSource(srcaddr);
//   header.SetDestination(Ipv4Address ("10.1.1.4"));

//   int pktSize = 7;
//   uint8_t *payload = new uint8_t[pktSize];
//   payload[6] = (uint8_t) 1;
//   uint64_t nextHopMac = macTable[Ipv4Address("10.1.1.8").Get()];
//   payload[0] = (uint8_t) (nextHopMac >> 40); payload[1] = (uint8_t) ((nextHopMac >> 32) % 256); payload[2] = (uint8_t) ((nextHopMac >> 24) % 256);
//   payload[3] = (uint8_t) ((nextHopMac >> 16) % 256); payload[4] = (uint8_t) ((nextHopMac >> 8) % 256); payload[5] = (uint8_t) (nextHopMac % 256);
//   Ptr <Packet> pkt = Create<Packet> (payload, pktSize);
//   pkt->AddHeader(header);

//   socket->SetPriority(1);
//   socket->Send (pkt);

//   std::cout << "SEND PACKET: src(" << srcaddr << ") dst(" << "10.1.1.4" << "), forwarding to satelite\n";
//   return;
// }


static void changeNodePos(NodeContainer c_tier2)
{
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (int i = 0; i < 3; i++)
  {
    positionAlloc->Add (Vector (50, 50, 50));
  }
  
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c_tier2);
}

// static void changeNodePos2(NodeContainer c)
// {
//   // MobilityHelper mobility;
//   // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//   // for (int i = 0; i < 3; i++)
//   // {
//   //   positionAlloc->Add (Vector (rand() % 50 / 5.0, rand() % 50 / 5.0, 0));
//   // }
  
//   // mobility.SetPositionAllocator (positionAlloc);
//   // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//   // NS_LOG_UNCOND ("Node pos changed");
//   // mobility.Install (c_tier2);


//   MobilityHelper mobility;
//   Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//   for (int i = 0; i < numOfTier1Switch * numOfTier2Controller + numOfTier2Controller; i++)
//   {
//     positionAlloc->Add (Vector (rand() % 50 / 5.0, rand() % 50 / 5.0, 10));
//   }
//   positionAlloc->Add (Vector (2.5, 2.5, 2.5));
  
//   mobility.SetPositionAllocator (positionAlloc);
//   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//   mobility.Install (c);
// }

void ReceivePacketInSatelite (Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> packet = socket->RecvFrom (from);
  InetSocketAddress address = InetSocketAddress::ConvertFrom (from);
  Ipv4Header hdr;
  packet->PeekHeader(hdr);
  Ipv4Address dstAddress = Ipv4Address(hdr.GetDestination().Get() + 256 + 1);
  socket->SendTo (packet, 0, InetSocketAddress (dstAddress, address.GetPort()));
  NS_LOG_UNCOND ("GET PACKET IN SATELITE:: Destination Received " << packet->GetSize () << " bytes from " << address.GetIpv4 ());
  NS_LOG_UNCOND ("PACKET INFO " << hdr.GetSource() << " -> " << hdr.GetDestination());
}

void SendPacketInSatelite (Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> packet = socket->RecvFrom (from);
  packet->RemoveAllPacketTags ();
  packet->RemoveAllByteTags ();
  InetSocketAddress address = InetSocketAddress::ConvertFrom (from);
  NS_LOG_INFO ("Destination Received " << packet->GetSize () << " bytes from " << address.GetIpv4 ());
  NS_LOG_INFO ("Triggering packet back to satelite");
  SendStuff (socket, Ipv4Address ("10.1.2.1"), address.GetPort ());
}

void SendStuff (Ptr<Socket> sock, Ipv4Address dstaddr, uint16_t port)
{
  Ipv4Header header;
  Address from;
  header.SetProtocol(6); // satelite msg
  Ptr<Packet> p = sock-> RecvFrom (from);
  p->AddHeader(header);
  // NS_LOG_UNCOND(dstaddr << ", " << port);
  sock->SendTo (p, 0, InetSocketAddress (Ipv4Address (dstaddr), port));
  return;
}


static void ChangeSocket (NodeContainer c, TypeId tid){

  if (nodeindex != 0) {
  
  Ptr<Socket> controllerRecvSink = Socket::CreateSocket (c.Get (nodeindex), tid);  
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  controllerRecvSink->Bind (local);
  controllerRecvSink->SetRecvCallback (MakeCallback (&ReceivePacketInController));

  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  controllerRecvSink->SetAllowBroadcast (true);
  controllerRecvSink->Connect (remote);
  controllerRecvSink->SetSendCallback (MakeCallback (&SendPacket));
  
  }

  nodeindex = 0;
  Simulator::Schedule(Seconds(0.0001), &ChangeSocket, c, tid);
}



int main (int argc, char *argv[])
{
  std::string phyMode ("DsssRate11Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 20;
  double interval = 1.0; // seconds
  bool verbose = false;
  Packet::EnablePrinting (); 

  CommandLine cmd (__FILE__);
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse (argc, argv);
  // Convert to time object

  initController();
  Time interPacketInterval = Seconds (interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  NodeContainer c;
  c.Create ((numOfTier1Switch * numOfTier2Controller) + numOfTier2Controller + 1); // switch + controller2 + controller3

  NodeContainer c_tier3;
  c_tier3.Add (c.Get(numOfTier2Controller * numOfTier1Switch + numOfTier2Controller));

  NodeContainer c_tier2;
  NodeContainer c_tier1;
  for (int idx = 0; idx < numOfTier2Controller; idx++){
    c_tier2.Add (c.Get (numOfTier1Switch + (numOfTier1Switch + 1) * idx));
    for (int j = 0; j < numOfTier1Switch; j++){
      c_tier1.Add (c.Get(idx * (numOfTier1Switch + 1) + j));
    }
  }

  NodeContainer satelite;
  satelite.Create(1);
  
  NodeContainer c_satelite;
  c_satelite.Add (satelite);
  for (int i=0 ; i < (numOfTier1Switch * numOfTier2Controller) + numOfTier2Controller + 1 ; i++) {
    c_satelite.Add (c.Get(i));
  }
  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  wifi.SetStandard (WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  // wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(10.0));
  wifiPhy.SetChannel (wifiChannel.Create ());
  
  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

                                  
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  // Set it to adhoc mode
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);
  
  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("1Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (500)));
  NetDeviceContainer csmaDevices;
  csmaDevices = csma.Install (c_satelite);
  
  // MobilityHelper mobility;
  // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  // for (int i = 0; i < numOfTier2Controller; i++)
  // {
  //   positionAlloc->Add (Vector (1 * i + 2.5, 1 * i + 2.5, 1 * i + 2.5));
  //   for (int j = 0; j < numOfTier1Switch; j++)
  //     positionAlloc->Add (Vector (1 * i + (rand() % 50 / 10.0), 1 * i + (rand() % 50 / 10.0), 1 * i + (rand() % 50 / 10.0)));
  // }
  // mobility.SetPositionAllocator (positionAlloc);
  // mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
  //                            "Mode", StringValue ("Time"),
  //                            "Time", TimeValue (Seconds (3.0)),
  //                            "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
  //                            "Bounds", StringValue ("0|30|0|30"));
  // mobility.Install (c);

  // MobilityHelper mobility_satelite;
  // Ptr<ListPositionAllocator> positionAlloc_satelite = CreateObject<ListPositionAllocator> ();
  // positionAlloc_satelite->Add (Vector (150.0, 150.0, 150.0));
  // mobility_satelite.SetPositionAllocator (positionAlloc_satelite);
  // mobility_satelite.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // mobility_satelite.Install (satelite);
  
    
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  positionAlloc->Add (Vector (12.5, 18.5, 0));
  positionAlloc->Add (Vector (6.5, 6.5, 0));
  positionAlloc->Add (Vector (18.5, 6.5, 0));



  for (int i = 0; i < numOfTier1Switch; i++)
  {
    positionAlloc->Add (Vector (10 + rand() % 50 / 10.0, 16 + rand() % 50 / 10.0, 0));
  }

  for (int i = 0; i < numOfTier1Switch; i++)
  {
    positionAlloc->Add (Vector (4 + rand() % 50 / 10.0, 4 + rand() % 50 / 10.0, 0));
  }

  for (int i = 0; i < numOfTier1Switch; i++)
  {
    positionAlloc->Add (Vector (16 + rand() % 50 / 10.0, 4 + rand() % 50 / 10.0, 0));
  }
  positionAlloc->Add (Vector (12.5, 12.5, 0));
  positionAlloc->Add (Vector (100.0, 100.0, 100.0));
  
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility.Install (c_tier2);
  mobility.Install (c_tier1);
  mobility.Install (c_tier3);
  mobility.Install (satelite);
  
  
  InternetStackHelper internet;
  internet.Install (c);
  internet.Install (satelite);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  Ipv4AddressHelper ipv4_s;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4_s.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer i_s = ipv4_s.Assign (csmaDevices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  uint64_t broadcast = 281474976710655; // FF:FF:FF:FF:FF:FF
  macTable.insert({Ipv4Address("255.255.255.255").Get(), broadcast});
  for (int idx = 0; idx < numOfTier2Controller; idx++) {
    Ptr<Socket> controllerRecvSink = Socket::CreateSocket (c.Get (numOfTier1Switch + (numOfTier1Switch + 1) * idx), tid); // 
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    controllerRecvSink->Bind (local);
    controllerRecvSink->SetRecvCallback (MakeCallback (&ReceivePacketInController));

    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
    controllerRecvSink->SetAllowBroadcast (true);
    controllerRecvSink->Connect (remote);
    controllerRecvSink->SetSendCallback (MakeCallback (&SendPacket));
    
    for (int j = 0; j < numOfTier1Switch; j++) {
      Ptr<Socket> switchRecvSink = Socket::CreateSocket (c.Get (idx * (numOfTier1Switch + 1) + j), tid); 
      local = InetSocketAddress (Ipv4Address::GetAny (), 80);
      switchRecvSink->Bind (local);
      switchRecvSink->SetRecvPktInfo(true);
      switchRecvSink->SetRecvCallback (MakeCallback (&ReceivePacketInSwitch));

      remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
      switchRecvSink->SetAllowBroadcast (true);
      switchRecvSink->Connect (remote);
      switchRecvSink->SetSendCallback (MakeCallback (&SendPacket));

      Ptr<Ipv4> cip = c.Get(idx * (numOfTier1Switch + 1) + j)->GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddr = cip->GetAddress (1,0); 
      Ipv4Address ipAddr = iaddr.GetLocal ();
      
      switchIpTable.insert({ipAddr.Get(), true});
      macTable.insert({ipAddr.Get(), idx * (numOfTier1Switch + 1) + j});
      Simulator::Schedule (Seconds(2), &TestForwardingB, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.001), &TestForwardingC, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.002), &TestForwardingD, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.003), &TestForwardingE, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.004), &TestForwardingF, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.005), &TestForwardingG, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.006), &TestForwardingH, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.007), &TestForwardingI, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.008), &TestForwardingJ, switchRecvSink, ipAddr, PktCount_Forwarding);
      Simulator::Schedule (Seconds(2.009), &TestForwardingK, switchRecvSink, ipAddr, PktCount_Forwarding);
    }
    
    Ptr<Ipv4> cip = c.Get(numOfTier1Switch + (numOfTier1Switch + 1) * idx)->GetObject<Ipv4>();
    Ipv4InterfaceAddress iaddr = cip->GetAddress (1,0); 
    Ipv4Address ipAddr = iaddr.GetLocal ();
    controllerIpTable.insert({ipAddr.Get(), true});
    macTable.insert({ipAddr.Get(), numOfTier1Switch + (numOfTier1Switch + 1) * idx}); // 00:00:00:00:01:01

    Simulator::Schedule (Seconds(2), &TestForwarding, controllerRecvSink, ipAddr);
    Simulator::Schedule (Seconds(0.5 + 0.01 * idx), &GenerateTraffic, controllerRecvSink, 10, 
                       numPackets, Seconds(0.5 + 0.01 * idx), ipAddr, Ipv4Address ("255.255.255.255"));
  }

  // int abc = 0;
  Ptr<Socket> tier3ControllerRecvSink = Socket::CreateSocket (c.Get (numOfTier2Controller * numOfTier1Switch + numOfTier2Controller), tid); 
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  tier3ControllerRecvSink->Bind (local);
  tier3ControllerRecvSink->SetRecvPktInfo(true);
  tier3ControllerRecvSink->SetRecvCallback (MakeCallback (&ReceivePacketInTier3Controller));
  // tier3ControllerRecvSink->SetRecvCallback (MakeCallback (&ReceivePacketInTier3Controller, c));

  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  tier3ControllerRecvSink->SetAllowBroadcast (true);
  tier3ControllerRecvSink->Connect (remote);
  tier3ControllerRecvSink->SetSendCallback (MakeCallback (&SendPacket));
  Ptr<Ipv4> cip = c.Get(numOfTier2Controller * numOfTier1Switch + numOfTier2Controller)->GetObject<Ipv4>();
  Ipv4InterfaceAddress iaddr = cip->GetAddress (1,0); 
  Ipv4Address ipAddr = iaddr.GetLocal ();
  macTable.insert({ipAddr.Get(), numOfTier2Controller * numOfTier1Switch + numOfTier2Controller});

  Simulator::Schedule (Seconds(0.5), &SendPacketInTier3Controller, tier3ControllerRecvSink, 10, 
                    numPackets, Seconds(0.5), ipAddr, Ipv4Address ("255.255.255.255"));

  // Ptr<Socket> sateliteSource = Socket::CreateSocket (c_satelite.Get (numOfTier2Controller * numOfTier1Switch + numOfTier2Controller + 1), TypeId::LookupByName ("ns3::UdpSocketFactory"));
  
  // InetSocketAddress srcaddr = InetSocketAddress (Ipv4Address ("10.1.2.15"), 12345);
  // sateliteSource->Bind (srcaddr);
  // sateliteSource->SetRecvCallback (MakeCallback (&SendPacketInSatelite));
  // // 얘 sateliteSource 라는 socket에 모든 c 노드 컨테이너 노드들을 담아주는거 있어야 함. 
  
  Ptr<Socket> sateliteRecvSink = Socket::CreateSocket (c_satelite.Get (0), TypeId::LookupByName ("ns3::UdpSocketFactory"));
  uint16_t dstport = 12345;
  Ipv4Address dstaddr ("10.1.2.1");
  InetSocketAddress dst = InetSocketAddress (dstaddr, dstport);
  sateliteRecvSink->Bind (dst);
  sateliteRecvSink->SetRecvCallback (MakeCallback (&ReceivePacketInSatelite));
  // Simulator::Schedule (Seconds (4.01), &SendStuff, sateliteSource, dstaddr, dstport);
  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);
 
  // Output what we are doing
  NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );

  Simulator::Schedule(Seconds(8.0), &changeNodePos, c_tier2);
  // Simulator::Schedule(Seconds(8.0), &changeNodePos2, c);
  Simulator::Schedule(Seconds(1.0), &CalculateThroughput);
  Simulator::Schedule(Seconds(2.5), &ChangeSocket, c, tid);
  // AnimationInterface anim ("hanwha.xml");
  Simulator::Run ();
  Simulator::Destroy ();
  
  return 0;
}

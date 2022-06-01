from ryu.base import app_manager
from ryu.controller import mac_to_port
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.mac import haddr_to_bin
from ryu.lib.packet import packet
from ryu.lib.packet import arp
from ryu.lib.packet import ipv4
from ryu.lib.packet import ethernet
from ryu.lib.packet import ether_types
from ryu.lib import mac
from ryu.lib import hub
from operator import attrgetter
from ryu.topology.api import get_switch, get_link
from ryu.app.wsgi import ControllerBase
from ryu.topology import event, switches
from collections import defaultdict
import time

topo = {
            1 : { 1: 0, 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : 10, 6: float('inf'),7: 20, 8 : float('inf'), 9 : 30, 10 : float('inf'), 11: 40, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            2 : { 1: float('inf'), 2: 0, 3: float('inf'), 4: float('inf'), 5 : 20, 6: float('inf'),7: 10, 8 : float('inf'), 9 : 20, 10 : float('inf'), 11: 30, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            3 : { 1: float('inf'), 2: float('inf'), 3: 0, 4: float('inf'), 5 : float('inf'), 6: 30,7: float('inf'), 8 : 20, 9 : float('inf'), 10 : 10, 11: float('inf'), 12: 20, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            4 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: 0, 5 : float('inf'), 6: 40,7: float('inf'), 8 : 30, 9 : float('inf'), 10 : 20, 11: float('inf'), 12: 10, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            5 : { 1: 10, 2: 20, 3: float('inf'), 4: float('inf'), 5 : 0, 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 10, 14: 20, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            6 : { 1: float('inf'), 2: float('inf'), 3: 30, 4: 40, 5 : float('inf'), 6: 0,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 20, 14: 10, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            7 : { 1: 20, 2: 10, 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 0, 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 10, 16: 20, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            8 : { 1: float('inf'), 2: float('inf'), 3: 20, 4: 30, 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : 0, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 20, 16: 10, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            9 : { 1: 30, 2: 20, 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 0, 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 10, 18 : 20, 19 : float('inf'), 20 : float('inf')},
            10: { 1: float('inf'), 2: float('inf'), 3: 10, 4: 20, 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : 0, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 20, 18 : 10, 19 : float('inf'), 20 : float('inf')},
            11 : { 1: 40, 2: 30, 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 0, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 10, 20 : 20},
            12 : { 1: float('inf'), 2: float('inf'), 3: 20, 4: 10, 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: 0, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 20, 20 : 10},
            13 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : 10, 6: 20,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 0, 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            14 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : 20, 6: 10,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: 0, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            15 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 10, 8 : 20, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 0, 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            16 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 20, 8 : 10, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: 0, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            17 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 10, 10 : 20, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 0, 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            18 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 20, 10 : 10, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : 0, 19 : float('inf'), 20 : float('inf')},
            19 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 10, 12: 20, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 0, 20 : float('inf')},
            20: { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 20, 12: 10, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : 0},
        }

topobw = {
            1 : { 1: 0, 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : 5, 6: float('inf'),7: 5, 8 : float('inf'), 9 : 5, 10 : float('inf'), 11: 5, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            2 : { 1: float('inf'), 2: 0, 3: float('inf'), 4: float('inf'), 5 : 10, 6: float('inf'),7: 10, 8 : float('inf'), 9 : 10, 10 : float('inf'), 11: 10, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            3 : { 1: float('inf'), 2: float('inf'), 3: 0, 4: float('inf'), 5 : float('inf'), 6: 5,7: float('inf'), 8 : 5, 9 : float('inf'), 10 : 5, 11: float('inf'), 12: 5, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            4 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: 0, 5 : float('inf'), 6: 10,7: float('inf'), 8 : 10, 9 : float('inf'), 10 : 10, 11: float('inf'), 12: 10, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            5 : { 1: 5, 2: 10, 3: float('inf'), 4: float('inf'), 5 : 0, 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 2, 14: 2, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            6 : { 1: float('inf'), 2: float('inf'), 3: 5, 4: 10, 5 : float('inf'), 6: 0,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 3, 14: 3, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            7 : { 1: 5, 2: 10, 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 0, 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 2, 16: 2, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            8 : { 1: float('inf'), 2: float('inf'), 3: 5, 4: 10, 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : 0, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 3, 16: 3, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            9 : { 1: 5, 2: 10, 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 0, 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 2, 18 : 2, 19 : float('inf'), 20 : float('inf')},
            10 : { 1: float('inf'), 2: float('inf'), 3: 5, 4: 10, 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : 0, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 3, 18 : 3, 19 : float('inf'), 20 : float('inf')},
            11 : { 1: 5, 2: 10, 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 0, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 2, 20 : 2},
            12 : { 1: float('inf'), 2: float('inf'), 3: 5, 4: 10, 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: 0, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 3, 20 : 3},
            13 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : 2, 6: 3,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 0, 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            14 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : 2, 6: 3,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: 0, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            15 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 2, 8 : 3, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 0, 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            16 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 2, 8 : 3, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: 0, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            17 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 2, 10 : 3, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 0, 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            18 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 2, 10 : 3, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : 0, 19 : float('inf'), 20 : float('inf')},
            19 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 2, 12: 3, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 0, 20 : float('inf')},
            20 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 2, 12: 3, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : 0},
        }

#Πίνακας switches
switches = []
 
# mymac[srcmac]->(switch, port)
mymac={}
 
#Γειτνίαση adjacency map [sw1][sw2]->port from sw1 to sw2
adjacency=defaultdict(lambda:defaultdict(lambda:None))
 
def minimum_distance(distance, Q):
  min = float('Inf')
  node = 0
  for v in Q:
    if distance[v] < min:
      min = distance[v]
      node = v
  return node
 
def get_path (src,dst,first_port,final_port):
  #Dijkstra's algorithm
  # print("get_path is called, src=",src," dst=",dst, " first_port=", first_port, " final_port=", final_port)
  distance = {}
  previous = {}
 
  for dpid in switches:
    distance[dpid] = float('Inf')
    previous[dpid] = None
 
  distance[src]=0
  Q=set(switches)
   
  while len(Q)>0:
    u = minimum_distance(distance, Q)
    Q.remove(u)
   
    for p in switches:
      if adjacency[u][p]!=None:
        w = 1/topobw[u][p]  # 1 for hop, topo[u][p] for distance, 1/topobw[u][p] for bandwidth
        if distance[u] + w < distance[p]:
          distance[p] = distance[u] + w
          previous[p] = u
 
  r=[]
  p=dst
  r.append(p)
  q=previous[p]
  while q is not None:
    if q == src:
      r.append(q)
      break
    p=q
    r.append(p)
    q=previous[p]
 
  r.reverse()
  if src==dst:
    path=[src]
  else:
    path=r
 
  # Now add the ports
  r = []
  in_port = first_port
  for s1,s2 in zip(path[:-1],path[1:]):
    out_port = adjacency[s1][s2]
    r.append((s1,in_port,out_port))
    in_port = adjacency[s2][s1]
  r.append((dst,in_port,final_port))
  return r
 
class ProjectController(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]
 
    def __init__(self, *args, **kwargs):
        super(ProjectController, self).__init__(*args, **kwargs)
        self.mac_to_port = {}
        self.topology_api_app = self
        self.datapath_list=[]
        self.port_speed = {}    # record the port speed 
        self.sleep = 0.5        # the interval of getting statistic
        self.state_len = 3      # the length of speed list of per port and flow.
        self.port_stats = {}
        self.datapaths = {}
        self.monitor_thread = hub.spawn(self._monitor)
        self.k={}
        self.countr=0

    def _monitor(self):
        while True:
            for dp in self.datapaths.values():
                self._request_stats(dp)
            hub.sleep(self.sleep)
    
    def _save_stats(self, _dict, key, value, length):
        if key not in _dict:
            _dict[key] = []
        _dict[key].append(value)

        if len(_dict[key]) > length:
            _dict[key].pop(0)

    def _get_time(self, sec, nsec):
        return sec + nsec / (10 ** 9)
    
    def _get_speed(self, now, pre, period):
        if period:
            return (((now - pre)*8) / (period*1000000))
        else:
            return 0

    def _request_stats(self, datapath):
        self.logger.debug('send stats request: %016x', datapath.id)
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser

        req = parser.OFPPortStatsRequest(datapath, 0, ofproto.OFPP_ANY)
        datapath.send_msg(req)
    
    def _get_period(self, n_sec, n_nsec, p_sec, p_nsec):
        return self._get_time(n_sec, n_nsec) - self._get_time(p_sec, p_nsec)
    
    @set_ev_cls(ofp_event.EventOFPPortStatsReply, MAIN_DISPATCHER)
    def _port_stats_reply_handler(self, ev):
        body = ev.msg.body
        self.countr+=1

        # self.logger.info('datapath port '
        #     'rx-pkts rx-bytes rx-error '
        #     'tx-pkts tx-bytes tx-error')
        # self.logger.info('---------------- -------- '
        #     '-------- -------- -------- '
        #     '-------- -------- --------')
        # for stat in sorted(body, key=attrgetter('port_no')):
        #     self.logger.info('%016x %8x %8d %8d %8d %8d %8d %8d',
        #         ev.msg.datapath.id, stat.port_no,
        #         stat.rx_packets, stat.rx_bytes, stat.rx_errors,
        #         stat.tx_packets, stat.tx_bytes, stat.tx_errors)

        # for key, value in sorted(self.port_speed.items()):
        #   print(self.port_speed[(1)], ' : ', value)
        # print('\n Speed:\n', self.port_speed)
        
          
        
        for stat in sorted(body, key=attrgetter('port_no')):
            if stat.port_no != ofproto_v1_3.OFPP_LOCAL:
                key = (ev.msg.datapath.id, stat.port_no)
                value = (
                    stat.tx_bytes, stat.rx_bytes, stat.rx_errors,
                    stat.duration_sec, stat.duration_nsec)

                self._save_stats(self.port_stats, key, value, self.state_len)

                # Get port speed.
                pre = 0
                period = self.sleep
                tmp = self.port_stats[key]
                if len(tmp) > 1:
                    pre = tmp[-2][0] + tmp[-2][1]
                    period = self._get_period(
                        tmp[-1][3], tmp[-1][4],
                        tmp[-2][3], tmp[-2][4])

                speed = self._get_speed(
                    self.port_stats[key][-1][0]+self.port_stats[key][-1][1],
                    pre, period)

                self._save_stats(self.port_speed, key, speed, self.state_len)
      
        if(self.countr%20==0):
          global topobw
          topobw = {
            1 : { 1: 0, 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : abs(5- self.port_speed.get((1,1))[-1]), 6: float('inf'),7: abs(5- self.port_speed.get((1,2))[-1]), 8 : float('inf'), 9 : abs(5- self.port_speed.get((1,3))[-1]), 10 : float('inf'), 11: abs(5- self.port_speed.get((1,4))[-1]), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            2 : { 1: float('inf'), 2: 0, 3: float('inf'), 4: float('inf'), 5 : abs(10- self.port_speed.get((2,1))[-1]), 6: float('inf'),7: abs(10- self.port_speed.get((2,1))[-1]), 8 : float('inf'), 9 : abs(10- self.port_speed.get((2,1))[-1]), 10 : float('inf'), 11: abs(10- self.port_speed.get((2,1))[-1]), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            3 : { 1: float('inf'), 2: float('inf'), 3: 0, 4: float('inf'), 5 : float('inf'), 6: abs(5- self.port_speed.get((3,1))[-1]),7: float('inf'), 8 : abs(5- self.port_speed.get((3,1))[-1]), 9 : float('inf'), 10 : abs(5- self.port_speed.get((3,1))[-1]), 11: float('inf'), 12: abs(5- self.port_speed.get((3,1))[-1]), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            4 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: 0, 5 : float('inf'), 6: abs(10- self.port_speed.get((4,1))[-1]),7: float('inf'), 8 : abs(10- self.port_speed.get((4,1))[-1]), 9 : float('inf'), 10 : abs(10- self.port_speed.get((4,1))[-1]), 11: float('inf'), 12: abs(10- self.port_speed.get((4,1))[-1]), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            5 : { 1: abs(5- self.port_speed.get((5,1))[-1]), 2: abs(10- self.port_speed.get((5,2))[-1]), 3: float('inf'), 4: float('inf'), 5 : 0, 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: abs(2- self.port_speed.get((5,3))[-1]), 14: abs(2- self.port_speed.get((5,4))[-1]), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            6 : { 1: float('inf'), 2: float('inf'), 3: abs(5- self.port_speed.get((6,1))[-1]), 4: abs(10- self.port_speed.get((6,2))[-1]), 5 : float('inf'), 6: 0,7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: abs(3- self.port_speed.get((6,3))[-1]), 14: abs(3- self.port_speed.get((6,4))[-1]), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            7 : { 1: abs(5- self.port_speed.get((7,1))[-1]), 2: abs(10- self.port_speed.get((7,2))[-1]), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: 0, 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : abs(2- self.port_speed.get((7,3))[-1]), 16: abs(2- self.port_speed.get((7,4))[-1]), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            8 : { 1: float('inf'), 2: float('inf'), 3: abs(5- self.port_speed.get((8,1))[-1]), 4: abs(10- self.port_speed.get((8,2))[-1]), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : 0, 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : abs(3- self.port_speed.get((8,3))[-1]), 16: abs(3- self.port_speed.get((8,4))[-1]), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            9 : { 1: abs(5- self.port_speed.get((9,1))[-1]), 2: abs(10- self.port_speed.get((9,2))[-1]), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : 0, 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: abs(2- self.port_speed.get((9,3))[-1]), 18 : abs(2- self.port_speed.get((9,4))[-1]), 19 : float('inf'), 20 : float('inf')},
            10 : { 1: float('inf'), 2: float('inf'), 3: abs(5- self.port_speed.get((10,1))[-1]), 4: abs(10- self.port_speed.get((10,2))[-1]), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : 0, 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: abs(3- self.port_speed.get((10,3))[-1]), 18 : abs(3- self.port_speed.get((10,4))[-1]), 19 : float('inf'), 20 : float('inf')},
            11 : { 1: abs(5- self.port_speed.get((11,1))[-1]), 2: abs(10- self.port_speed.get((11,2))[-1]), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: 0, 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : abs(2- self.port_speed.get((11,3))[-1]), 20 : abs(2- self.port_speed.get((11,4))[-1])},
            12 : { 1: float('inf'), 2: float('inf'), 3: abs(5- self.port_speed.get((12,1))[-1]), 4: abs(10- self.port_speed.get((12,2))[-1]), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: 0, 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : abs(3- self.port_speed.get((12,3))[-1]), 20 : abs(3- self.port_speed.get((12,4))[-1])},
            13 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : abs(2- self.port_speed.get((13,1))[-1]), 6: abs(3- self.port_speed.get((13,2))[-1]),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: 0, 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            14 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : abs(2- self.port_speed.get((14,1))[-1]), 6: abs(3- self.port_speed.get((14,2))[-1]),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: 0, 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            15 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: abs(2- self.port_speed.get((15,1))[-1]), 8 : abs(3- self.port_speed.get((15,2))[-1]), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : 0, 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            16 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: abs(2- self.port_speed.get((16,1))[-1]), 8 : abs(3- self.port_speed.get((16,2))[-1]), 9 : float('inf'), 10 : float('inf'), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: 0, 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            17 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : abs(2- self.port_speed.get((17,1))[-1]), 10 : abs(3- self.port_speed.get((17,2))[-1]), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: 0, 18 : float('inf'), 19 : float('inf'), 20 : float('inf')},
            18 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : abs(2- self.port_speed.get((18,1))[-1]), 10 : abs(3- self.port_speed.get((18,2))[-1]), 11: float('inf'), 12: float('inf'), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : 0, 19 : float('inf'), 20 : float('inf')},
            19 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: abs(2- self.port_speed.get((19,1))[-1]), 12: abs(3- self.port_speed.get((19,2))[-1]), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : 0, 20 : float('inf')},
            20 : { 1: float('inf'), 2: float('inf'), 3: float('inf'), 4: float('inf'), 5 : float('inf'), 6: float('inf'),7: float('inf'), 8 : float('inf'), 9 : float('inf'), 10 : float('inf'), 11: abs(2- self.port_speed.get((20,1))[-1]), 12: abs(3- self.port_speed.get((20,2))[-1]), 13: float('inf'), 14: float('inf'), 15 : float('inf'), 16: float('inf'), 17: float('inf'), 18 : float('inf'), 19 : float('inf'), 20 : 0},
          
          }
          # print(topobw)


    def install_path(self, p, ev, src_mac, dst_mac, buffer_id=None):
       print("install_path is called")
       #print "p=", p, " src_mac=", src_mac, " dst_mac=", dst_mac
       msg = ev.msg
       datapath = msg.datapath
       ofproto = datapath.ofproto
       parser = datapath.ofproto_parser
       for sw, in_port, out_port in p:
        # print(src_mac,"->", dst_mac, "via ", sw, " in_port=", in_port, " out_port=", out_port)
        match=parser.OFPMatch(in_port=in_port, eth_src=src_mac, eth_dst=dst_mac)
        actions=[parser.OFPActionOutput(out_port)]
        datapath=self.datapath_list[int(sw)-1]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS , actions)]
        if buffer_id:
          mod = datapath.ofproto_parser.OFPFlowMod( 
              datapath=datapath, buffer_id=buffer_id, match=match, idle_timeout=0, 
              hard_timeout=5,priority=1, instructions=inst)
        else:
          mod = datapath.ofproto_parser.OFPFlowMod( 
              datapath=datapath, match=match, idle_timeout=0, 
              hard_timeout=5,priority=1, instructions=inst)
        datapath.send_msg(mod)
 
    @set_ev_cls(ofp_event.EventOFPSwitchFeatures , CONFIG_DISPATCHER)
    def switch_features_handler(self , ev):
        print("switch_features_handler is called")
        datapath = ev.msg.datapath
        self.datapaths[datapath.id] = datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = datapath.ofproto_parser.OFPFlowMod(
        datapath=datapath, match=match, cookie=0,
        command=ofproto.OFPFC_ADD, idle_timeout=0, hard_timeout=0,
        priority=0, instructions=inst)
        datapath.send_msg(mod)
 
    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']
 
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        parp = pkt.get_protocol(arp.arp)
        pkt_ipv4 = pkt.get_protocol(ipv4.ipv4)
 
        #avoid broadcast from LLDP
        if eth.ethertype==35020:
          return

        dst = eth.dst
        src = eth.src
        # src_ip = parp.src_ip
        self.k.setdefault(src,{})
        dpid = datapath.id

        if parp:
          dst_ip = parp.dst_ip
          if src not in self.k.keys():
            self.k[src][dst_ip]=0

          if (src in self.k.keys() and dst_ip not in self.k[src].keys()):
            self.k[src][dst_ip]=0
        elif ipv4:
          dst_ipv4 = pkt_ipv4.dst
          self.k[src][dst_ipv4]=1000
        

        # self.logger.info("packet-in %s" % (pkt,))
        if src not in mymac.keys():
            mymac[src]=( dpid,  in_port)

        if dst in mymac.keys():
            p = get_path(mymac[src][0], mymac[dst][0], mymac[src][1], mymac[dst][1])
            
            if msg.buffer_id != ofproto.OFP_NO_BUFFER:
              self.install_path(p, ev, src, dst, msg.buffer_id)
            else:
              self.install_path(p, ev, src, dst)
            return
            # out_port = p[0][2]
            # actions = [parser.OFPActionOutput(out_port)]
            # print(out_port)
        else:
            # out_port = in_port
            out_port = ofproto.OFPP_FLOOD

            if self.k[src][dst_ip]<1000:
              self.k[src][dst_ip]+=1
              actions = [parser.OFPActionOutput(out_port)]
            else:
              actions = ''
            
            data=None
            if msg.buffer_id==ofproto.OFP_NO_BUFFER:
              data=msg.data
 
            out = parser.OFPPacketOut(
              datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,
              actions=actions, data=data)
            datapath.send_msg(out)

        
    
    @set_ev_cls(event.EventSwitchEnter)
    def get_topology_data(self, ev):
        global switches
        switch_list = get_switch(self.topology_api_app, None)  
        switches=[switch.dp.id for switch in switch_list]
        self.datapath_list=[switch.dp for switch in switch_list]
        # print("self.datapath_list=", self.datapath_list)
        print("switches=", switches)
 
        links_list = get_link(self.topology_api_app, None)
        mylinks=[(link.src.dpid,link.dst.dpid,link.src.port_no,link.dst.port_no) for link in links_list]
        for s1,s2,port1,port2 in mylinks:
          adjacency[s1][s2]=port1
          adjacency[s2][s1]=port2
          #print s1,s2,port1,port2
        


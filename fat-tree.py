#!/usr/bin/python                                                                            
                                                                                             
from unittest import result
from mininet.topo import Topo
from mininet.net import Mininet
from mininet.node import Controller, RemoteController
from mininet.util import dumpNodeConnections
from mininet.log import setLogLevel
from mininet.link import TCLink
from mininet.cli import CLI
from mininet.log import setLogLevel, info

class FatTreeTopo(Topo):
    def build( self ):


        
        # for switchNum in range(20):
        #     node = self.addSwitch( 's%s' % switchNum )
        
        # Adding all switches upfront
        switches = [ self.addSwitch( 's%s' % s )
                     for s in range( 1, 21 ) ]


        # Adding all hosts upfront
        hosts = [ self.addHost( 'h%s' % h )
                  for h in range( 1, 17 ) ]

        # Implementing the neccesary links between switches
        
        for i in range(4, 11, 2):
            self.addLink( switches[0], switches[i], cls=TCLink, bw=5)
        
        for i in range(4, 11, 2):
            self.addLink( switches[1], switches[i], cls=TCLink, bw=10) 
                
        for i in range(5, 12, 2):
            self.addLink( switches[2], switches[i], cls=TCLink, bw=5)

        for i in range(5, 12, 2):
            self.addLink( switches[3], switches[i], cls=TCLink, bw=10)  

        # 2nd tier
        
        for j in range(12,14):
            self.addLink( switches[4], switches[j], cls=TCLink, bw=2)
        
        for j in range(12,14):
            self.addLink( switches[5], switches[j], cls=TCLink, bw=3)

        for j in range(14,16):
            self.addLink( switches[6], switches[j], cls=TCLink, bw=2)
        
        for j in range(14,16):
            self.addLink( switches[7], switches[j], cls=TCLink, bw=3)

        for j in range(16,18):
            self.addLink( switches[8], switches[j], cls=TCLink, bw=2)
        
        for j in range(16,18):
            self.addLink( switches[9], switches[j], cls=TCLink, bw=3)

        for j in range(18,20):
            self.addLink( switches[10], switches[j], cls=TCLink, bw=2)
        
        for j in range(18,20):
            self.addLink( switches[11], switches[j], cls=TCLink, bw=3)

        #Implementing the neccesary links between switches and hosts
        
        
        for i in range(0,2):
            self.addLink( switches[12], hosts[i], cls=TCLink, bw=10)
        
        for i in range(2,4):
            self.addLink( switches[13], hosts[i], cls=TCLink, bw=10)

        for i in range(4,6):
            self.addLink( switches[14], hosts[i], cls=TCLink, bw=10)
        
        for i in range(6,8):
            self.addLink( switches[15], hosts[i], cls=TCLink, bw=10)

        for i in range(8,10):
            self.addLink( switches[16], hosts[i], cls=TCLink, bw=10)
        
        for i in range(10,12):
            self.addLink( switches[17], hosts[i], cls=TCLink, bw=10)

        for i in range(12,14):
            self.addLink( switches[18], hosts[i], cls=TCLink, bw=10)
        
        for i in range(14,16):
            self.addLink( switches[19], hosts[i], cls=TCLink, bw=10)

def BandwidthTest():
    results = {}
    
    topo = FatTreeTopo()

    net = Mininet(topo, build=False, link = TCLink)
    net.addController('c0',controller=RemoteController)
    
    net.start()
    print("Dumping host connections")
    dumpNodeConnections(net.hosts)
    print("Testing network connectivity")
    info( '*** Running CLI\n' )
    CLI( net )
    # net.pingAll()
    # net.stop()


if __name__ == '__main__':
    # Tell mininet to print useful information
    setLogLevel('info')
    BandwidthTest()
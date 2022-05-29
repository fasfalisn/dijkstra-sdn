# dijkstra-sdn
A Ryu-based controller built to manage traffic of a Fat-Tree topology through the shortest available paths at all times.

## How to run the topology
`sudo python fat-tree.py`

## How to run the controller
`ryu-manager dijkstra.py --observe-links`

## How it works
The controller is calculating with dijkstra algorithm the shortest route available based on hop, bandwidth or distance. In order to overcome one of the biggest problems in controllers routing on a looped topology, at first it sends broadcast messages to find and learn the topology of the hosts and as long as it knows where to find the destination the dijkstra path is created and passed to the switches. 

At the same time a thread is getting live statistics and it is calculating the available bandwidth in each route. Ensuring that the packets are always taking the fastest route. Every route gets deleted after 10 seconds and is being recalculated.
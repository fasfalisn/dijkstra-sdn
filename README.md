# dijkstra-sdn
A Ryu-based controller built to manage traffic of a Fat-Tree topology through the shortest available paths at all times.

## How to run the topology
sudo python fat-tree.py

## How to run the controller
ryu-manager dijkstra.py --observe-links

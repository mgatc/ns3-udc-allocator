# UDC Allocator

UDC Allocator is a plugin for the open-source network simulator ns3 [1], giving a class capable of constructing wireless coverage regions fast. The underlying algorithms, solving the two-dimensional Unit Disk Cover (UDC) problem, were chosen after careful analysis in [2]. The example given shows the creation of a LoRa network with sensors placed randomly and measures the performance of networks with gateways placed by different UDC algorithms. Performance is measured by number of gateways placed and packet reception ratio.

---------------------
Dependencies
---------------------

1.	ns3 [1]
2.	CGAL [3]



---------------------
References
---------------------
[1] https://www.nsnam.org/

[2] https://doi.org/10.1007/978-3-030-34029-2_10

[3] https://www.cgal.org/

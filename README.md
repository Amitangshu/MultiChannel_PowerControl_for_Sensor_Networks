# MultiChannel_PowerControl_for_Sensor_Networks

Overview:
We consider large-scale WSNs for data collection applications, where implementation of network-wide time synchronization is a significant challenge. Hence, it is difficult to apply synchronized duty cycling and scheduled transmissions in such networks, which are critical for avoiding energy wastage from overhearing. The complexity of this energy optimization problem in sensor networks arises due to the fact that it has to be addressed by network wide adaptations as opposed to independent adaptations at the nodes. The objective of this research is to design networking protocols that control the energy consumption at the nodes to adapt to such spatial and temporal variations of energy resources. Key components of this research include (a) the development of energy availability models for rechargeable networks which involves energy source prediction and energy storage modeling, and (b) the development of adaptive networking protocols that allow variable energy consumption where we have explored joint power control and routing protocols, multi-channel routing protocols, adaptive duty-cycling, and event-based sampling schemes. The adaptive schemes have also been tested and validated on real testbed using MICAz sensor motes. 


Description:
The repository consists of files that are written in Tinyos2.x. These are mainly the modifed version of Collection Tree Protocol (CTP), along with the aother adaptive and fully distributed features like multi-channel routing and transmit power control for overhearing minimization in asynchronous sensor networks. The files corresponding to channel selection and routing is written with an extension of "_multichannel.nc" whereas those corresponding to transmit power control and routing are written with an extension of "_powercontrol.nc".


Related Publications:

1. Amitangshu Pal, and Asis Nasipuri, "A Joint Power Control and Routing Adaptation Scheme for Rechargeable Sensor Networks", in preparation, Available at https://astro.temple.edu/~tuf79640/pc_routing_long_version.pdf.

2. Amitangshu Pal, and Asis Nasipuri, "Distributed Routing and Channel Selection for Multi-ChannelWireless Sensor Networks", MDPI Journal of Sensor and Actuator Networks, Vol. 6, No. 10, pp 1-18, 2017.

3. Amitangshu Pal and Asis Nasipuri, "Lifetime of Asynchronous Wireless Sensor Networks with Multiple Channels and Power Control", In Proc. IEEE WCNC 2014, Istanbul, Turkey.

4. Amitangshu Pal and Asis Nasipuri, "PCOR: A Joint Power Control and Routing Scheme for Rechargeable Sensor Networks", In Proc. IEEE WCNC 2014, Istanbul, Turkey.

5. Amitangshu Pal and Asis Nasipuri, "DRCS: A Distributed Routing and Channel Selection Scheme for Multi-Channel Wireless Sensor Networks", In Proc. IEEE PerSeNS 2013, pp. 602-608, San Diego, California, USA.

6. Amitangshu Pal, Bonee Soibam and Asis Nasipuri, "A Distributed Power Control and Routing Scheme for Rechargeable Sensor Networks", In Proc. IEEE SoutheastCon 2013, Jacksonville, Florida, USA.

7. Amitangshu Pal and Asis Nasipuri, "A Distributed Channel Selection Scheme for Multi-Channel Wireless Sensor Networks", In Proc. ACM MobiHoc 2012, pp. 263 - 264, Hilton Head Island, SC, USA. 

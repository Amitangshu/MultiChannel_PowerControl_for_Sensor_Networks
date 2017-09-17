#include <Timer.h>
#include <TreeRouting.h>
#include <CollectionDebugMsg.h>
//#include "printf.h"



#define MIN_POWER_VAL 9

#define MIN_ETX_LIMIT 15
#define MAX_ETX_LIMIT 20
/* $Id: CtpRoutingEngineP.nc,v 1.25 2010-06-29 22:07:49 scipio Exp $ */
/*
 * Copyright (c) 2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** 
 *  The TreeRoutingEngine is responsible for computing the routes for
 *  collection.  It builds a set of trees rooted at specific nodes (roots) and
 *  maintains these trees using information provided by the link estimator on
 *  the quality of one hop links.
 * 
 *  <p>Each node is part of only one tree at any given time, but there is no
 *  difference from the node's point of view of which tree it is part. In other
 *  words, a message is sent towards <i>a</i> root, but which one is not
 *  specified. It is assumed that the roots will work together to have all data
 *  aggregated later if need be.  The tree routing engine's responsibility is
 *  for each node to find the path with the least number of transmissions to
 *  any one root.
 *
 *  <p>The tree is proactively maintained by periodic beacons sent by each
 *  node. These beacons are jittered in time to prevent synchronizations in the
 *  network. All nodes maintain the same <i>average</i> beacon sending rate
 *  (defined by BEACON_INTERVAL +- 50%). The beacon contains the node's parent,
 *  the current hopcount, and the cumulative path quality metric. The metric is
 *  defined as the parent's metric plus the bidirectional quality of the link
 *  between the current node and its parent.  The metric represents the
 *  expected number of transmissions along the path to the root, and is 0 by
 *  definition at the root.
 * 
 *  <p>Every time a node receives an update from a neighbor it records the
 *  information if the node is part of the neighbor table. The neighbor table
 *  keeps the best candidates for being parents i.e., the nodes with the best
 *  path metric. The neighbor table does not store the full path metric,
 *  though. It stores the parent's path metric, and the link quality to the
 *  parent is only added when the information is needed: (i) when choosing a
 *  parent and (ii) when choosing a route. The nodes in the neighbor table are
 *  a subset of the nodes in the link estimator table, as a node is not
 *  admitted in the neighbor table with an estimate of infinity.
 * 
 *  <p>There are two uses for the neighbor table, as mentioned above. The first
 *  one is to select a parent. The parent is just the neighbor with the best
 *  path metric. It serves to define the node's own path metric and hopcount,
 *  and the set of child-parent links is what defines the tree. In a sense the
 *  tree is defined to form a coherent propagation substrate for the path
 *  metrics. The parent is (re)-selected periodically, immediately before a
 *  node sends its own beacon, in the updateRouteTask.
 *  
 *  <p>The second use is to actually choose a next hop towards any root at
 *  message forwarding time.  This need not be the current parent, even though
 *  it is currently implemented as such.
 *
 *  <p>The operation of the routing engine has two main tasks and one main
 *  event: updateRouteTask is called periodically and chooses a new parent;
 *  sendBeaconTask broadcasts the current route information to the neighbors.
 *  The main event is the receiving of a neighbor's beacon, which updates the
 *  neighbor table.
 *  
 *  <p> The interface with the ForwardingEngine occurs through the nextHop()
 *  call.
 * 
 *  <p> Any node can become a root, and routed messages from a subset of the
 *  network will be routed towards it. The RootControl interface allows
 *  setting, unsetting, and querying the root state of a node. By convention,
 *  when a node is root its hopcount and metric are 0, and the parent is
 *  itself. A root always has a valid route, to itself.
 *
 *  @author Rodrigo Fonseca
 *  @author Philip Levis (added trickle-like updates)
 *  Acknowledgment: based on MintRoute, MultiHopLQI, BVR tree construction, Berkeley's MTree
 *                           
 *  @date   $Date: 2010-06-29 22:07:49 $
 *  @see Net2-WG
 */

generic module CtpRoutingEngineP(uint8_t routingTableSize, uint32_t minInterval, uint32_t maxInterval) {
    provides {
        interface UnicastNameFreeRouting as Routing;
        interface RootControl;
        interface CtpInfo;
        interface StdControl;
        interface CtpRoutingPacket;
        interface Init;
    } 
    uses {
        interface AMSend as BeaconSend;
        interface Receive as BeaconReceive;
        interface LinkEstimator;
        interface AMPacket;
        interface SplitControl as RadioControl;
        interface Timer<TMilli> as BeaconTimer;
        interface Timer<TMilli> as RouteTimer;
	interface Timer<TMilli> as Timer_for_lifetime_calc;
	interface Timer<TMilli> as Timer_for_Power_Control;
        interface Random;
        interface CollectionDebug;
        interface CtpCongestion;
	interface GetNow<uint16_t> as GetVoltageData; //mod

	interface CompareBit;
	interface CC2420Packet;
 	//interface Leds;

    }
}


implementation {

    bool ECNOff = TRUE;

    /* Keeps track of whether the radio is on. No sense updating or sending
     * beacons if radio is off */
    bool radioOn = FALSE;
    /* Controls whether the node's periodic timer will fire. The node will not
     * send any beacon, and will not update the route. Start and stop control this. */
    bool running = FALSE;
    /* Guards the beacon buffer: only one beacon being sent at a time */
    bool sending = FALSE;

    /* Tells updateNeighbor that the parent was just evicted.*/ 
    bool justEvicted = FALSE;


    route_info_t routeInfo;
    bool state_is_root;
    am_addr_t my_ll_addr;
    double POS_THRESHOLD=0.75;
    uint16_t REC_NEIGH_LIFE;
    uint8_t red_pow=0;
    bool badnode=0, from_route_timer, I_am_in_shade;
    uint8_t count_routing_table=0;
    uint8_t ETX_MAX=99;
    uint16_t P, max_P, REC_P;
    uint16_t Q, LOAD;
    uint16_t Q_pathmetric, store_current_forward_etx=0, which_one=0;
    uint8_t count_predicted_quality=0;
    uint8_t count_for_5_min=0;

    message_t beaconMsgBuffer;
    ctp_routing_header_t* beaconMsg;

    /* routing table -- routing info about neighbors */
    routing_table_entry routingTable[routingTableSize];
    uint8_t routingTableActive;

    /* statistics */
    uint32_t parentChanges;
    /* end statistics */

    // forward declarations
    void routingTableInit();
    uint8_t routingTableFind(am_addr_t);
    error_t routingTableUpdateEntry(am_addr_t, am_addr_t , uint16_t, uint8_t);
    error_t routingTableEvict(am_addr_t neighbor);



  /* 
     For each interval t, you set a timer to fire between t/2 and t
     (chooseAdvertiseTime), and you wait until t (remainingInterval). Once
     you are at t, you double the interval (decayInterval) if you haven't
     reached the max. For reasons such as topological inconsistency, you
     reset the timer to a small value (resetInterval).
  */


    uint32_t currentInterval = minInterval;
    uint32_t t; 
    bool tHasPassed;

    void chooseAdvertiseTime() {
       t = currentInterval;
       t /= 2;
       t += call Random.rand32() % t;
       tHasPassed = FALSE;
       call BeaconTimer.startOneShot(t);
    }

    void resetInterval() {
      currentInterval = minInterval;
      chooseAdvertiseTime();
    }

    void decayInterval() {
        currentInterval *= 2;
        if (currentInterval > maxInterval) {
          currentInterval = maxInterval;
        }
      chooseAdvertiseTime();
    }

    void remainingInterval() {
       uint32_t remaining = currentInterval;
       remaining -= t;
       tHasPassed = TRUE;
       call BeaconTimer.startOneShot(remaining);
    }
    
    /* Converts the output of the link estimator to path metric
 * units, that can be *added* to form path metric measures */
	uint16_t evaluateEtx(uint16_t quality, uint16_t nodeid) {
		uint8_t i;
		routing_table_entry* entry;
		for (i = 0; i < routingTableActive; i++) {
			entry = &routingTable[i];
			if(entry->neighbor == nodeid){	
				if(entry->info.route_quality_send != 99){	//printf("\rYES99	%d\n", entry->info.route_quality_send);
					return entry->info.route_quality_send;
				}
			}
		}
		return (quality /*+ 10*/);
	}

    command error_t Init.init() {
        uint8_t maxLength;
        radioOn = FALSE;
        running = FALSE;
        parentChanges = 0;
        state_is_root = 0;
        routeInfoInit(&routeInfo);
        routingTableInit();
        beaconMsg = call BeaconSend.getPayload(&beaconMsgBuffer, call BeaconSend.maxPayloadLength());
        maxLength = call BeaconSend.maxPayloadLength();
        //dbg("TreeRoutingCtl","TreeRouting initialized. (used payload:%d max payload:%d!\n", 
        //      sizeof(beaconMsg), maxLength);
        return SUCCESS;
    }

    command error_t StdControl.start() {
      my_ll_addr = call AMPacket.address();
      //start will (re)start the sending of messages
      if (!running) {
	running = TRUE;
	resetInterval();
	call RouteTimer.startPeriodic(BEACON_INTERVAL);
	call Timer_for_lifetime_calc.startPeriodic(60000);
	call Timer_for_Power_Control.startPeriodic(600000);//kept for power control that gave good result
	//call Timer_for_Power_Control.startPeriodic(1200000);
	//dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
	Life=call GetVoltageData.getNow();
	if(Life==0)	Life=999;
      }     
      return SUCCESS;
    }

int battery_life_cal(){
	int v;
	double pc;
	double soc, I_prev;
        int Life_prev;

	Life_prev=Life;
	I_prev=I;


	I = (0.001*rate_RUI_p*I_Rt*T_Rt + 0.001*rate_FWD_p*I_Dt*T_Dt + 0.001*rate_RUI_RECV_p*I_Rt*T_Rt + 0.001*(rate_OVERHEAR_p+rate_RCV_p)*I_Dt*T_Dt + 
	0.001*rate_SENSE_p*I_s*T_s+ 0.001*8*I_p*T_p*60);

	if(Life_prev != 999)	I = 0.5*I+0.5*I_prev;

	LOAD = rate_FWD_p;
	v = call GetVoltageData.getNow();
	if(v>=482)	v=481;
	pc = 1*(482- v)/(double)65; //Added for lifetime calc

	if(pc>1)	pc=1;
	if(I!=0)	soc=(50*pc*60)/(double)(24*I*1); // Battery lifetime is calculated every once in a minute.
	soc=soc*100;		// Broken up into 2 lines; to avoid the val going > 2^16 (converted in days)	

	Life=(int)soc;

	percent_cap=(int)(pc*100);
	

	if(TOS_NODE_ID <= 20 && TOS_NODE_ID >=0)	Life = 480;
	if(TOS_NODE_ID <=160 && TOS_NODE_ID >=150)	Life = 480;


	if(TOS_NODE_ID == 156)	Life = 95;	//Highly critical
	if(TOS_NODE_ID == 1)	Life = 95;	//Less critical	

	return Life; //Working

}

	event void Timer_for_lifetime_calc.fired()
	{

		rate_RUI_p = rate_RUI;
		rate_FWD_p = rate_FWD;
		rate_RUI_RECV_p = rate_RUI_RECV;
		rate_OVERHEAR_p = rate_OVERHEAR;
		rate_SENSE_p=rate_SENSE;
		rate_RCV_p=rate_RCV;

		rate_RUI = 0;
		rate_FWD = 0;
		rate_RUI_RECV = 0;
		rate_OVERHEAR = 0;
		rate_SENSE=0;
		rate_RCV=0;

		count_for_5_min++;
		if(count_for_5_min == 5){ //kept for power control that gave good result
			store_rate_OVERHEAR_5_min = rate_OVERHEAR_5_min;	/*store_rate_OVERHEAR_5_min = 77;*/		rate_OVERHEAR_5_min=0;
			count_for_5_min=0;
		}
		/*Life=*/battery_life_cal();
	}


    command error_t StdControl.stop() {
        running = FALSE;
        //dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
        return SUCCESS;
    } 

    event void RadioControl.startDone(error_t error) {
        radioOn = TRUE;
        //dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
        if (running) {
            uint16_t nextInt;
            nextInt = call Random.rand16() % BEACON_INTERVAL;
            nextInt += BEACON_INTERVAL >> 1;
        }
    } 

    event void RadioControl.stopDone(error_t error) {
        radioOn = FALSE;
        //dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
    }

    /* Is this quality measure better than the minimum threshold? */
    // Implemented assuming quality is EETX
    bool passLinkEtxThreshold(uint16_t etx) {
        return (etx < ETX_THRESHOLD);
    }


double predicted_quality(void){
	
	int i, j, m=0, count_mini;
	double sum_t=0, sum_t2=0,sum_p=0, sum_tp=0;
	double a, b, mini;

	routing_table_entry* entry;
	for (j = 0; j < routingTableActive; j++) {
		entry = &routingTable[j];

		mini = 999;	m=0;
		sum_t=0; sum_t2=0; sum_p=0; sum_tp=0;	count_mini = 0;

		for(i=0;i < MAX_NUMBER_POWER_LEVEL_ENTRY;i++){
			if(entry->info.t_array[i]!=0 /*&& entry->info.t_array[i]!=0.0001*/ && entry->info.pos[i]!=99/*&& i!=power_level*/){
				if(entry->info.pos[i]==1)	entry->info.pos[i] = 0.999;
				m++;
				sum_t=sum_t + entry->info.t_array[i];
				sum_t2 = sum_t2 + entry->info.t_array[i]*entry->info.t_array[i];
				sum_p = sum_p + log(entry->info.pos[i]/((double)(1-entry->info.pos[i])));
				//sum_tp = sum_tp + (log(entry->info.pos[i]/((double)(1-entry->info.pos[i])))) * (log(entry->info.pos[i]/((double)(1-entry->info.pos[i]))));
				sum_tp = sum_tp + entry->info.t_array[i]*(log(entry->info.pos[i]/((double)(1-entry->info.pos[i]))));

				if(entry->info.pos[i] < mini)	mini=entry->info.pos[i];
				if(entry->info.pos[i] < 0.75)	count_mini++;
			}
		}

		if(m>=6 /*&& mini < 0.75*/ && count_mini>=2){			//Checking for minimum number of points; To get enough confidence
			a = ( sum_p/((double)m) - sum_tp/((double)sum_t) )/(double)(( sum_t/((double)m) - sum_t2/((double)sum_t) ));
			b = sum_p/((double)m) - sum_t*a/((double)m);		//Linear best fit prediction model
		}
		else {	a=99;	b=99;	}
		entry->info.a = a; entry->info.b = b;					//Coefficients for linear best fit
	}

}

void event_Timer_for_Power_Control_fired(){
		int i, kkk, dest;
		uint16_t r1;
		double power_float;
		double PL[32];
		routing_table_entry* entry;	

		dest = routeInfo.parent;

		r1 = (uint16_t)(100*(rand()/(double)RAND_MAX));
				
		for (i = 0; i < routingTableActive; i++) {
			entry = &routingTable[i];
			if(entry->neighbor == dest)
				break;
		}
		
		if(entry->info.a_send != 99)	entry->info.confidence = 1;
		PL[31] = 0.0001;	PL[30] = -9.14E+02;	PL[29] = -3.01E+03; 	PL[28] = -6.10E+03;	PL[27] = -1.00E+04;	PL[26] = -1.45E+04;
		PL[25] = -1.95E+04;	PL[24] = -2.47E+04;	PL[23] = -3.00E+04;	PL[22] = -3.52E+04;	PL[21] = -4.03E+04;	PL[20] = -4.52E+04;
		PL[19] = -5.00E+04;	PL[18] = -5.47E+04;	PL[17] = -5.94E+04;	PL[16] = -6.44E+04;	PL[15] = -7.00E+04;
		PL[14] = -7.63E+04;	PL[13] = -8.33E+04;	PL[12] = -9.12E+04;	PL[11] = -1.00E+05;	PL[10] = -1.10E+05;
		PL[9] = -1.21E+05;	PL[8] = -1.34E+05;	PL[7] = -1.50E+05;	PL[6] = -1.69E+05;	PL[5] = -1.92E+05;
		PL[4] = -2.18E+05;	PL[3] = -2.50E+05;	PL[2] = -2.87E+05;	PL[1] = -3.30E+05;	PL[0] = -3.79E+05; 
		
		if	(( (entry->info.route_quality_send > MAX_ETX_LIMIT && entry->info.route_quality_send != 99)
			|| (entry->info.route_quality_send == 99 && call LinkEstimator.getLinkQuality(entry->neighbor) >= MAX_ETX_LIMIT ) 
			|| call LinkEstimator.get_Forward_LinkQuality(entry->neighbor) > MAX_ETX_LIMIT /*|| consecutive_failure > 5*/) && power<=MAX_POWER_VAL-step_pc) {
			power=power+step_pc;
		}
		
		else if (red_pow==1 && badnode!=1 && /*REC_NEIGH_LIFE!=dest &&*/ (entry->info.route_quality_send <=MIN_ETX_LIMIT || (entry->info.route_quality_send == 99 && call LinkEstimator.getLinkQuality(entry->neighbor) <=MIN_ETX_LIMIT && call LinkEstimator.get_Forward_LinkQuality(entry->neighbor) <=MIN_ETX_LIMIT )) && power>=MIN_POWER_VAL+step_pc && r1 < max_P){
			    
			if((entry->info.a_send == 99 || entry->info.a_send == 0) && power >= MIN_POWER_VAL+step_pc && (entry->info.route_quality_send <=MIN_ETX_LIMIT || (entry->info.route_quality_send == 99 && call LinkEstimator.getLinkQuality(entry->neighbor) <=MIN_ETX_LIMIT )) )	{
				power = power - step_pc;
			}
			else if(/*Q !=0 &&*/ /*r1/(double)100 > 10/(double)Q*/ /*entry->info.a !=99 &&*/ entry->info.a_send != 99 && r1 < REC_P && entry->info.a_send != 0){	

				power_float = (log(POS_THRESHOLD/(1-POS_THRESHOLD)) - entry->info.b_send)/(double)(entry->info.a_send);   //not in int,,, in dbm

				for(kkk=MIN_POWER_VAL;kkk<MAX_POWER_VAL;kkk=kkk+step_pc){
					if(power_float >= PL[kkk]/(double)10000 && power_float <= PL[kkk+step_pc]/(double)10000){
						power = kkk+step_pc;
						break;
					}
				}
				if(power > MAX_POWER_VAL)	power = MAX_POWER_VAL;
				else if (power < MIN_POWER_VAL)	power = MIN_POWER_VAL;

			}
		}
		
		if(power > MAX_POWER_VAL)	power = MAX_POWER_VAL;
}


event void Timer_for_Power_Control.fired(){
		int i, kkk, dest;
		uint16_t r1;
		double PL[32];
		double power_float;
		routing_table_entry* entry;	

		dest = routeInfo.parent;

		r1 = (uint16_t)(100*(rand()/(double)RAND_MAX));
				
		
		for (i = 0; i < routingTableActive; i++) {
			entry = &routingTable[i];
			if(entry->neighbor == dest)
				break;
		}
		
		if(entry->info.a_send != 99)	entry->info.confidence = 1;
		PL[31] = 0.0001;	PL[30] = -9.14E+02;	PL[29] = -3.01E+03; 	PL[28] = -6.10E+03;	PL[27] = -1.00E+04;	PL[26] = -1.45E+04;
		PL[25] = -1.95E+04;	PL[24] = -2.47E+04;	PL[23] = -3.00E+04;	PL[22] = -3.52E+04;	PL[21] = -4.03E+04;	PL[20] = -4.52E+04;
		PL[19] = -5.00E+04;	PL[18] = -5.47E+04;	PL[17] = -5.94E+04;	PL[16] = -6.44E+04;	PL[15] = -7.00E+04;
		PL[14] = -7.63E+04;	PL[13] = -8.33E+04;	PL[12] = -9.12E+04;	PL[11] = -1.00E+05;	PL[10] = -1.10E+05;
		PL[9] = -1.21E+05;	PL[8] = -1.34E+05;	PL[7] = -1.50E+05;	PL[6] = -1.69E+05;	PL[5] = -1.92E+05;			// From https://www.cs.wmich.edu/gupta/teaching/cs5950/fall2011/mica%20pinouts%20from%20memsic%20mpr-mib_series_users_manual%207430-0021-09_a-t.pdf
		PL[4] = -2.18E+05;	PL[3] = -2.50E+05;	PL[2] = -2.87E+05;	PL[1] = -3.30E+05;	PL[0] = -3.79E+05; 			//Power levels to dBm; Multiplied by 10^4; Only changed for P[31] to avoid divide by 0 (http://mail.millennium.berkeley.edu/pipermail/tinyos-help/2008-February/031385.html)
		

		if	(( (entry->info.route_quality_send > MAX_ETX_LIMIT && entry->info.route_quality_send != 99)
			|| (entry->info.route_quality_send == 99 && call LinkEstimator.getLinkQuality(entry->neighbor) >= MAX_ETX_LIMIT ) 
			|| call LinkEstimator.get_Forward_LinkQuality(entry->neighbor) > MAX_ETX_LIMIT /*|| consecutive_failure > 5*/) && power<=MAX_POWER_VAL-step_pc) {
			power=power+step_pc;
		}
		
		else if (red_pow==1 && badnode!=1 && /*REC_NEIGH_LIFE!=dest &&*/ (entry->info.route_quality_send <=MIN_ETX_LIMIT || (entry->info.route_quality_send == 99 && call LinkEstimator.getLinkQuality(entry->neighbor) <=MIN_ETX_LIMIT && call LinkEstimator.get_Forward_LinkQuality(entry->neighbor) <=MIN_ETX_LIMIT  )) && power>=MIN_POWER_VAL+step_pc && r1 < max_P){

			if((entry->info.a_send == 99 || entry->info.a_send == 0) && power >= MIN_POWER_VAL+step_pc && (entry->info.route_quality_send <=MIN_ETX_LIMIT || (entry->info.route_quality_send == 99 && call LinkEstimator.getLinkQuality(entry->neighbor) <=MIN_ETX_LIMIT )))	{
				power = power - step_pc;
			}
			else if(/*Q !=0 &&*/ /*r1/(double)100 > 10/(double)Q*/ /*entry->info.a !=99 &&*/ entry->info.a_send != 99 && r1 < REC_P && entry->info.a_send != 0){	
				power_float = (log(POS_THRESHOLD/(1-POS_THRESHOLD)) - entry->info.b_send)/(double)(entry->info.a_send);   //not in int,,, in dbm
				for(kkk=MIN_POWER_VAL;kkk<MAX_POWER_VAL;kkk=kkk+step_pc){
					if(power_float >= PL[kkk]/(double)10000 && power_float <= PL[kkk+step_pc]/(double)10000){
						power = kkk+step_pc;
						break;
					}
				}
				if(power > MAX_POWER_VAL)	power = MAX_POWER_VAL;
				else if (power < MIN_POWER_VAL)	power = MIN_POWER_VAL;
			}
		}
		
		if(power > MAX_POWER_VAL)	power = MAX_POWER_VAL;
		
}


    /* updates the routing information, using the info that has been received
     * from neighbor beacons. Two things can cause this info to change: 
     * neighbor beacons, changes in link estimates, including neighbor eviction */
    task void updateRouteTask() {
        uint8_t i, j, k;
        routing_table_entry* entry;
        routing_table_entry* best;
	routing_table_entry* best_q;
	routing_table_entry* entry_min;
        uint16_t minEtx, minEtx_q = 5500, m_Qpathmetric=999, Q_pathmetric_t;
        uint16_t currentEtx;
        uint16_t linkEtx, pathEtx;
	uint16_t minlinkEtx_q, minEtx_ctp, minEtx_q_ctp;

	int found, fnd;
	double sum_life, prob, power_float;
	double minQ = 999, linkEtxthresh = 0.5;
	int count_sum;
	double PL[32], mean_life;

	if(TOS_NODE_ID == 0){
		count_predicted_quality++;
		if(count_predicted_quality == 30){
			count_predicted_quality=0;
			predicted_quality();
		}
	}

	//already_in_update_route_task=1;
	//printf("\n\nupdate route task\n");	printfflush();
	found=0, fnd=0;
	entry_min=NULL;

	
		
		PL[31] = 0.0001;	PL[30] = -9.14E+02;	PL[29] = -3.01E+03; 	PL[28] = -6.10E+03;	PL[27] = -1.00E+04;	PL[26] = -1.45E+04;
		PL[25] = -1.95E+04;	PL[24] = -2.47E+04;	PL[23] = -3.00E+04;	PL[22] = -3.52E+04;	PL[21] = -4.03E+04;	PL[20] = -4.52E+04;
		PL[19] = -5.00E+04;	PL[18] = -5.47E+04;	PL[17] = -5.94E+04;	PL[16] = -6.44E+04;	PL[15] = -7.00E+04;
		PL[14] = -7.63E+04;	PL[13] = -8.33E+04;	PL[12] = -9.12E+04;	PL[11] = -1.00E+05;	PL[10] = -1.10E+05;
		PL[9] = -1.21E+05;	PL[8] = -1.34E+05;	PL[7] = -1.50E+05;	PL[6] = -1.69E+05;	PL[5] = -1.92E+05;
		PL[4] = -2.18E+05;	PL[3] = -2.50E+05;	PL[2] = -2.87E+05;	PL[1] = -3.30E+05;	PL[0] = -3.79E+05;
	
	sum_life=0;
	count_sum=0;
	max_P=0;	REC_NEIGH_LIFE=999;

        if (state_is_root)
            return;
       
        best = NULL;
        /* Minimum etx found among neighbors, initially infinity */
        minEtx = MAX_METRIC;		minEtx_ctp = MAX_METRIC;		minEtx_q_ctp = MAX_METRIC;
        /* Metric through current parent, initially infinity */
        currentEtx = MAX_METRIC;

        //dbg("TreeRouting","%s\n",__FUNCTION__);

        /* Find best path in table, other than our current */
        for (i = 0; i < routingTableActive; i++) {
            entry = &routingTable[i];


	    if(entry->neighbor!=0 && entry->info.Life_record != 999){
				sum_life=sum_life + entry->info.Life_record;

				count_sum++;
	    }

	    if(entry->info.P_record > max_P /*&& entry->neighbor!=0 && entry->info.route_quality_send!=99*/){  //Raw val is inversely proportional
				max_P = entry->info.P_record;
				REC_NEIGH_LIFE = entry->neighbor;
				REC_P= max_P;
		}
	

            // Avoid bad entries and 1-hop loops
            if (entry->info.parent == INVALID_ADDR || entry->info.parent == my_ll_addr) {
              /*dbg("TreeRouting", 
                  "routingTable[%d]: neighbor: [id: %d parent: %d  etx: NO ROUTE]\n",  
                  i, entry->neighbor, entry->info.parent);*/
              continue;
            }

            linkEtx = call LinkEstimator.getLinkQuality(entry->neighbor);
            /*dbg("TreeRouting", 
                "routingTable[%d]: neighbor: [id: %d parent: %d etx: %d retx: %d]\n",  
                i, entry->neighbor, entry->info.parent, linkEtx, entry->info.etx);*/
            pathEtx = linkEtx + entry->info.etx;

	    

            /* Operations specific to the current parent */
            if (entry->neighbor == routeInfo.parent) {
                //dbg("TreeRouting", "   already parent.\n");
                currentEtx = pathEtx;
                /* update routeInfo with parent's current info */
		routeInfo.etx = entry->info.etx;
		routeInfo.congested = entry->info.congested;
		routeInfo.reduce_power = entry->info.reduce_power;
		if(best == NULL)	best=entry;
                continue;
            }
            /* Ignore links that are congested */
            if (entry->info.congested)
                continue;
            /* Ignore links that are bad */
            if (!passLinkEtxThreshold(linkEtx)) {
              //dbg("TreeRouting", "   did not pass threshold.\n");
              continue;
            }
            
            if (pathEtx < minEtx) {
	      //dbg("TreeRouting", "   best is %d, setting to %d\n", pathEtx, entry->neighbor);
                minEtx = pathEtx;	minEtx_ctp = pathEtx;
                best = entry;
            }  
        }

	for (j = 0; j < routingTableActive; j++) {
		entry = &routingTable[j];
		if(REC_NEIGH_LIFE == 999 /*|| entry->info.route_quality_send == 99*/)	{	Q=0;	break; 	}   // Q is the prob of overhear
		if(entry->neighbor == REC_NEIGH_LIFE)	{	entry_min = entry;	Q = entry->info.route_quality_send;	
			break; }
	}

	minEtx_q = 55000;

	for (k = 0; k < routingTableActive; k++) {
		entry = &routingTable[k];

		if(entry->info.a_send != 99 && REC_NEIGH_LIFE != 999 && entry->info.a_send !=0 && entry_min->info.a_send != 99){
			power_float = (log(POS_THRESHOLD/(1-POS_THRESHOLD)) - entry->info.b_send)/(double)(entry->info.a_send);   //not in int,,, in dbm
			Q = (uint16_t)(100/(double)(1+exp(-(entry_min->info.a_send*power_float + entry_min->info.b_send))));
		}
		
		if(Q!=0 && Q!=99 /*&& Q<1000*/){
			Q=(uint16_t)(Q*LOAD/(double)(10));  //Extra 10 multiplied in the numerator for extra accuracy
		}
		else {
			Q=0;
		}
		
		linkEtx = evaluateEtx(call LinkEstimator.getLinkQuality(entry->neighbor), entry->neighbor);
		pathEtx = linkEtx + entry->info.etx;
		Q_pathmetric_t = Q + entry->info.Q_patharray;

		
		if(Q_pathmetric_t <= m_Qpathmetric && entry->info.etx < routeInfo.etx + call LinkEstimator.getLinkQuality(routeInfo.parent) /*&& linkEtx < 30*/ && call LinkEstimator.getLinkQuality(entry->neighbor) < 15){
			if(Q_pathmetric_t == m_Qpathmetric){ 
				if(pathEtx < minEtx_q){ 
					m_Qpathmetric = Q_pathmetric_t;
					best_q = entry;
					minEtx_q = pathEtx;	minEtx_q_ctp = call LinkEstimator.getLinkQuality(entry->neighbor) + entry->info.etx;
					minlinkEtx_q = linkEtx;
				}
			}
			else{
				m_Qpathmetric = Q_pathmetric_t;
				best_q = entry;
				minEtx_q = pathEtx;	minEtx_q_ctp = call LinkEstimator.getLinkQuality(entry->neighbor) + entry->info.etx;
				minlinkEtx_q = linkEtx;
			}
		}

		
	}


	if(m_Qpathmetric != 999 /*&& minQ!=999*/ /*&& minEtx_q!=99 && m_Qpathmetric != 0*/){	
		if(best!=NULL){		
			if(best->neighbor != 0 /*&& m_Qpathmetric != 0*/ && minlinkEtx_q < MIN_ETX_LIMIT && best_q!=NULL && call LinkEstimator.getLinkQuality(best_q->neighbor) < 15 && best_q->info.etx < routeInfo.etx + call LinkEstimator.getLinkQuality(routeInfo.parent) && best_q->info.parent != my_ll_addr && minEtx_q_ctp < minEtx_ctp + 5){
				best = best_q;		minEtx = minEtx_q;	
			}
		}
	}


        /* Now choose between the current parent and the best neighbor */
        /* Requires that: 
            1. at least another neighbor was found with ok quality and not congested
            2. the current parent is congested and the other best route is at least as good
            3. or the current parent is not congested and the neighbor quality is better by 
               the PARENT_SWITCH_THRESHOLD.
          Note: if our parent is congested, in order to avoid forming loops, we try to select
                a node which is not a descendent of our parent. routeInfo.ext is our parent's
                etx. Any descendent will be at least that + 10 (1 hop), so we restrict the 
                selection to be less than that.
        */
        if (minEtx != MAX_METRIC) {
            if (currentEtx == MAX_METRIC ||
                (routeInfo.congested && (minEtx < (routeInfo.etx + 10))) ||
                minEtx + 5 < currentEtx) {
	//if(best!=NULL){
		//if(best->neighbor != routeInfo.parent) { 
                // routeInfo.metric will not store the composed metric.
                // since the linkMetric may change, we will compose whenever
                // we need it: i. when choosing a parent (here); 
                //            ii. when choosing a next hop
                if(best->neighbor != routeInfo.parent)	parentChanges++;

                dbg("TreeRouting","Changed parent. from %d to %d\n", routeInfo.parent, best->neighbor);
                call CollectionDebug.logEventDbg(NET_C_TREE_NEW_PARENT, best->neighbor, best->info.etx, minEtx);
                if(routeInfo.reduce_power != 1)	call LinkEstimator.unpinNeighbor(routeInfo.parent);
                call LinkEstimator.pinNeighbor(best->neighbor);
                call LinkEstimator.clearDLQ(best->neighbor);

		if(best->info.confidence==1 || best->info.a_send != 99/*&& minlinkEtx_q < 25*/)	{	
			event_Timer_for_Power_Control_fired();
		}
		routeInfo.parent = best->neighbor;
		routeInfo.etx = best->info.etx;
		routeInfo.congested = best->info.congested;
		routeInfo.reduce_power = best->info.reduce_power;

		store_current_forward_etx = best->info.route_quality_send;
		Q_pathmetric = Q + best->info.Q_patharray;	

		if (currentEtx - minEtx > 20) {
		  call CtpInfo.triggerRouteUpdate();
		}
            }
        }    

        /* Finally, tell people what happened:  */
        /* We can only loose a route to a parent if it has been evicted. If it hasn't 
         * been just evicted then we already did not have a route */
        if (justEvicted && routeInfo.parent == INVALID_ADDR) 
            signal Routing.noRoute();
        /* On the other hand, if we didn't have a parent (no currentEtx) and now we
         * do, then we signal route found. The exception is if we just evicted the 
         * parent and immediately found a replacement route: we don't signal in this 
         * case */
        else if (!justEvicted && 
                  currentEtx == MAX_METRIC &&
                  minEtx != MAX_METRIC)
            signal Routing.routeFound();
        justEvicted = FALSE; 

	
	if(count_sum==0){	P=0; goto BBB;}	
	mean_life=sum_life/count_sum;
	if(Life<0.5*mean_life*1) { badnode=1;	/*printf("\rbad_ID=%d	Life=%d	mean_life=%f\n",TOS_NODE_ID,Life, mean_life);*/ }
	else badnode=0;

	if(mean_life==0 || Life==999 || mean_life==999){	P=0; goto BBB;}	
	if(0.5*mean_life>Life){
		   prob=(mean_life-Life)/((double)(mean_life)); //printf("\rSmelling Bad?	%f\n", prob);
		   P=(uint16_t)(100*prob);
	}
	else 	P=0;


	BBB:
	;
	count_predicted_quality++;
	if(count_predicted_quality == 30){
		count_predicted_quality=0;
		predicted_quality();
	}
	already_in_update_route_task=0;
    }

    

    /* send a beacon advertising this node's routeInfo */
    // only posted if running and radioOn
    task void sendBeaconTask() {
	uint8_t jj;
        error_t eval;
    	int etxsend, CNT;
	routing_table_entry* entry;
        if (sending) {
            return;
        }

	rate_RUI++; //Added for lifetime calc
	event_RUI++; //Added for lifetime calc
	rate_RUI_5_min++;


	//beaconMsg->Life = ((int)(Life/(double)1));
	beaconMsg->Life = Life;		//printf("Life=%d\n",Life);
	beaconMsg->badnode = badnode;
        beaconMsg->P = P;


	CNT=0;
	beaconMsg->node1_0 = 99;	beaconMsg->node1_1 = 100*99;		beaconMsg->node1_2 = 100*99;		beaconMsg->node1_3 = 99;
	beaconMsg->node2_0 = 99;	beaconMsg->node2_1 = 100*99;		beaconMsg->node2_2 = 100*99;		beaconMsg->node2_3 = 99;
	beaconMsg->node3_0 = 99;	beaconMsg->node3_1 = 100*99;		beaconMsg->node3_2 = 100*99;		beaconMsg->node3_3 = 99;

	for (jj = count_routing_table; jj < routingTableActive; jj++) {
		entry = &routingTable[jj]; 
		
		if(entry == NULL || entry->info.currentpossend == 0)	{ /*printf("\rTest a = %f	b=%f\n",entry->info.a, entry->info.b);	exit(0);*/ }
		
		if(entry->info.currentpossend == 99)	etxsend = 99;
		else etxsend = (int)(10/(double)(entry->info.currentpossend));	


		if(CNT == 0) { beaconMsg->node1_0 = entry->neighbor;	beaconMsg->node1_1 = ((int)(100*entry->info.a)/*100*99*/);	
			beaconMsg->node1_2 = ((int)(100*entry->info.b));
			beaconMsg->node1_3 = etxsend;	CNT++;	count_routing_table++; 
		}
		else if(CNT == 1) { beaconMsg->node2_0 = entry->neighbor;	beaconMsg->node2_1 = ((int)(100*entry->info.a));	
			beaconMsg->node2_2 = ((int)(100*entry->info.b));
			beaconMsg->node2_3 = etxsend;	CNT++;	count_routing_table++; 
		}
		else if(CNT == 2) { beaconMsg->node3_0 = entry->neighbor;	beaconMsg->node3_1 = ((int)(100*entry->info.a));	
			beaconMsg->node3_2 = ((int)(100*entry->info.b));
			beaconMsg->node3_3 = etxsend;	CNT++;	count_routing_table++; 
		}
		beaconMsg->transpower = power;	
	
	if (count_routing_table == routingTableActive)	count_routing_table = 0;	
	}


        beaconMsg->options = 0;

        /* Congestion notification: am I congested? */
        if (call CtpCongestion.isCongested()) {
            beaconMsg->options |= CTP_OPT_ECN;
        }

        beaconMsg->parent = routeInfo.parent;
        if (state_is_root) {
            beaconMsg->etx = routeInfo.etx;
	    beaconMsg->Qpatharray = 0;
        }
        else if (routeInfo.parent == INVALID_ADDR) {
            beaconMsg->etx = routeInfo.etx;
            beaconMsg->options |= CTP_OPT_PULL;
        } else {
	    beaconMsg->etx = routeInfo.etx + evaluateEtx(call LinkEstimator.getLinkQuality(routeInfo.parent), routeInfo.parent );
	    beaconMsg->Qpatharray = Q_pathmetric;
        }
	beaconMsg->rt_p = 1;

        dbg("TreeRouting", "%s parent: %d etx: %d\n",
                  __FUNCTION__,
                  beaconMsg->parent, 
                  beaconMsg->etx);
        call CollectionDebug.logEventRoute(NET_C_TREE_SENT_BEACON, beaconMsg->parent, 0, beaconMsg->etx);

	call CC2420Packet.setPower(&beaconMsgBuffer, MAX_POWER_VAL);//Added 
        eval = call BeaconSend.send(AM_BROADCAST_ADDR, 
                                    &beaconMsgBuffer, 
                                    sizeof(ctp_routing_header_t));
        if (eval == SUCCESS) {
            sending = TRUE;
        } else if (eval == EOFF) {
            radioOn = FALSE;
            dbg("TreeRoutingCtl","%s running: %d radioOn: %d\n", __FUNCTION__, running, radioOn);
        }
    }

    event void BeaconSend.sendDone(message_t* msg, error_t error) {
        if ((msg != &beaconMsgBuffer) || !sending) {
            //something smells bad around here
            return;
        }
        sending = FALSE;
    }

    event void RouteTimer.fired() {

      if (radioOn && running) {
         if(already_in_update_route_task==0)	post updateRouteTask();
      }
    }
      
    event void BeaconTimer.fired() {
      if (radioOn && running) {
        if (!tHasPassed) {
          post updateRouteTask(); //always send the most up to date info
          post sendBeaconTask();
          dbg("RoutingTimer", "Beacon timer fired at %s\n", sim_time_string());
          remainingInterval();
        }
        else {
          decayInterval();
        }
      }
    }


    ctp_routing_header_t* getHeader(message_t* ONE m) {
      return (ctp_routing_header_t*)call BeaconSend.getPayload(m, call BeaconSend.maxPayloadLength());
    }
    

    uint16_t command_CtpInfo_getReducePower(void) {
      	int i, jj, power_reduce;
	routing_table_entry* entry;	
	power_reduce=0;

	for (jj = 0; jj < routingTableActive; jj++) {
		entry = &routingTable[jj]; 
		if(entry->info.reduce_power==1){
			power_reduce=1;
			break;
		}
	}	
	return power_reduce;
    }
    
    /* Handle the receiving of beacon messages from the neighbors. We update the
     * table, but wait for the next route update to choose a new parent */
    event message_t* BeaconReceive.receive(message_t* msg, void* payload, uint8_t len) {
        am_addr_t from;
        ctp_routing_header_t* rcvBeacon;
        bool congested;
	int jj, etx_neighbor_metric, CA[32], CNT=0;
	routing_table_entry* entry;


	CA[MIN_POWER_VAL+9*step_pc]=9;	CA[MIN_POWER_VAL+8*step_pc]=8;	CA[MIN_POWER_VAL+7*step_pc]=7;	CA[MIN_POWER_VAL+6*step_pc]=6;	CA[MIN_POWER_VAL+5*step_pc]=5;
	CA[MIN_POWER_VAL+4*step_pc]=4;	CA[MIN_POWER_VAL+3*step_pc]=3;	CA[MIN_POWER_VAL+2*step_pc]=2;	CA[MIN_POWER_VAL+1*step_pc]=1;	CA[MIN_POWER_VAL]=0;

	rate_RUI_RECV++; //Added for lifetime calc
	event_RUI_RECV++; //Added for lifetime calc

	rate_RUI_RECV_5_min++;

        // Received a beacon, but it's not from us.
        if (len != sizeof(ctp_routing_header_t)) {
          dbg("LITest", "%s, received beacon of size %hhu, expected %i\n",
                     __FUNCTION__, 
                     len,
                     (int)sizeof(ctp_routing_header_t));
              
          return msg;
        }
        
        //need to get the am_addr_t of the source
        from = call AMPacket.source(msg);
	rcvBeacon = (ctp_routing_header_t*)payload;

	if(from==156)	which_one=156;
	if(from==1)	which_one=1;


	congested = call CtpRoutingPacket.getOption(msg, CTP_OPT_ECN);

        dbg("TreeRouting","%s from: %d  [ parent: %d etx: %d]\n",
            __FUNCTION__, from, 
            rcvBeacon->parent, rcvBeacon->etx);

        //update neighbor table
        if (rcvBeacon->parent != INVALID_ADDR) {

            /* If this node is a root, request a forced insert in the link
             * estimator table and pin the node. */
            if (rcvBeacon->etx == 0 || rcvBeacon->badnode==1) {
                dbg("TreeRouting","from a root, inserting if not in table\n");
                call LinkEstimator.insertNeighbor(from);
                call LinkEstimator.pinNeighbor(from);
            }
            //TODO: also, if better than my current parent's path etx, insert

            routingTableUpdateEntry(from, rcvBeacon->parent, rcvBeacon->etx, rcvBeacon->badnode);
            call CtpInfo.setNeighborCongested(from, congested);
        }

	for (jj = 0; jj < routingTableActive; jj++) {
		entry = &routingTable[jj]; 

		if(entry->neighbor == from && rcvBeacon->node1_0 == TOS_NODE_ID){
			entry->info.a_send = rcvBeacon->node1_1/(double)100;	entry->info.b_send = rcvBeacon->node1_2/(double)100; 	
			entry->info.route_quality_send = rcvBeacon->node1_3;	
			CNT++;
		}

		else if(entry->neighbor == from && rcvBeacon->node2_0 == TOS_NODE_ID){
			entry->info.a_send = rcvBeacon->node2_1/(double)100;	entry->info.b_send = rcvBeacon->node2_2/(double)100; 	
			entry->info.route_quality_send = rcvBeacon->node2_3;
			CNT++;	
		}

		else if(entry->neighbor == from && rcvBeacon->node3_0 == TOS_NODE_ID){
			entry->info.a_send = rcvBeacon->node3_1/(double)100;	entry->info.b_send = rcvBeacon->node3_2/(double)100; 	
			entry->info.route_quality_send = rcvBeacon->node3_3;	
			CNT++;	
		}

		if(entry->neighbor == from){		
			if(rcvBeacon->badnode == 1) {
				entry->info.reduce_power=1;	red_pow = 1;	
			}
			else if(entry->info.reduce_power == 1){
				entry->info.reduce_power=0;
				red_pow=command_CtpInfo_getReducePower();
			}
			else entry->info.reduce_power=0;

			entry->info.Life_record = rcvBeacon->Life;	
			entry->info.Q_patharray = rcvBeacon->Qpatharray;

			entry->info.P_record=rcvBeacon->P;	
		}

	}
        

        

        if (call CtpRoutingPacket.getOption(msg, CTP_OPT_PULL)) {
              resetInterval();
        }
        return msg;
    }


    /* Signals that a neighbor is no longer reachable. need special care if
     * that neighbor is our parent */
    event void LinkEstimator.evicted(am_addr_t neighbor) {
        routingTableEvict(neighbor);
        dbg("TreeRouting","%s\n",__FUNCTION__);
        if (routeInfo.parent == neighbor) {
            routeInfoInit(&routeInfo);
            justEvicted = TRUE;
            if(already_in_update_route_task==0)	post updateRouteTask();
        }
    }

    /* Interface UnicastNameFreeRouting */
    /* Simple implementation: return the current routeInfo */
    command am_addr_t Routing.nextHop() {
        return routeInfo.parent;    
    }
    command bool Routing.hasRoute() {
        return (routeInfo.parent != INVALID_ADDR);
    }
   
    /* CtpInfo interface */
    command error_t CtpInfo.getParent(am_addr_t* parent) {
        if (parent == NULL) 
            return FAIL;
        if (routeInfo.parent == INVALID_ADDR)    
            return FAIL;
        *parent = routeInfo.parent;
        return SUCCESS;
    }

    command error_t CtpInfo.getParent_random(uint16_t* parent) {
        
        *parent = max_P;
        return SUCCESS;
    } 

    command error_t CtpInfo.getEtx(uint16_t* etx) {
        if (etx == NULL) 
            return FAIL;
        if (routeInfo.parent == INVALID_ADDR)    
            return FAIL;
	if (state_is_root == 1) {
	  *etx = 0;
	} else {
	  *etx = routeInfo.etx + call LinkEstimator.getLinkQuality(routeInfo.parent);
	}
        return SUCCESS;
    }

   command error_t CtpInfo.store_current_forward_etx(uint16_t* etx) {
        
	*etx = store_current_forward_etx;
        return SUCCESS;
    }

    command error_t CtpInfo.linketx_for_check(uint16_t* etx) {
        
	*etx = call LinkEstimator.getLinkQuality(routeInfo.parent);
        return SUCCESS;
    }

    command void CtpInfo.getPower(uint16_t* power_level){
	*power_level=power;
    }

    command error_t CtpInfo.get_set_Beacontrs(uint16_t* val) {
        *val = rate_RUI_5_min;		rate_RUI_5_min=0;
	return SUCCESS;
    }

    command error_t CtpInfo.get_set_Beaconrcv(uint16_t* val) {
        *val = rate_RUI_RECV_5_min;		rate_RUI_RECV_5_min=0;
	return SUCCESS;
    }

    command error_t CtpInfo.get_set_Datafwd(uint16_t* val) {
        *val = rate_FWD_5_min;		rate_FWD_5_min=0;
	return SUCCESS;
    }
    
    command error_t CtpInfo.get_set_Datarcv(uint16_t* val) {
        *val = rate_RCV_5_min;		rate_RCV_5_min=0;
	return SUCCESS;
    }

    command error_t CtpInfo.get_set_Dataov(uint16_t* val) {
        *val = rate_OVERHEAR_5_min;		rate_OVERHEAR_5_min=0;
	return SUCCESS;
    }

    command error_t CtpInfo.get_Remain_Life(uint16_t* val) {

	*val = which_one;
	which_one=0;	
	return SUCCESS;
    }

    command error_t CtpInfo.get_Percent_cap(uint16_t* val) {
        *val = percent_cap;		
	return SUCCESS;
    }

    command void CtpInfo.recomputeRoutes() {
      if(already_in_update_route_task==0)	post updateRouteTask();
    }

    command void CtpInfo.triggerRouteUpdate() {
      resetInterval();
     }

    command void CtpInfo.triggerImmediateRouteUpdate() {
      resetInterval();
    }

    command void CtpInfo.setNeighborCongested(am_addr_t n, bool congested) {
        uint8_t idx;    
        if (ECNOff)
            return;
        idx = routingTableFind(n);
        if (idx < routingTableActive) {
            routingTable[idx].info.congested = congested;
        }
        if (routeInfo.congested && !congested){ 
            if(already_in_update_route_task==0)		post updateRouteTask();
	}
        else if (routeInfo.parent == n && congested){
            if(already_in_update_route_task==0)		post updateRouteTask();
	}
    }

    command bool CtpInfo.isNeighborCongested(am_addr_t n) {
        uint8_t idx;    

        if (ECNOff) 
            return FALSE;

        idx = routingTableFind(n);
        if (idx < routingTableActive) {
            return routingTable[idx].info.congested;
        }
        return FALSE;
    }
    
    /* RootControl interface */
    /** sets the current node as a root, if not already a root */
    /*  returns FAIL if it's not possible for some reason      */
    command error_t RootControl.setRoot() {
        bool route_found = FALSE;
        route_found = (routeInfo.parent == INVALID_ADDR);
	state_is_root = 1;
	routeInfo.parent = my_ll_addr; //myself
	routeInfo.etx = 0;

        if (route_found) 
            signal Routing.routeFound();
        dbg("TreeRouting","%s I'm a root now!\n",__FUNCTION__);
        call CollectionDebug.logEventRoute(NET_C_TREE_NEW_PARENT, routeInfo.parent, 0, routeInfo.etx);
        return SUCCESS;
    }

    command error_t RootControl.unsetRoot() {
      state_is_root = 0;
      routeInfoInit(&routeInfo);

      dbg("TreeRouting","%s I'm not a root now!\n",__FUNCTION__);
      if(already_in_update_route_task==0)	post updateRouteTask();
      return SUCCESS;
    }

    command bool RootControl.isRoot() {
        return state_is_root;
    }

    default event void Routing.noRoute() {
    }
    
    default event void Routing.routeFound() {
    }


  /* The link will be recommended for insertion if it is better* than some
   * link in the routing table that is not our parent.
   * We are comparing the path quality up to the node, and ignoring the link
   * quality from us to the node. This is because of a couple of things:
   *   1. we expect this call only for links with white bit set
   *   2. we are being optimistic to the nodes in the table, by ignoring the
   *      1-hop quality to them (which means we are assuming it's 1 as well)
   *      This actually sets the bar a little higher for replacement
   *   3. this is faster
   */
    event bool CompareBit.shouldInsert(message_t *msg, void* payload, uint8_t len) {
        
        bool found = FALSE;
        uint16_t pathEtx;
        uint16_t neighEtx;
        int i;
        routing_table_entry* entry;
        ctp_routing_header_t* rcvBeacon;

        if ((call AMPacket.type(msg) != AM_CTP_ROUTING) ||
            (len != sizeof(ctp_routing_header_t))) 
            return FALSE;

        /* 1.determine this packet's path quality */
        rcvBeacon = (ctp_routing_header_t*)payload;

        if (rcvBeacon->parent == INVALID_ADDR)
            return FALSE;
        /* the node is a root, recommend insertion! */
        if (rcvBeacon->etx == 0) {
            return TRUE;
        }
    
        pathEtx = rcvBeacon->etx; // + linkEtx;

        /* 2. see if we find some neighbor that is worse */
        for (i = 0; i < routingTableActive && !found; i++) {
            entry = &routingTable[i];
            //ignore parent, since we can't replace it
            if (entry->neighbor == routeInfo.parent)
                continue;
            neighEtx = entry->info.etx;
            found |= (pathEtx < neighEtx); 
        }
        return found;
    }


    /************************************************************/
    /* Routing Table Functions                                  */

    /* The routing table keeps info about neighbor's route_info,
     * and is used when choosing a parent.
     * The table is simple: 
     *   - not fragmented (all entries in 0..routingTableActive)
     *   - not ordered
     *   - no replacement: eviction follows the LinkEstimator table
     */

    void routingTableInit() {
        routingTableActive = 0;
    }

    /* Returns the index of parent in the table or
     * routingTableActive if not found */
    uint8_t routingTableFind(am_addr_t neighbor) {
        uint8_t i;
        if (neighbor == INVALID_ADDR)
            return routingTableActive;
        for (i = 0; i < routingTableActive; i++) {
            if (routingTable[i].neighbor == neighbor)
                break;
        }
        return i;
    }


    error_t routingTableUpdateEntry(am_addr_t from, am_addr_t parent, uint16_t etx, uint8_t rcv_beacon_badnode)    {
        uint8_t idx;
        uint16_t  linkEtx;
	int iii;        
	routing_table_entry* entry;	
	//linkEtx = call LinkEstimator.getLinkQuality(from);
	linkEtx = evaluateEtx(call LinkEstimator.getLinkQuality(from), from);
	

        idx = routingTableFind(from);
        if (idx >= routingTableSize-2 && rcv_beacon_badnode != 1) {
            //not found and table is full
            //if (passLinkEtxThreshold(linkEtx))
                //TODO: add replacement here, replace the worst
            //}
            dbg("TreeRouting", "%s FAIL, table full\n", __FUNCTION__);
            return FAIL;
        }
        else if (idx == routingTableActive) {
            //not found and there is space
            if (passLinkEtxThreshold(linkEtx) || rcv_beacon_badnode == 1) {
	      routingTable[idx].neighbor = from;
	      routingTable[idx].info.parent = parent;
	      routingTable[idx].info.etx = etx;
	      routingTable[idx].info.haveHeard = 1;
	      routingTable[idx].info.congested = FALSE;
	      routingTableActive++;
	      entry = &routingTable[idx];
			for(iii=0;iii<MAX_NUMBER_POWER_LEVEL_ENTRY;iii++){
					entry->info.t_array[iii] = 0;
					entry->info.pos[iii] = 99;
			}
			entry->info.lastsequence = 0;		entry->info.first_time = 1;	
			entry->info.reduce_power = 0;		entry->info.confidence = 0;	entry->info.Life_record = 999;
			entry->info.a = 99;	entry->info.b = 99;	entry->info.a_send = 99;	entry->info.b_send = 99;
			entry->info.route_quality_send = 99;	entry->info.currentpossend = 99;
	      dbg("TreeRouting", "%s OK, new entry\n", __FUNCTION__);
            } else {
                dbg("TreeRouting", "%s Fail, link quality (%hu) below threshold\n", __FUNCTION__, linkEtx);
            }
        } else {
            //found, just update
	  routingTable[idx].neighbor = from;
	  routingTable[idx].info.parent = parent;
	  routingTable[idx].info.etx = etx;
	  routingTable[idx].info.haveHeard = 1;
	  dbg("TreeRouting", "%s OK, updated entry\n", __FUNCTION__);
        }
        return SUCCESS;
    }

    /* if this gets expensive, introduce indirection through an array of pointers */
    error_t routingTableEvict(am_addr_t neighbor) {
        uint8_t idx,i;
        idx = routingTableFind(neighbor);
        if (idx == routingTableActive) 
            return FAIL;
        routingTableActive--;
        for (i = idx; i < routingTableActive; i++) {
            routingTable[i] = routingTable[i+1];    
        } 
        return SUCCESS; 
    }

    
    /*********** end routing table functions ***************/

    /* Default implementations for CollectionDebug calls.
     * These allow CollectionDebug not to be wired to anything if debugging
     * is not desired. */

    default command error_t CollectionDebug.logEvent(uint8_t type) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventSimple(uint8_t type, uint16_t arg) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node) {
        return SUCCESS;
    }
    default command error_t CollectionDebug.logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t etx) {
        return SUCCESS;
    }

    command bool CtpRoutingPacket.getOption(message_t* msg, ctp_options_t opt) {
      return ((getHeader(msg)->options & opt) == opt) ? TRUE : FALSE;
    }

    command void CtpRoutingPacket.setOption(message_t* msg, ctp_options_t opt) {
      getHeader(msg)->options |= opt;
    }

    command void CtpRoutingPacket.clearOption(message_t* msg, ctp_options_t opt) {
      getHeader(msg)->options &= ~opt;
    }

    command void CtpRoutingPacket.clearOptions(message_t* msg) {
      getHeader(msg)->options = 0;
    }

    
    command am_addr_t     CtpRoutingPacket.getParent(message_t* msg) {
      return getHeader(msg)->parent;
    }
    command void          CtpRoutingPacket.setParent(message_t* msg, am_addr_t addr) {
      getHeader(msg)->parent = addr;
    }
    
    command uint16_t      CtpRoutingPacket.getEtx(message_t* msg) {
      return getHeader(msg)->etx;
    }
    command void          CtpRoutingPacket.setEtx(message_t* msg, uint16_t etx) {
      getHeader(msg)->etx = etx;
    }

    command uint8_t CtpInfo.numNeighbors() {
      return routingTableActive;
    }
    command uint16_t CtpInfo.getNeighborLinkQuality(uint8_t n) {
      return (n < routingTableActive)? call LinkEstimator.getLinkQuality(routingTable[n].neighbor):0xffff;
    }
    command uint16_t CtpInfo.getNeighborRouteQuality(uint8_t n) {
      return (n < routingTableActive)? call LinkEstimator.getLinkQuality(routingTable[n].neighbor) + routingTable[n].info.etx:0xfffff;
    }
    command am_addr_t CtpInfo.getNeighborAddr(uint8_t n) {
      return (n < routingTableActive)? routingTable[n].neighbor:AM_BROADCAST_ADDR;
    }


    command void CtpInfo.passfromForwarding(int neighborID, uint8_t currentsequence, int power_in_pkt) {
			double pos;
			int i, failure, CA[32];
			double PL[32];		
			routing_table_entry* entry;
			PL[31] = 0.0001;	PL[30] = -9.14E+02;	PL[29] = -3.01E+03; 	PL[28] = -6.10E+03;	PL[27] = -1.00E+04;	PL[26] = -1.45E+04;
		PL[25] = -1.95E+04;	PL[24] = -2.47E+04;	PL[23] = -3.00E+04;	PL[22] = -3.52E+04;	PL[21] = -4.03E+04;	PL[20] = -4.52E+04;
		PL[19] = -5.00E+04;	PL[18] = -5.47E+04;	PL[17] = -5.94E+04;	PL[16] = -6.44E+04;	PL[15] = -7.00E+04;
		PL[14] = -7.63E+04;	PL[13] = -8.33E+04;	PL[12] = -9.12E+04;	PL[11] = -1.00E+05;	PL[10] = -1.10E+05;
		PL[9] = -1.21E+05;	PL[8] = -1.34E+05;	PL[7] = -1.50E+05;	PL[6] = -1.69E+05;	PL[5] = -1.92E+05;
		PL[4] = -2.18E+05;	PL[3] = -2.50E+05;	PL[2] = -2.87E+05;	PL[1] = -3.30E+05;	PL[0] = -3.79E+05; 

	entry=NULL;

	CA[MIN_POWER_VAL+9*step_pc]=9;	CA[MIN_POWER_VAL+8*step_pc]=8;	CA[MIN_POWER_VAL+7*step_pc]=7;	CA[MIN_POWER_VAL+6*step_pc]=6;	CA[MIN_POWER_VAL+5*step_pc]=5;
	CA[MIN_POWER_VAL+4*step_pc]=4;	CA[MIN_POWER_VAL+3*step_pc]=3;	CA[MIN_POWER_VAL+2*step_pc]=2;	CA[MIN_POWER_VAL+1*step_pc]=1;	CA[MIN_POWER_VAL]=0;

		
			for (i = 0; i < routingTableActive; i++) {
				entry = &routingTable[i];
				if(entry->neighbor == neighborID)
					break;
			}
			
			success++;


			if(entry == NULL || entry->neighbor != neighborID) 	goto CCC;
			else if (currentsequence == entry->info.lastsequence /*|| entry->info.first_time==1*/)	goto BBB;
			else if(currentsequence > entry->info.lastsequence){
				failure = currentsequence - entry->info.lastsequence - 1;
				pos = success/(double)(success + failure);
			}
			else	{	
				failure = currentsequence - entry->info.lastsequence + 255;
				pos = success/(double)(success + failure);
				success=0;	
			}

			if(entry->info.t_array[CA[power_in_pkt]]==0)		
				entry->info.t_array[CA[power_in_pkt]] = PL[power_in_pkt]/10000; // These many power levels I tried

			if(entry->info.pos[CA[power_in_pkt]] == 99) 	{
				pos=1;
				entry->info.pos[CA[power_in_pkt]] = pos;
			}
			else	entry->info.pos[CA[power_in_pkt]] = 0.5*entry->info.pos[CA[power_in_pkt]] + 0.5*pos;



			entry->info.currentpossend = entry->info.pos[CA[power_in_pkt]];	


			BBB:	;
			entry->info.first_time=0;
			entry->info.lastsequence = currentsequence;		
			CCC: ;
		
			
}

  command uint16_t CtpInfo.ov_nxt(uint16_t* m){
	*m=store_rate_OVERHEAR_5_min;
  }

  command uint16_t CtpInfo.forwd_num(uint16_t* m){
	*m = Life;
  }

  /*command error_t CtpInfo.get_Remain_Life(uint16_t* val) {
        *val = Life;		
	return SUCCESS;
    }*/

  command void CtpInfo.power_level(uint16_t* m){
  	*m=power;
  }
    
} 

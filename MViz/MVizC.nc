/*
 * Copyright (c) 2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * MViz demo application using the collection layer. 
 * See README.txt file in this directory and TEP 119: Collection.
 *
 * @author David Gay
 * @author Kyle Jamieson
 * @author Philip Levis
 */

#include "MViz.h"
#include "printf.h"

module MVizC @safe(){

	provides {
	//interface GetNow<uint16_t>;
	interface GetNow<uint16_t> as GetVoltageData;
	}
  uses {
    // Interfaces for initialization:
    interface Boot;
    interface SplitControl as RadioControl;
    interface SplitControl as SerialControl;
    interface StdControl as RoutingControl;
    
    // Interfaces for communication, multihop and serial:
    interface Send;
    interface Receive as Snoop;
    interface Receive;
    interface Receive as Receive2;
    interface AMSend as SerialSend;
    interface CollectionPacket;
    interface RootControl;

    // Miscalleny:
    interface Timer<TMilli>;
    interface Read<uint16_t>;
    interface Leds;
    interface CtpInfo;
    interface CtpPacket;
    interface LinkEstimator;
    interface Random;
    interface CC2420Config;
    //interface Timer<TMilli> as Timer0;
    interface LowPowerListening;  //mod
    interface Read<uint16_t> as VoltageRead;
    interface GetNow<uint16_t> as GetRate;
    interface GetNow<uint16_t> as GetMAX_c;
  }
}

implementation {
  task void uartSendTask();
  static void startTimer();
  static void fatal_problem();
  static void report_problem();
  static void report_sent();
  static void report_received();
  //int tmp;
  //int tx_counter=0;
  //int cur_chan;
  //uint16_t voltage_adc, VOLT_DATA, CHOSEN_CHAN=99;
  //int num_chan=8;
  int chan[8], i, MAX1=0, MAX_c=13, min_bl[8], ch, volt, old_chan, PAR;

  uint8_t uartlen;
  message_t sendbuf;
  message_t uartbuf;
  bool sendbusy=FALSE, uartbusy=FALSE;
  int TRK, SEQ_NO=1;

  /* Current local state - interval, version and accumulated readings */
  mviz_msg_t local;
  uint16_t voltagedata=0;

  uint8_t reading; /* 0 to NREADINGS */
  /*  ctp_data_header_t* getHeader(message_t* m) {
    return (ctp_data_header_t*)call SubPacket.getPayload(m, sizeof(ctp_data_header_t));
  }
  command int       CtpPacket.getchan(message_t* msg) {return getHeader(msg)->chan;}*/

  /* When we head an Oscilloscope message, we check it's sample count. If
     it's ahead of ours, we "jump" forwards (set our count to the received
     count). However, we must then suppress our next count increment. This
     is a very simple form of "time" synchronization (for an abstract
     notion of time). */
  bool suppress_count_change;
  int DEF_CHAN;

  // 
  // On bootup, initialize radio and serial communications, and our
  // own state variables.
  //
  event void Boot.booted() {
    local.interval = DEFAULT_INTERVAL;
    local.origin = TOS_NODE_ID;

    DEF_CHAN=call CC2420Config.getChannel();

    // Beginning our initialization phases:
    if (call RadioControl.start() != SUCCESS){ TRK=1;
      fatal_problem();
    }

    if (call RoutingControl.start() != SUCCESS){ TRK=2;
      fatal_problem();}
  }


    
  
  
  event void RadioControl.startDone(error_t error) {
    if (error != SUCCESS){ TRK=3;
      fatal_problem();}

    if (sizeof(local) > call Send.maxPayloadLength()){ TRK=4;
      fatal_problem();}

    if (call SerialControl.start() != SUCCESS){ TRK=5;
      fatal_problem();}
  }

  event void SerialControl.startDone(error_t error) {
    if (error != SUCCESS){ TRK=6;
      fatal_problem();}

    // This is how to set yourself as a root to the collection layer:
    if (local.origin % 500 == 0){
      call RootControl.setRoot();
	call LowPowerListening.setLocalWakeupInterval(0);
    }
	//printf("\r\nStart %d", TRK);
	//printfflush();

    startTimer();
  }

  static void startTimer() {
    call Timer.startPeriodic(local.interval);
    reading = 0;
  }

  event void RadioControl.stopDone(error_t error) { }
  event void SerialControl.stopDone(error_t error) { }

  //
  // Only the root will receive messages from this interface; its job
  // is to forward them to the serial uart for processing on the pc
  // connected to the sensor network.
  //
  event message_t*
  Receive.receive(message_t* msg, void *payload, uint8_t len) {
    //call Leds.led2On();
    if (uartbusy == FALSE) {
      mviz_msg_t* in = (mviz_msg_t*)payload;
      mviz_msg_t* out = (mviz_msg_t*)call SerialSend.getPayload(&uartbuf, sizeof(mviz_msg_t));
      printf("volt=%d\n",in->forward);	printfflush();	
      if (out == NULL) {
	return msg;
      }
      else {
	memcpy(out, in, sizeof(mviz_msg_t));
      }
      uartbusy = TRUE;
      uartlen = sizeof(mviz_msg_t);
      post uartSendTask();
    }

    return msg;
  }


  event message_t*
  Receive2.receive(message_t* msg, void *payload, uint8_t len) {
    //call Leds.led2On();


    return msg;
  }
  
  
  task void uartSendTask() {
    if (call SerialSend.send(0xffff, &uartbuf, uartlen) != SUCCESS) {
      uartbusy = FALSE;
    }
  }
  //
  // Overhearing other traffic in the network.
  //
  event message_t* 
  Snoop.receive(message_t* msg, void* payload, uint8_t len) {
    mviz_msg_t *omsg = payload;

	//chan[msg->etx]++;
	//ch=call CtpPacket.getchan(msg);
	//ch=0.5*(ch-11);
	//volt=call CtpPacket.getvoltage(msg);
	//call GetVoltageData.getNow();
	//if(call CtpPacket.getOrigin(msg) == 1) call Leds.set(10);
	//call Leds.set(call CtpPacket.getOrigin(msg));
	//call Leds.set(10);
	//if(volt<min_bl[ch])	min_bl[ch]=volt;
	//chan[ch]++;
	//getHeader(msg)->chan;
    report_received();

    // If we receive a newer version, update our interval. 
    if (omsg->version > local.version) {
      local.version = omsg->version;
      local.interval = omsg->interval;
      startTimer();
    }

    // If we hear from a future count, jump ahead but suppress our own
    // change.
    if (omsg->count > local.count) {
      local.count = omsg->count;
      suppress_count_change = TRUE;
    }

    return msg;
  }

  /* At each sample period:
     - if local sample buffer is full, send accumulated samples
     - read next sample
  */
  event void Timer.fired() {
  
      old_chan=call CC2420Config.getChannel(); 
      //if(TOS_NODE_ID==0 && old_chan!=26) call Leds.set(7);
	//printf("\nStart %d", DEFAULT_INTERVAL);
	printf("\n\rStart %d", DEFAULT_INTERVAL);
	printfflush();
	//call Leds.led1Toggle();
	
  
    if (!sendbusy) {
      mviz_msg_t *o = (mviz_msg_t *)call Send.getPayload(&sendbuf, sizeof(mviz_msg_t));
      if (o == NULL) {//TRK=7;
	fatal_problem();
	return;
      }
      memcpy(o, &local, sizeof(local));
      if (call Send.send(&sendbuf, sizeof(local)) == SUCCESS)
	sendbusy = TRUE;
      else
	report_problem();
    }

    
    reading = 0;
    /* Part 2 of cheap "time sync": increment our count if we didn't
       jump ahead. */
    if (!suppress_count_change)
      local.count++;
    suppress_count_change = FALSE;

	SEQ_NO++;

    /*if (TOS_NODE_ID!=0 && old_chan!=0 && call CtpInfo.get_chan_assigned()==1)	{
	 call CC2420Config.setChannel(old_chan);	
		call CC2420Config.sync();
    }*/
    //call Leds.set((int)(0.5*((call CC2420Config.getChannel()-11))+3));
    call Timer.stop();
    call Timer.startPeriodic(local.interval);    
    if (call Read.read() != SUCCESS){ //TRK=7;
      fatal_problem();}
  }

  event void Send.sendDone(message_t* msg, error_t error) {
    if (error == SUCCESS){
      report_sent();
     
      }
    else
      report_problem();

    sendbusy = FALSE;

	    /*if (TOS_NODE_ID!=0 && old_chan!=0 && call CtpInfo.get_chan_assigned()==1)		{ 
			call Leds.led2On();			
			call CC2420Config.setChannel(old_chan);	
			call CC2420Config.sync();
		}
    		call Leds.set((int)(0.5*((call CC2420Config.getChannel()-11))+3));*/
  }

  event void Read.readDone(error_t result, uint16_t data) {
    uint16_t val;
    if (result != SUCCESS) {
      data = 0xffff;
      report_problem();
    }
    //local.reading = data;
 //   VOLT_DATA=data;
    call CtpInfo.getEtx(&val);
    local.etx = val;
    //local.etx = call CtpInfo.getchan();
    //local.etx = call CC2420Config.getChannel();
    //local.etx = call CtpInfo.hop_cnt_nxt();  //commented on 8th Oct
    //local.etx = old_chan;
//local.reading=MAX_c;
    call CtpInfo.getParent(&val);
    //val=0;
    val=call CtpInfo.getparent_random();
    //val=voltagedata;
    //local.parent = val;
	local.parent = SEQ_NO;
 //   PAR = val;
    local.link_route_addr = val;
    //local.reading = 999;
    local.forward = 99;
    call CtpInfo.ov_nxt(&val);		local.forward = val;	local.forward = voltagedata;
    //local.forward = SEQ_NO;
    call CtpInfo.forwd_num(&val);	local.count = val;
    //local.link_route_value = call LinkEstimator.getLinkQuality(local.link_route_addr);

    voltagedata = data;
    if(call VoltageRead.read() != SUCCESS)
    	fatal_problem();
  }
  event void LinkEstimator.evicted(am_addr_t addr){}
  
  event void SerialSend.sendDone(message_t *msg, error_t error) {
    uartbusy = FALSE;
  }

 
  /****** read battery voltage ******************/
  event void VoltageRead.readDone(error_t result,uint16_t data){
  
  	atomic{
  		if(result != SUCCESS){
  	  		voltagedata = 0xffff;
  	  		report_problem();
  		}
  	//local.reading = data;
  	voltagedata = data;
  	}	
  }

 
 
  // Use LEDs to report various status issues.
  static void fatal_problem() { 
    call Leds.led0On(); 
    //call Leds.led1On();
    call Leds.led2On();
//TRK=1;
//call Leds.set(TRK);
    call Timer.stop();
  }
  
  
    int counter=100, t;
    event void CC2420Config.syncDone(error_t error){

 }

  static void report_problem() { /*call Leds.led0Toggle();*/ }
  static void report_sent() { /*call Leds.led1Toggle();*/ }
  static void report_received() { /*call Leds.led2Toggle();*/ }
  
    async command uint16_t GetVoltageData.getNow(){
  	return voltagedata;
  }
}



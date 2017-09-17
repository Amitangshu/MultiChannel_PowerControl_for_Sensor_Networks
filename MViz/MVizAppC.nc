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

#include <MViz.h>
#include "printf.h"

configuration MVizAppC { }
implementation {
  components MainC, MVizC, LedsC, new TimerMilliC(), 
    //new PhotoC() as Sensor, RandomC;
    new VoltageC() as Sensor, RandomC;
    components CC2420ControlC;
    //components new TimerMilliC() as Timer0;
    
  //MainC.SoftwareInit -> Sensor;
  enum{ TREE_ROUTING_TABLE_SIZE = 15};
  
  //MVizC.Timer0 -> Timer0;
  MVizC.Boot -> MainC;
  MVizC.Timer -> TimerMilliC;
  MVizC.Read -> Sensor;
  MVizC.Leds -> LedsC;
  MVizC.Random -> RandomC;
  //
  // Communication components.  These are documented in TEP 113:
  // Serial Communication, and TEP 119: Collection.
  //
  components CollectionC as Collector,  // Collection layer
    ActiveMessageC,                         // AM layer
    new CollectionSenderC(AM_MVIZ_MSG), // Sends multihop RF
    SerialActiveMessageC,                   // Serial messaging
    new SerialAMSenderC(AM_MVIZ_MSG);   // Sends to the serial port

  components new AMReceiverC(AM_MVIZ_MSG);

  components CtpP as Ctp;
  //components CtpForwardingEngineP;
//components CtpRoutingEngineP;
  //components CtpPacket as CtpPack;
  //components new CtpRoutingEngineP(TREE_ROUTING_TABLE_SIZE, 128,15360) as Routern;
  
  MVizC.RadioControl -> ActiveMessageC;
  MVizC.SerialControl -> SerialActiveMessageC;
  MVizC.RoutingControl -> Collector;

  MVizC.Send -> CollectionSenderC;
  MVizC.SerialSend -> SerialAMSenderC.AMSend;
  MVizC.Snoop -> Collector.Snoop[AM_MVIZ_MSG];
  MVizC.Receive -> Collector.Receive[AM_MVIZ_MSG];
  MVizC.Receive2 -> AMReceiverC;
  MVizC.RootControl -> Collector;
  MVizC.CtpInfo -> Ctp;
  MVizC.CtpPacket -> Ctp;
  //MVizC.CtpPacket -> CtpForwardingEngineP;
  MVizC.LinkEstimator -> Ctp;
  MVizC.CC2420Config -> CC2420ControlC;
  MVizC.LowPowerListening->ActiveMessageC;  //mod
  //MVizC.CtpPacket -> CtpPack;
  //MVizC.GetVoltageData -> Router.GetVoltageData;  //mod
  //MVizC.GetMAX_c -> Routern.GetMAX_c;
    //MVizC.GetMAX_c -> Ctp;
//MVizC.GetMAX_c -> CtpRoutingEngineP;
//MVizC.GetMAX_c -> Routern;
  
  components new VoltageC() as Voltage;
  
  MVizC.VoltageRead -> Voltage;

}

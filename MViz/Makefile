COMPONENT=MVizAppC
BOOTLOADER=tosboot      #mod

CFLAGS += -DLOW_POWER_LISTENING
CFLAGS += -DLPL_DEF_LOCAL_WAKEUP=128
CFLAGS += -DLPL_DEF_REMOTE_WAKEUP=128
CFLAGS += -DDELAY_AFTER_RECEIVE=20
CFLAGS += -DCC2420_DEF_RFPOWER=20
#CFLAGS += -I$(TOSDIR)/lib/net/ \
#		 -I$(TOSDIR)/lib/net/ctp \
#		 -I$(TOSDIR)/sensorboards/mts300 \
#		 -I$(TOSDIR)/lib/net/4bitle \
#		 -I/home/amitangshu/Desktop/tinyos-2.1.1_for_me_27thDec/tos/lib/net/ctp \
#		 -I$(TOSDIR)/lib/printf -I.

CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION
		 
CFLAGS += -I$(TOSDIR)/lib/net/ \
		 -I$(TOSDIR)/lib/net/ctp \
		 -I$(TOSDIR)/sensorboards/mts300 \
		 -I$(TOSDIR)/lib/net/4bitle \
		 -I$(TOSDIR)/lib/printf -I.		 
		 
#CFLAGS += -I/home/amitangshu/Desktop/tinyos-2.1.1_for_me_27thDec/tos/lib/net \
#		 -I/home/amitangshu/Desktop/tinyos-2.1.1_for_me_27thDec/tos/lib/net/ctp \
#		 -I/home/amitangshu/Desktop/tinyos-2.1.1_for_me_27thDec/tos/sensorboards/mts300 \
#		 -I/home/amitangshu/Desktop/tinyos-2.1.1_for_me_27thDec/tos/lib/net/4bitle \
#		 -I/home/amitangshu/Desktop/tinyos-2.1.1_for_me_27thDec/tos/lib/printf -I.		 
BUILD_EXTRA_DEPS = MVizMsg.class

MVizMsg.java: MViz.h
	mig -target=null -java-classname=MVizMsg java MViz.h mviz_msg -o $@

MVizMsg.class: MVizMsg.java
	javac MVizMsg.java

#BUILD_EXTRA_DEPS = MVizMsg.o

MVizMsg.c: 
	mig -o MVizMsg.h c MViz.h mviz_msg_t -c-prefix=MVIZ_H -target=micaz $@

mig c -c-prefix=MVIZ_H -target=micaz MViz.h mviz_msg -o VizMsg.h  //remeber this command

include $(MAKERULES)

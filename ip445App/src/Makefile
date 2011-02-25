TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_vxWorks += ip445
LIBRARY_IOC_RTEMS   += ip445

# If your Linux system supports drvIpac and devLib then uncomment this line
LIBRARY_IOC_Linux += ip445

ip445_SRCS += drvIP445.cpp

#INC += drvXXX.h
DBD += ip445.dbd

ip445_LIBS += $(EPICS_BASE_IOC_LIBS)
#=============================


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

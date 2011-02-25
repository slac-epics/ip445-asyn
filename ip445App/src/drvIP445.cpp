/* drvIP445.cpp

    Author: Mark Rivers

    This is the driver for the Acromag IP445 digital output IP module
*/

/* EPICS includes */
#include <drvIpac.h>
#include <errlog.h>
#include <epicsTypes.h>
#include <epicsExport.h>
#include <iocsh.h>

#include <asynPortDriver.h>

#define ACROMAG_ID     0xA3
#define IP445_ID       0x09

typedef struct {
    volatile epicsUInt8 *controlRegister;
    volatile epicsUInt8 *outputPort0;
    volatile epicsUInt8 *outputPort1;
    volatile epicsUInt8 *outputPort2;
    volatile epicsUInt8 *outputPort3;
} IP445Registers;


static const char *driverName = "IP445";

/** This is the class definition for the IP445 class*/
class IP445 : public asynPortDriver
{
public:
    IP445(const char *portName, int carrier, int slot);
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
    virtual void report(FILE *fp, int details);

private:
    asynStatus readBack(asynUser *pasynUser);
    epicsUInt8 *baseAddress;
    IP445Registers regs;
    epicsUInt32 prevValue;
    int initialized;
    int dataParam;
};

IP445::IP445(const char *portName, int carrier, int slot)
    :asynPortDriver(portName,1,1,             // portName, maxAddr, paramTableSize
                    asynUInt32DigitalMask,    // interfaceMask
                    asynUInt32DigitalMask,    // interruptMask
                    0,1,0,0)                  // asynFlags, autoConnect, priority, stackSize
{
    //static const char *functionName = "IP445";
    ipac_idProm_t *id;
    epicsUInt8 *base;
    int manufacturer, model;
    
    this->initialized = 0;
   
    if (ipmCheck(carrier, slot)) {
       errlogPrintf("%s: bad carrier or slot\n", driverName);
       return;
    }
    id = (ipac_idProm_t *) ipmBaseAddr(carrier, slot, ipac_addrID);
    base = (epicsUInt8 *) ipmBaseAddr(carrier, slot, ipac_addrIO);
    this->baseAddress = base;
    manufacturer = id->manufacturerId & 0xff;
    model = id->modelId & 0xff;

    if ((manufacturer != ACROMAG_ID) || ( model != IP445_ID)) {
        errlogPrintf("%s: manufacturer and/or model incorrect = %x/%x, should be %x/%x\n",
            driverName, manufacturer, model, ACROMAG_ID, IP445_ID);
        return;
    }

    /* Set up the register pointers.*/
    /* Define registers in units of 8-bit bytes */
    this->regs.controlRegister          = base + 0x1;
    this->regs.outputPort0              = base + 0x3;
    this->regs.outputPort1              = base + 0x5;
    this->regs.outputPort2              = base + 0x7;
    this->regs.outputPort3              = base + 0x9;
    
    /* Create the asynPortDriver parameter for the data */
    createParam("DIGITAL_DATA", asynParamUInt32Digital, &this->dataParam); 

    this->initialized = 1;

    /* Write 1 to the control register to reset all the outputs to 0 */
    *this->regs.controlRegister = 1;
        
    /* Read back values */
    this->readBack(this->pasynUserSelf);
}

asynStatus IP445::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    static const char *functionName = "writeUInt32Digital";
    IP445Registers r = this->regs;

    if (!this->initialized) return(asynError);
    /* Set any bits that are set in the value and the mask */
    *r.outputPort0  |= (epicsUInt8) (value & mask);
    *r.outputPort1  |= (epicsUInt8) ((value & mask) >> 8);
    *r.outputPort2  |= (epicsUInt8) ((value & mask) >> 16);
    *r.outputPort3  |= (epicsUInt8) ((value & mask) >> 24);
    /* Clear bits that are clear in the value and set in the mask */
    *r.outputPort0  &= (epicsUInt8) (value | ~mask);
    *r.outputPort1  &= (epicsUInt8) ((value | ~mask) >> 8);
    *r.outputPort2  &= (epicsUInt8) ((value | ~mask) >> 16);
    *r.outputPort3  &= (epicsUInt8) ((value | ~mask) >> 24);
    readBack(pasynUser);
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s, value=%x, mask=%x\n", 
              driverName, functionName, value, mask);
    return(asynSuccess);
}

asynStatus IP445::readBack(asynUser *pasynUser)
{
    epicsUInt32 value;
    IP445Registers r = this->regs;
    static const char *functionName = "readBack";

    if (!this->initialized) return(asynError);
    value = (*r.outputPort0);
    value |= ((*r.outputPort1) << 8);
    value |= ((*r.outputPort2) << 16);
    value |= ((*r.outputPort3) << 24);
    /* Set the mask of bits that have changed for interrupt callbacks */
    setInterruptUInt32Digital(this->pasynUserSelf, (value^prevValue), interruptOnBoth);
    prevValue = value;
    this->setUIntDigitalParam(dataParam, value, 0xFFFFFFFF);
    callParamCallbacks();
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s, readBack=%x\n", 
              driverName, functionName, value);
    return(asynSuccess);
}

void IP445::report(FILE *fp, int details)
{
    IP445Registers r = this->regs;

    if (!this->initialized) {
        fprintf(fp, "%s %s: not initialized!\n", driverName, this->portName);
        return;
    }
    fprintf(fp, "%s %s: connected at base address %p\n",
            driverName, this->portName, this->baseAddress);
    if (details >= 1) {
        fprintf(fp, "  control register=%x\n", *r.controlRegister);
        fprintf(fp, "  port 0=%x\n", *r.outputPort0);
        fprintf(fp, "  port 1=%x\n", *r.outputPort1);
        fprintf(fp, "  port 2=%x\n", *r.outputPort2);
        fprintf(fp, "  port 3=%x\n", *r.outputPort3);
    }
    asynPortDriver::report(fp, details);
}

extern "C" int initIP445(const char *portName, int carrier, int slot)
{
    IP445 *pIP445=new IP445(portName,carrier,slot);
    pIP445=NULL;
    return(asynSuccess);
}

/* iocsh functions */
static const iocshArg initArg0 = { "Port name",iocshArgString};
static const iocshArg initArg1 = { "Carrier",iocshArgInt};
static const iocshArg initArg2 = { "Slot",iocshArgInt};
static const iocshArg * const initArgs[3] = {&initArg0,
                                             &initArg1,
                                             &initArg2};
static const iocshFuncDef initFuncDef = {"initIP445",3,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    initIP445(args[0].sval, args[1].ival, args[2].ival);
}
void IP445Register(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(IP445Register);








#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "ti/comm_modules/i2c/controller/i2c_comm_controller.h"
#include "Si5391-RevA-5391AEVB-Registers.h"


volatile I2C_CommandInfo gCommand;

I2C_ResponseInfo gResponse;

volatile bool gSendCommand = true;

#define SI5391_I2C_ADDR 0x74

I2C_Instance gI2C;



void Si5391_writeRegister(uint8_t address, uint8_t data)
{

    I2C_CommandInfo command;
    command.targetAddr = SI5391_I2C_ADDR;
    command.commandType = WRITE_COMMAND;
    command.addr = address;              // Register Address
    command.dataArray = &data;              // Data
    command.dataSize = 1;       // Datasize
    command.crcEnable = false;               // Not used

    // Send commands by i2c
    I2C_sendCommand(&gI2C, &command);
}
uint8_t Si5391_readRegister(uint8_t address)
{
    uint8_t tx_data;
    I2C_CommandInfo command;
    command.targetAddr = SI5391_I2C_ADDR;
    command.commandType = READ_COMMAND;
    command.addr = address;              // Register Address
    command.dataArray = &tx_data;              // Data
    command.dataSize = 1;       // Datasize
    command.crcEnable = false;               // Not used

    // Send commands by i2c
    I2C_sendCommand(&gI2C, &command);

    tx_data = I2C_getResponse(&gI2C,command.targetAddr);
    return tx_data;
}

void Si5391_write_16bit_register(si5391_reva_register_t reg)
{
    static uint8_t page_reg_last;
    uint8_t page_reg_current=(reg.address >> 8) & 0xFF;

            if(page_reg_last != page_reg_current){
                Si5391_writeRegister(0x01 , page_reg_current);
                Si5391_writeRegister(reg.address , reg.value);
                page_reg_last=page_reg_current;

            }else{
                Si5391_writeRegister(reg.address , reg.value);
    }
}

void si5391_load_configuration() {

    uint16_t num_regs=SI5391_REVA_REG_CONFIG_NUM_REGS;

    for (uint16_t i = 0; i < 2; i++) {
        Si5391_write_16bit_register(si5391_reva_registers[i]);

    }

    delay_cycles(0.3*32000000UL);

    for (uint16_t i = 2; i < num_regs; i++) {
        Si5391_write_16bit_register(si5391_reva_registers[i]);

    }
}

uint8_t si5391_ready(){
    if(Si5391_readRegister(0xFE)==0x0F){
        return 1;
    }else{
        return 0;
    }
}

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(I2C_INST_INT_IRQN);

    I2C_init(&gI2C);

    while(!si5391_ready()){}
    si5391_load_configuration();

}


int rest_val = 0;

void I2C_INST_IRQHandler(void)
{
    rest_val = DL_I2C_getPendingInterrupt(I2C_INST);
    switch (rest_val)
    {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            gI2C.rxMsg.len = gI2C.rxMsg.ptr;
            I2C_decodeResponse(&gI2C,&gResponse);
            gI2C.status = I2C_STATUS_RX_COMPLETE;
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(
                I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2C.status = I2C_STATUS_TX_COMPLETE;
            break;

        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2C.status = I2C_STATUS_RX_INPROGRESS;
            /* Store bytes received from target in Rx Msg Buffer */
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST) != true) {
                if (gI2C.rxMsg.ptr < MAX_BUFFER_SIZE) {
                    gI2C.rxMsg.buffer[gI2C.rxMsg.ptr++] =
                        DL_I2C_receiveControllerData(I2C_INST);
                } else {
                    /* Ignore and remove from FIFO if the buffer is full */
                    DL_I2C_receiveControllerData(I2C_INST);
                }
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            gI2C.status = I2C_STATUS_TX_INPROGRESS;
            /* Fill TX FIFO with bytes to send */
            if (gI2C.txMsg.ptr < gI2C.txMsg.len) {
                gI2C.txMsg.ptr += DL_I2C_fillControllerTXFIFO(
                    I2C_INST, &gI2C.txMsg.buffer[gI2C.txMsg.ptr], gI2C.txMsg.len - gI2C.txMsg.ptr);
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            gI2C.status = I2C_STATUS_ERROR;
            break;
        default:
            //gI2C.status = I2C_STATUS_ERROR;
            break;
    }
}

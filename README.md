## Summary

This code writes the registers from a register header file to the SI5391 device using the MSP communication module via the I2C protocol. 
This header file can be generated using the ClockBuilder application published by Texas Instruments.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| I2C0 | PA0 | I2C Serial Data line (SDA) |
| I2C0 | PA1 | I2C Serial Clock line (SCL) |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

there are examples within embassy for this MCU - we're clearly building something just slightly wrong.

X> Get executor working.
X> Get UART working.

-> Note that HAL currently has no mechanism for CAN or more complex clock configuration. Will need to do some edits for sure.
-> Porting for CAN driver as well - definitely seems to be bosch m_can. 

Required:

-> Support for PLL (present in PAC, missing in HAL)
-> CAN peripheral (missing in PAC and HAL)
-> Better HardFault and panic handlers -> dump to UART?
-> actually write code :)

Optional:

-> Support for flash controller (embedded_storage types)

-> How are time drivers initialized?
    -> HAL init function is doing it.
there are examples within embassy for this MCU - we're clearly building something just slightly wrong.

X> Get executor working.
X> Get UART working.

-> Note that HAL currently has no mechanism for CAN or more complex clock configuration. Will need to do some edits for sure.

-> How are time drivers initialized? I don't see an init function for it, yet timers are working.
-> 

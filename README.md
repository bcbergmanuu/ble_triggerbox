Pin 0 enables sending the trigger. Each combination should always include pin0 (first bit / 1).

![image](https://github.com/user-attachments/assets/276592ce-6d58-4154-bebd-10d3c30bddda)

Tested: 

minimum pulsewith = 3ms, pulse period = 6ms for 40 burst mode trigger interrupts. 
minimum pulsewith = 100uS, pulse period 200uS for 1 trigger interrupt.

And array with aquired data will be send, depending on speed between 1 and 50 bytes.
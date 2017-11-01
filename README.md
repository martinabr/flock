# Flock

This repository contains the source code of Flock for the use with Glossy.
Flock is an approach to compensate clock frequency deviations on-the-fly across synchronously transmitting nodes.

To learn more about Flock, check out our IEEE LCN'17 paper.

In case of questions, please contact me at *martina[dot]brachmann[at]tu-dresden[dot]de*.

## Code Layout

*Disclaimer: Although we tested the code extensively, Flock is a research prototype that likely contains bugs. We take no responsibility for and give no warranties in respect of using the code.*

Flock can be enabled/disabled with  `-DGLOSSY_CLOCK_DRIFT_COMP=<0 or 1>`

The source code of Flock extends the Glossy source code in

    /core/dev/glossy.c

Changing DCO frequencies on different nodes can be done by uncommenting `msp430_cpu_set(mcu_frequency_table[node_id]);` in `/platform/sky/contiki-sky-main.c`.

## Notes

The evaluation of this work was done with `msp430-gcc (GCC) 4.6.3 20120301 (mspgcc LTS 20120406 unpatched)`. Other compilers or versions of the msp430-gcc may lead to different evaluation results.



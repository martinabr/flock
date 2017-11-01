/*
 * Copyright (c) 2011, ETH Zurich.
 * Copyright (c) 2017, TU Dresden
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *         Martina Brachmann <martina.brachmann@tu-dresden.de>
 *
 */

/**
 *
 * \mainpage
 *           These files document the source code of Glossy, a flooding architecture for wireless sensor networks,
 *           implemented in <a href="http://www.sics.se/contiki/">Contiki</a> based on Tmote Sky sensor nodes.
 *
 *           Glossy was published at ACM/IEEE IPSN '11 in the paper titled
 *           <a href="ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZTS2011.pdf">
 *           Efficient network flooding and time synchronization with Glossy</a>,
 *           which also received the best paper award.
 *
 *           This documentation is divided into three main parts:
 *           \li \ref glossy-test "Simple application for testing Glossy":
 *           Example of a simple application that periodically floods a packet and prints related statistics.
 *           \li \ref glossy_interface "Glossy API":
 *           API provided by Glossy for an application that wants to use it.
 *           \li \ref glossy_internal "Glossy internal functions":
 *           Functions used internally by Glossy during a flood.
 *
 *           A complete overview of the documentation structure is available <a href="modules.html">here</a>.
 *
 * \author
 *           <a href="http://www.tik.ee.ethz.ch/~ferrarif">Federico Ferrari</a> <ferrari@tik.ee.ethz.ch>
 *
 */

/**
 * \defgroup glossy-test Simple application for testing Glossy
 *
 *           This application runs Glossy periodically to flood a packet from one node (initiator)
 *           to the other nodes (receivers) and prints flooding-related statistics.
 *
 *           The application schedules Glossy periodically with a fixed period \link GLOSSY_PERIOD \endlink.
 *
 *           The duration of each Glossy phase is given by \link GLOSSY_DURATION \endlink.
 *
 *           During each Glossy phase, the maximum number of transmissions in Glossy (N)
 *           is set to \link N_TX \endlink.
 *
 *           The initiator of the floods is the node having nodeId \link INITIATOR_NODE_ID \endlink.
 *
 *           The packet to be flooded has the format specified by data structure \link glossy_data_struct \endlink.
 *
 *           Receivers synchronize by computing the reference time during each Glossy phase.
 *
 *           To synchronize fast, at startup receivers run Glossy with a significantly shorter period
 *           (\link GLOSSY_INIT_PERIOD \endlink) and longer duration (\link GLOSSY_INIT_DURATION \endlink).
 *
 *           Receivers exit the bootstrapping phase when they have computed the reference time for
 *           \link GLOSSY_BOOTSTRAP_PERIODS \endlink consecutive Glossy phases.
 *
 * @{
 */

/**
 * \file
 *         A simple example of an application that uses Glossy, source file.
 *
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#include "glossy-test.h"

/**
 * \defgroup glossy-test-variables Application variables
 * @{
 */

/**
 * \defgroup glossy-test-variables-sched-sync Scheduling and synchronization variables
 * @{
 */

static glossy_data_struct glossy_data;     /**< \brief Flooding data. */
static struct rtimer rt;                   /**< \brief Rtimer used to schedule Glossy. */
static struct pt pt;                       /**< \brief Protothread used to schedule Glossy. */
static rtimer_clock_t t_ref_l_old = 0;     /**< \brief Reference time computed from the Glossy
                                                phase before the last one. \sa get_t_ref_l */
static uint8_t skew_estimated = 0;         /**< \brief Not zero if the clock skew over a period of length
                                                \link GLOSSY_PERIOD \endlink has already been estimated. */
static uint8_t sync_missed = 0;            /**< \brief Current number of consecutive phases without
                                                synchronization (reference time not computed). */
static rtimer_clock_t t_start = 0;         /**< \brief Starting time (low-frequency clock)
                                                of the last Glossy phase. */
static int period_skew = 0;                /**< \brief Current estimation of clock skew over a period
                                                of length \link GLOSSY_PERIOD \endlink. */

/** @} */

/**
 * \defgroup glossy-test-variables-stats Statistics variables
 * @{
 */

static unsigned long packets_received = 0; /**< \brief Current number of received packets. */
static unsigned long packets_missed = 0;   /**< \brief Current number of missed packets. */
static unsigned long latency = 0;          /**< \brief Latency of last Glossy phase, in us. */
static unsigned long sum_latency = 0;      /**< \brief Current sum of latencies, in ticks of low-frequency
                                                clock (used to compute average). */

/** @} */
/** @} */

/**
 * \defgroup glossy-test-processes Application processes and functions
 * @{
 */

/**
 * \defgroup glossy-test-print-stats Print statistics information
 * @{
 */

PROCESS(glossy_print_stats_process, "Glossy print stats");
PROCESS_THREAD(glossy_print_stats_process, ev, data)
{
	PROCESS_BEGIN();

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		// Print statistics only if Glossy is not still bootstrapping.
		if (!GLOSSY_IS_BOOTSTRAPPING()) {
			if (get_rx_cnt()) {	// Packet received at least once.
				// Increment number of successfully received packets.
				packets_received++;
				// Compute latency during last Glossy phase.
				rtimer_clock_t lat = get_t_first_rx_l() - get_t_ref_l();
				// Add last latency to sum of latencies.
				sum_latency += lat;
				// Convert latency to microseconds.
				latency = (unsigned long)(lat) * 1e6 / RTIMER_SECOND;
				// Print information about last packet and related latency.
				printf("Glossy received %u time%s: seq_no %lu, latency %lu.%03lu ms\n",
						get_rx_cnt(), (get_rx_cnt() > 1) ? "s" : "", glossy_data.seq_no,
								latency / 1000, latency % 1000);
			} else {	// Packet not received.
				// Increment number of missed packets.
				packets_missed++;
				// Print failed reception.
				printf("Glossy NOT received\n");
			}
#if GLOSSY_DEBUG
                        printf("Set DCO to %ld, node_id %u \n", msp430_dco_freq_get(), node_id);
			uint32_t freq = (get_Crx_e() - 2) * 1000000 /288.619;
                        printf("skew %ld ppm, T_irq %u, freq %lu\n", 
                                (long)(period_skew * 1e6) / GLOSSY_PERIOD, get_T_irq(), freq);
			printf("C_rx_e %u, I %u, nops %u\n", get_Crx_e(), get_I() - (uint16_t)((352 * freq) /1000000), get_nops_to_add());
                        printf("high_T_irq %u, high_T_rx %u, rx_timeout %u, bad_length %u, bad_header %u, bad_crc %u, cov %u\n",
                                high_T_irq, high_T_rx, rx_timeout, bad_length, bad_header, bad_crc, cov);
#endif /* GLOSSY_DEBUG */
			// Compute current average reliability.
			unsigned long avg_rel = packets_received * 1e5 / (packets_received + packets_missed);
			// Print information about average reliability.
			printf("average reliability %3lu.%03lu %% ",
					avg_rel / 1000, avg_rel % 1000);
			printf("(missed %lu out of %lu packets)\n",
					packets_missed, packets_received + packets_missed);
#if ENERGEST_CONF_ON
			// Compute average radio-on time, in microseconds.
			unsigned long avg_radio_on = (unsigned long)GLOSSY_PERIOD * 1e6 / RTIMER_SECOND *
					(energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT)) /
					(energest_type_time(ENERGEST_TYPE_CPU) + energest_type_time(ENERGEST_TYPE_LPM));
			// Print information about average radio-on time.
			printf("average radio-on time %lu.%03lu ms\n",
					avg_radio_on / 1000, avg_radio_on % 1000);
#endif /* ENERGEST_CONF_ON */
			// Compute average latency, in microseconds.
			unsigned long avg_latency = sum_latency * 1e6 / (RTIMER_SECOND * packets_received);
			// Print information about average latency.
			printf("average latency %lu.%03lu ms\n",
					avg_latency / 1000, avg_latency % 1000);
                        
#if GLOSSY_DEBUG
                        printf("rcv:\t");
                        uint8_t i;
                        for(i = 0; i < (sizeof(rcv)/sizeof(uint8_t)); i++) {
                            printf("%d ", rcv[i]);
                        }
                        printf("\n");
                        
                        printf("rssi:\t");
                        for(i = 0; i < (sizeof(rssi)/sizeof(int8_t)); i++) {
                            printf("%d ", rssi[i]+(-45));
                        }
                        printf("\n");
                        
                        printf("lqi:\t");
                        for(i = 0; i < (sizeof(lqi)/sizeof(uint8_t)); i++) {
                            printf("%d ", lqi[i]);
                        }
                        printf("\n");
#endif /* GLOSSY_DEBUG */
		}
	}

	PROCESS_END();
}

/** @} */

/**
 * \defgroup glossy-test-skew Clock skew estimation
 * @{
 */

static inline void estimate_period_skew(void) {
	// Estimate clock skew over a period only if the reference time has been updated.
	if (GLOSSY_IS_SYNCED()) {
		// Estimate clock skew based on previous reference time and the Glossy period.
		period_skew = get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t)GLOSSY_PERIOD);
		// Update old reference time with the newer one.
		t_ref_l_old = get_t_ref_l();
		// If Glossy is still bootstrapping, count the number of consecutive updates of the reference time.
		if (GLOSSY_IS_BOOTSTRAPPING()) {
			// Increment number of consecutive updates of the reference time.
			skew_estimated++;
			// Check if Glossy has exited from bootstrapping.
			if (!GLOSSY_IS_BOOTSTRAPPING()) {
				// Glossy has exited from bootstrapping.
				leds_off(LEDS_RED);
				// Initialize Energest values.
				energest_init();
#if GLOSSY_DEBUG
				high_T_irq = 0;
				bad_crc = 0;
				bad_length = 0;
				bad_header = 0;
#endif /* GLOSSY_DEBUG */

			}
		}
	}
}

/** @} */

/**
 * \defgroup glossy-test-scheduler Periodic scheduling
 * @{
 */

char glossy_scheduler(struct rtimer *t, void *ptr) {
	PT_BEGIN(&pt);

	if (IS_INITIATOR()) {	// Glossy initiator.
		while (1) {
			// Increment sequence number.
			glossy_data.seq_no++;
			// Glossy phase.
			leds_on(LEDS_GREEN);
			rtimer_clock_t t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
			// Start Glossy.
			glossy_start((uint8_t *)&glossy_data, DATA_LEN, GLOSSY_INITIATOR, GLOSSY_SYNC, N_TX,
					APPLICATION_HEADER, t_stop, (rtimer_callback_t)glossy_scheduler, t, ptr);
			// Store time at which Glossy has started.
			t_start = RTIMER_TIME(t);
			// Yield the protothread. It will be resumed when Glossy terminates.
			PT_YIELD(&pt);

			// Off phase.
			leds_off(LEDS_GREEN);
			// Stop Glossy.
			glossy_stop();
			if (!GLOSSY_IS_BOOTSTRAPPING()) {
				// Glossy has already successfully bootstrapped.
				if (!GLOSSY_IS_SYNCED()) {
					// The reference time was not updated: increment reference time by GLOSSY_PERIOD.
					set_t_ref_l(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD);
					set_t_ref_l_updated(1);
				}
			}
			// Schedule begin of next Glossy phase based on GLOSSY_PERIOD.
			rtimer_set(t, t_start + GLOSSY_PERIOD, 1, (rtimer_callback_t)glossy_scheduler, ptr);
			// Estimate the clock skew over the last period.
			estimate_period_skew();
			// Poll the process that prints statistics (will be activated later by Contiki).
			process_poll(&glossy_print_stats_process);
			// Yield the protothread.
			PT_YIELD(&pt);
		}
	} else {	// Glossy receiver.
		while (1) {
			// Glossy phase.
			leds_on(LEDS_GREEN);
			rtimer_clock_t t_stop;
			if (GLOSSY_IS_BOOTSTRAPPING()) {
				// Glossy is still bootstrapping:
				// Schedule end of Glossy phase based on GLOSSY_INIT_DURATION.
				t_stop = RTIMER_TIME(t) + GLOSSY_INIT_DURATION;
			} else {
				// Glossy has already successfully bootstrapped:
				// Schedule end of Glossy phase based on GLOSSY_DURATION.
				t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
			}
			// Start Glossy.
			glossy_start((uint8_t *)&glossy_data, DATA_LEN, GLOSSY_RECEIVER, GLOSSY_SYNC, N_TX,
					APPLICATION_HEADER, t_stop, (rtimer_callback_t)glossy_scheduler, t, ptr);
			// Yield the protothread. It will be resumed when Glossy terminates.
			PT_YIELD(&pt);

			// Off phase.
			leds_off(LEDS_GREEN);
			// Stop Glossy.
			glossy_stop();
			if (GLOSSY_IS_BOOTSTRAPPING()) {
				// Glossy is still bootstrapping.
				if (!GLOSSY_IS_SYNCED()) {
					// The reference time was not updated: reset skew_estimated to zero.
					skew_estimated = 0;
				}
			} else {
				// Glossy has already successfully bootstrapped.
				if (!GLOSSY_IS_SYNCED()) {
					// The reference time was not updated:
					// increment reference time by GLOSSY_PERIOD + period_skew.
					set_t_ref_l(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD + period_skew);
					set_t_ref_l_updated(1);
					// Increment sync_missed.
					sync_missed++;
				} else {
					// The reference time was not updated: reset sync_missed to zero.
                    sync_missed = 0;
                }
            }
            // Estimate the clock skew over the last period.
            estimate_period_skew();
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy is still bootstrapping.
                if (skew_estimated == 0) {
                    // The reference time was not updated:
                    // Schedule begin of next Glossy phase based on last begin and GLOSSY_INIT_PERIOD.
                    rtimer_set(t, RTIMER_TIME(t) + GLOSSY_INIT_PERIOD, 1,
                            (rtimer_callback_t) glossy_scheduler, ptr);
                } else {
                    // The reference time was updated:
                    // Schedule begin of next Glossy phase based on reference time and GLOSSY_INIT_PERIOD.

                    rtimer_clock_t time = GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD - GLOSSY_INIT_GUARD_TIME;
                    if RTIMER_CLOCK_LT(RTIMER_NOW(), time) {
                        //if (get_rx_cnt() && RTIMER_CLOCK_LT(time, (RTIMER_TIME(t) + GLOSSY_DURATION + GLOSSY_PERIOD))) {
                            rtimer_set(t, time,
                                    1, (rtimer_callback_t) glossy_scheduler, ptr);
                    } else {
                        // We missed the current Glossy phase. 
                        // We schedule the timer for the next period.
                        rtimer_set(t, time + GLOSSY_PERIOD,
                                1, (rtimer_callback_t) glossy_scheduler, ptr);
                    }
                }
            } else {
                // Glossy has already successfully bootstrapped:
                // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
                rtimer_clock_t time = GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD + period_skew - GLOSSY_GUARD_TIME * (1 + sync_missed);

                if RTIMER_CLOCK_LT(RTIMER_NOW(), time) {
                        rtimer_set(t, time,
                                1, (rtimer_callback_t) glossy_scheduler, ptr);
                } else {
                    // We missed the current Glossy phase. 
                    // We schedule the timer for the next period.
                    rtimer_set(t, time + GLOSSY_PERIOD,
                            1, (rtimer_callback_t) glossy_scheduler, ptr);
                }
            }
            // Poll the process that prints statistics (will be activated later by Contiki).
            process_poll(&glossy_print_stats_process);
            // Yield the protothread.
            PT_YIELD(&pt);
        }
    }

    PT_END(&pt);
}

/** @} */

/**
 * \defgroup glossy-test-init Initialization
 * @{
 */

PROCESS(glossy_test, "Glossy test");
AUTOSTART_PROCESSES(&glossy_test);

PROCESS_THREAD(glossy_test, ev, data) {
    PROCESS_BEGIN();

    leds_on(LEDS_RED);
    // Initialize Glossy data.
    glossy_data.seq_no = 0;
    // Start print stats processes.
    process_start(&glossy_print_stats_process, NULL);
    // Start Glossy busy-waiting process.
    process_start(&glossy_process, NULL);
    // Start Glossy experiment in one second.
    rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND, 1, (rtimer_callback_t) glossy_scheduler, NULL);

    PROCESS_END();
}

/** @} */
/** @} */
/** @} */
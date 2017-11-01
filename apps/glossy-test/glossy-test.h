/*
 * Copyright (c) 2011, ETH Zurich.
 * Copyright (c) 2017, TU Dresden.
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
 * \defgroup glossy-test Simple application for testing Glossy
 * @{
 */

/**
 * \file
 *         A simple example of an application that uses Glossy, header file.
 *
 *         The application schedules Glossy periodically.
 *         The period is determined by \link GLOSSY_PERIOD \endlink.
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef GLOSSY_TEST_H_
#define GLOSSY_TEST_H_

#include "glossy.h"
#include "node-id.h"
#include "msp430_freq.h"

/**
 * \defgroup glossy-test-settings Application settings
 * @{
 */

/**
 * \brief NodeId of the initiator.
 *        Default value: 1
 */
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID       1
#endif

/**
 * \brief Application-specific header.
 *        Default value: 0x0
 */
#define APPLICATION_HEADER      0

/**
 * \brief Maximum number of transmissions N.
 *        Default value: 5.
 */
#define N_TX                    3

/**
 * \brief Period with which a Glossy phase is scheduled.
 *        Default value: 250 ms.
 */
#define GLOSSY_PERIOD           (RTIMER_SECOND / 4)      // 250 ms

/**
 * \brief Duration of each Glossy phase.
 *        Default value: 20 ms.
 */
#define GLOSSY_DURATION         (RTIMER_SECOND / 50)     //  20 ms

/**
 * \brief Guard-time at receivers.
 *        Default value: 526 us.
 */
#if COOJA
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 500)
#else
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1900)   // 526 us
#endif /* COOJA */

/**
 * \brief Number of consecutive Glossy phases with successful computation of reference time required to exit from bootstrapping.
 *        Default value: 3.
 */
#define GLOSSY_BOOTSTRAP_PERIODS 3

/**
 * \brief Period during bootstrapping at receivers.
 *        It should not be an exact fraction of \link GLOSSY_PERIOD \endlink.
 *        Default value: 69.474 ms.
 */
#define GLOSSY_INIT_PERIOD      (GLOSSY_INIT_DURATION + RTIMER_SECOND / 100)                   //  69.474 ms

/**
 * \brief Duration during bootstrapping at receivers.
 *        Default value: 59.474 ms.
 */
#define GLOSSY_INIT_DURATION    (GLOSSY_DURATION - GLOSSY_GUARD_TIME + GLOSSY_INIT_GUARD_TIME) //  59.474 ms

/**
 * \brief Guard-time during bootstrapping at receivers.
 *        Default value: 50 ms.
 */
#define GLOSSY_INIT_GUARD_TIME  (RTIMER_SECOND / 20)                                           //  50 ms

/**
 * \brief Data structure used to represent flooding data.
 */
typedef struct {
	unsigned long seq_no; /**< Sequence number, incremented by the initiator at each Glossy phase. */
} glossy_data_struct;

/** @} */

/**
 * \defgroup glossy-test-defines Application internal defines
 * @{
 */

/**
 * \brief Length of data structure.
 */
#define DATA_LEN                    sizeof(glossy_data_struct)

/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)

/**
 * \brief Check if Glossy is still bootstrapping.
 * \sa \link GLOSSY_BOOTSTRAP_PERIODS \endlink.
 */
#define GLOSSY_IS_BOOTSTRAPPING()   (skew_estimated < GLOSSY_BOOTSTRAP_PERIODS)

/**
 * \brief Check if Glossy is synchronized.
 *
 * The application assumes that a node is synchronized if it updated the reference time
 * during the last Glossy phase.
 * \sa \link is_t_ref_l_updated \endlink
 */
#define GLOSSY_IS_SYNCED()          (is_t_ref_l_updated())

/**
 * \brief Get Glossy reference time.
 * \sa \link get_t_ref_l \endlink
 */
#define GLOSSY_REFERENCE_TIME       (get_t_ref_l())

/** @} */

/** @} */

#endif /* GLOSSY_TEST_H_ */

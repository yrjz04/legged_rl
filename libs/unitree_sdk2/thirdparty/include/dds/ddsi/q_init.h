/*
 * Copyright(c) 2006 to 2020 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
 */
#ifndef Q_INIT_H
#define Q_INIT_H

#if defined (__cplusplus)
extern "C" {
#endif

int create_multicast_sockets (struct ddsi_domaingv *gv);
int joinleave_spdp_defmcip (struct ddsi_domaingv *gv, int dojoin);

#if defined (__cplusplus)
}
#endif

#endif

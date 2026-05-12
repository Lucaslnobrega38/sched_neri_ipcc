// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel support for scheduler IPC classes
 *
 * Copyright (c) 2023, Intel Corporation.
 *
 * Author: Ricardo Neri <ricardo.neri-calderon@linux.intel.com>
 *
 * On hybrid processors, the architecture differences between types of CPUs
 * lead to different number of retired instructions per cycle (IPC). IPCs may
 * differ further by classes of instructions.
 *
 * The scheduler assigns an IPC class to every task with arch_update_ipcc()
 * from data that hardware provides. Implement this interface for x86.
 *
 * Classification uses an exponential-decay accumulator that tracks weighted
 * P-core and E-core ticks. A task is classified as P-type (ipcc=2) when its
 * P-core tick fraction exceeds 40%; otherwise E-type (ipcc=1). The decay
 * prevents stale history from dominating after a workload phase change.
 *
 * See kernel/sched/sched.h for details.
 */

#include <linux/sched.h>
#include <linux/sched/topology.h>

#include <asm/topology.h>
#include <linux/smp.h>

/*
 * ITMT priority threshold separating E-cores from P-cores.
 *
 * Measured priorities on supported SKUs:
 *   i5-1334U  : E-cores = 34,  P-cores = 59
 *   i9-13900K : E-cores = 46,  P-cores = 64
 *
 * Threshold 50 sits safely between the highest known E-core value (46)
 * and the lowest known P-core value (59), with margin on both sides.
 */
#define IPCC_ECORE_PRIO_THRESHOLD	50

/*
 * Right-shift all accumulators when ipcc_total_ticks reaches this threshold.
 * Keeps counters bounded and provides exponential decay of old history.
 */
#define IPCC_DECAY_THRESHOLD	256

/*
 * Minimum total ticks before emitting a classification. Avoids premature
 * decisions on newly-spawned tasks with few samples.
 */
#define IPCC_MIN_TOTAL		8

void intel_update_ipcc(struct task_struct *curr)
{
	u8 hfi_class;

	if (intel_hfi_read_classid(&hfi_class))
		return;

	/*
	 * E-cores do not produce reliable ITD classifications. Identify them
	 * via ITMT priority instead of a CPU model allowlist — this stays
	 * correct across new SKUs without requiring code changes.
	 */
	if (arch_asym_cpu_priority(smp_processor_id()) <= IPCC_ECORE_PRIO_THRESHOLD)
		return;

	/*
	 * When an SMT sibling is busy the hardware blends both threads into
	 * one ITD reading, making scalar (0) and vector (1) classes unreliable.
	 * VNNI (2) and spin (3) are extreme enough to survive the noise, so
	 * only those are accepted while a sibling is active.
	 */
	if (!sched_smt_siblings_idle(task_cpu(curr)) &&
	    hfi_class != 2 && hfi_class != 3)
		return;

	/*
	 * Accumulate weighted ticks per workload type.
	 *
	 * Hardware classes map to core affinities as follows:
	 *   0 (scalar)  → E-type, weight 1
	 *   1 (vector)  → P-type, weight 1
	 *   2 (VNNI)    → P-type, weight 2  (benefits most from P-core IPC)
	 *   3 (spin)    → E-type, weight 2  (low IPC, wastes P-core resources)
	 */
	switch (hfi_class) {
	case 0:
		curr->ipcc_ticks_E++;
		curr->ipcc_total_ticks++;
		break;
	case 1:
		curr->ipcc_ticks_P++;
		curr->ipcc_total_ticks++;
		break;
	case 2:
		curr->ipcc_ticks_P += 2;
		curr->ipcc_total_ticks += 2;
		break;
	case 3:
		curr->ipcc_ticks_E += 2;
		curr->ipcc_total_ticks += 2;
		break;
	}

	/* Exponential decay: halve all counters when total hits the threshold. */
	if (curr->ipcc_total_ticks >= IPCC_DECAY_THRESHOLD) {
		curr->ipcc_ticks_E >>= 1;
		curr->ipcc_ticks_P >>= 1;
		curr->ipcc_total_ticks >>= 1;
	}

	/* Wait for enough samples before committing to a class. */
	if (curr->ipcc_total_ticks < IPCC_MIN_TOTAL)
		return;

	/* P-type if P ticks exceed 40% of total; E-type otherwise. */
	if (curr->ipcc_ticks_P * 5 > curr->ipcc_total_ticks * 2)
		curr->ipcc = 2;
	else
		curr->ipcc = 1;
}

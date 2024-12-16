/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_MACH_TOPOLOGY_H
#define _ASM_MACH_TOPOLOGY_H

#ifdef CONFIG_NUMA

extern cpumask_t cpus_on_node[];
#define cpumask_of_node(node)  (&cpus_on_node[node])

void numa_set_distance(int from, int to, int distance);
struct pci_bus;
extern int pcibus_to_node(struct pci_bus *);

#define cpumask_of_pcibus(bus)	(cpu_online_mask)

extern unsigned char __node_distances[MAX_NUMNODES][MAX_NUMNODES];

#define node_distance(from, to)	(__node_distances[(from)][(to)])

#endif /* CONFIG_NUMA */
#endif /* _ASM_MACH_TOPOLOGY_H */

#ifndef _IRQ_LS2K_H
#define _IRQ_LS2K_H

struct iointc {
	struct irq_domain	*domain;
	int cascade;
	void *regbase;
	void *isrreg;
	raw_spinlock_t lock;
};

#endif /* _IRQ_LS2K_H */

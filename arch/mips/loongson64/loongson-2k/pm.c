#include <linux/suspend.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <asm/reboot.h>
#include <loongson-2k.h>
#include <loongson.h>
#include <linux/kexec.h>
#include <asm/bootinfo.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/libfdt.h>
#include <asm/tlbflush.h>
#include <linux/syscore_ops.h>
#include <linux/pm_wakeirq.h>
#include <linux/input.h>
#include <asm/irq.h>


extern u64 suspend_addr;
extern void loongson_suspend_lowlevel(void);
extern void ls2k_suspend_irq(void);
extern void ls2k_resume_irq(void);

/* defined in kernel/power/suspend.c */
extern const char *pm_states[PM_SUSPEND_MAX];

struct loongson2k_registers {
	u32 config5;
	u32 hwrena;
	u32 wired;
	u64 userlocal;
	u64 pagemask;
	u64 gmac;
	u64 uart;
	u64 gpu;
	u64 apbdma;
	u64 usb_phy01;
	u64 usb_phy23;
	u64 sata;
	u64 dma0;
	u64 dma1;
	u64 dma2;
	u64 dma3;
	u64 dma4;
	u64 ebase;
};

static struct loongson2k_registers loongson2k_regs;
static struct input_dev *pwrbtn;

typedef enum {
	ACPI_INT_EN = 1 << 0,
	ACPI_PWRBT_STATUS = 1 << 8,
	ACPI_RTC_WAKEUP_STATUS = 1 << 10,
	ACPI_PCIE_WAKEUP_STATUS = 1 << 14,
	ACPI_WAKE_STATUS = 1 << 15,
} AcpiEventStatusBits;

u32 loongson2k_nr_nodes;
u64 loongson2k_suspend_addr;
u32 loongson2k_pcache_ways;
u32 loongson2k_scache_ways;
u32 loongson2k_pcache_sets;
u32 loongson2k_scache_sets;
u32 loongson2k_pcache_linesz;
u32 loongson2k_scache_linesz;

void ls2k_pm(enum ACPI_Sx sx)
{
	unsigned long base;
	unsigned int acpi_ctrl;
	base = CKSEG1ADDR(APB_BASE) + ACPI_OFF;

	acpi_ctrl = readl((void *)(base + PM1_STS));
	acpi_ctrl &= 0xffffffff;
	writel(acpi_ctrl, (void *)(base + PM1_STS));

	acpi_ctrl = readl((void *)(base + GPE0_STS));
	acpi_ctrl &= 0xffffffff;
	writel(acpi_ctrl, (void *)(base + GPE0_STS));

	/*GMAC0_EN and GMAC1_EN*/
	acpi_ctrl = readl((void *)(base + GPE0_SR));
	acpi_ctrl |= 0x00000060;
	writel(acpi_ctrl, (void *)(base + GPE0_SR));

	/*WOL_BAT_EN*/
	acpi_ctrl = readl((void *)(base + RTC_GPMCR));
	acpi_ctrl |= 0x00000080;
	writel(acpi_ctrl, (void *)(base + RTC_GPMCR));

	/*USB_GMAC_OK set 1*/
	acpi_ctrl = readl((void *)(base + R_GPMCR));
	acpi_ctrl |= 0x00000080;
	writel(acpi_ctrl, (void *)(base + R_GPMCR));

	acpi_ctrl = ((sx << 10) | (1 << 13));
	writel(acpi_ctrl, (void *)(base + PM1_CTR));
}
EXPORT_SYMBOL_GPL(ls2k_pm);

irqreturn_t ls2k_acpi_handler(int irq, void *dev_id)
{

	unsigned int val;
	unsigned long base = CKSEG1ADDR(APB_BASE) + ACPI_OFF;

	val = readl((void *)(base + PM1_STS));

	if (val & ACPI_WAKE_STATUS) {
		if (val & ACPI_PCIE_WAKEUP_STATUS)
			pr_info("PCIE wake\n");
		if (val & ACPI_RTC_WAKEUP_STATUS)
			pr_info("RTC wake\n");
	}

	if (val & ACPI_PWRBT_STATUS) {
		pr_info("power button pressed\n");
		input_report_key(pwrbtn, KEY_POWER, 1);
		input_sync(pwrbtn);
		input_report_key(pwrbtn, KEY_POWER, 0);
		input_sync(pwrbtn);
	}
	writel(val, (void *)(base + PM1_STS));

	return IRQ_HANDLED;
}

int __init loongson_acpi_init(void)
{
	u16 value;
	void *acpi_status_reg;
	void *gpe0_status_reg;
	void *acpi_enable_reg;
	unsigned long base = CKSEG1ADDR(APB_BASE) + ACPI_OFF;
	int ret;

	acpi_status_reg = (void *)(base + PM1_STS);
	gpe0_status_reg = (void *)(base + GPE0_STS);
	acpi_enable_reg = (void *)(base + PM1_SR);

	/* PMEnable: Enable PwrBtn */
	value = readw(acpi_enable_reg);
	value |= ACPI_PWRBT_STATUS;
	value &= ~ACPI_PCIE_WAKEUP_STATUS;
	writew(value, acpi_enable_reg);

	/* PMStatus: Clear WakeStatus/PwrBtnStatus */
	value = readw(acpi_status_reg);
	value |= (ACPI_PWRBT_STATUS |
			ACPI_PCIE_WAKEUP_STATUS |
			ACPI_WAKE_STATUS |
			ACPI_RTC_WAKEUP_STATUS);
	writew(value, acpi_status_reg);

	/* GPEStatus: Clear all generated events */
	writel(readl(gpe0_status_reg), gpe0_status_reg);

	/* enable ACPI interrupt generate */
	value = readl((void *)(base + PM1_CTR));
	writel(value|ACPI_INT_EN, (void *)(base + PM1_CTR));

	pwrbtn = input_allocate_device();
	if (!pwrbtn) {
		pr_err("%s: input allocate device error\n", __func__);
		return -ENOMEM;
	}
	pwrbtn->name = "ACPI Power Button";
	pwrbtn->phys = "acpi/button/input0";
	pwrbtn->id.bustype = BUS_HOST;
	pwrbtn->dev.parent = NULL;
	input_set_capability(pwrbtn, EV_KEY, KEY_POWER);

	ret = input_register_device(pwrbtn);
	if (ret) {
		input_free_device(pwrbtn);
		return ret;
	}

	ret = request_irq(LS2K_ACPI_IRQ, ls2k_acpi_handler, 0, "acpi",
				&pwrbtn->dev);
	if (ret) {
		pr_err("%s: request irq error\n", __func__);
		input_free_device(pwrbtn);
		return ret;
	}

	device_init_wakeup(&pwrbtn->dev, true);
	dev_pm_set_wake_irq(&pwrbtn->dev, LS2K_ACPI_IRQ);

	return 0;
}
device_initcall(loongson_acpi_init);

static int ls2k_pm_valid_state(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_ON:
		return 1;
	case PM_SUSPEND_MEM:
		return !!suspend_addr;
	default:
		return 0;
	}
}

static int ls2k_pm_begin(suspend_state_t state)
{
	return 0;
}

void ls2k_suspend_reg(void)
{
	unsigned long base;

	base = CKSEG1ADDR(CONF_BASE);

	loongson2k_regs.gmac = ls64_conf_read64((void *)(base + GMAC_OFF));
	loongson2k_regs.uart = ls64_conf_read64((void *)(base + UART_OFF));
	loongson2k_regs.gpu = ls64_conf_read64((void *)(base + GPU_OFF));
	loongson2k_regs.apbdma = ls64_conf_read64((void *)(base + APBDMA_OFF));
	loongson2k_regs.usb_phy01 = ls64_conf_read64((void *)(base + USB_PHY01_OFF));
	loongson2k_regs.usb_phy23 = ls64_conf_read64((void *)(base + USB_PHY23_OFF));
	loongson2k_regs.sata = ls64_conf_read64((void *)(base + SATA_OFF));
	loongson2k_regs.dma0 = ls64_conf_read64((void *)(base + CONF_DMA0_OFF));
	loongson2k_regs.dma1 = ls64_conf_read64((void *)(base + CONF_DMA1_OFF));
	loongson2k_regs.dma2 = ls64_conf_read64((void *)(base + CONF_DMA2_OFF));
	loongson2k_regs.dma3 = ls64_conf_read64((void *)(base + CONF_DMA3_OFF));
	loongson2k_regs.dma4 = ls64_conf_read64((void *)(base + CONF_DMA4_OFF));
	if(cpu_has_divec) {
		loongson2k_regs.ebase = read_c0_ebase_64();
	}
}

void ls2k_resume_reg(void)
{
	unsigned long base;

	base = CKSEG1ADDR(CONF_BASE);
	ls64_conf_write64(loongson2k_regs.gmac, (void *)(base + GMAC_OFF));
	ls64_conf_write64(loongson2k_regs.uart, (void *)(base + UART_OFF));
	ls64_conf_write64(loongson2k_regs.gpu, (void *)(base + GPU_OFF));
	ls64_conf_write64(loongson2k_regs.apbdma, (void *)(base + APBDMA_OFF));
	ls64_conf_write64(loongson2k_regs.usb_phy01, (void *)(base + USB_PHY01_OFF));
	ls64_conf_write64(loongson2k_regs.usb_phy23, (void *)(base + USB_PHY23_OFF));
	ls64_conf_write64(loongson2k_regs.sata, (void *)(base + SATA_OFF));
	ls64_conf_write64(loongson2k_regs.dma0, (void *)(base + CONF_DMA0_OFF));
	ls64_conf_write64(loongson2k_regs.dma1, (void *)(base + CONF_DMA1_OFF));
	ls64_conf_write64(loongson2k_regs.dma2, (void *)(base + CONF_DMA2_OFF));
	ls64_conf_write64(loongson2k_regs.dma3, (void *)(base + CONF_DMA3_OFF));
	ls64_conf_write64(loongson2k_regs.dma4, (void *)(base + CONF_DMA4_OFF));
	if(cpu_has_divec) {
		write_c0_ebase_64(MIPS_EBASE_WG);
		write_c0_ebase_64(loongson2k_regs.ebase | MIPS_EBASE_WG);
	}
}

int mach_suspend(void)
{
	loongson2k_regs.config5 = read_c0_config5();
	loongson2k_regs.hwrena = read_c0_hwrena();
	loongson2k_regs.userlocal = read_c0_userlocal();
	loongson2k_regs.wired = read_c0_wired();
	ls2k_suspend_reg();

	return 0;
}

void mach_resume(void)
{
	write_c0_config5(loongson2k_regs.config5);
	write_c0_hwrena(loongson2k_regs.hwrena);
	write_c0_userlocal(loongson2k_regs.userlocal);
	write_c0_wired(loongson2k_regs.wired);
	local_flush_tlb_all();
	ls2k_resume_reg();
}

static int ls2k_pm_enter(suspend_state_t state)
{
	/* processor specific suspend */
	loongson2k_nr_nodes = MAX_NUMNODES;
	loongson2k_suspend_addr = CKSEG1ADDR(suspend_addr);
	loongson2k_pcache_ways = cpu_data[0].dcache.ways;
	loongson2k_scache_ways = cpu_data[0].scache.ways;
	loongson2k_pcache_sets = cpu_data[0].dcache.sets;
	loongson2k_scache_sets = cpu_data[0].scache.sets;
	loongson2k_pcache_linesz = cpu_data[0].dcache.linesz;
	loongson2k_scache_linesz = cpu_data[0].scache.linesz;
	loongson_suspend_lowlevel();

	return 0;
}

static void ls2k_pm_wake(void)
{

}

static void ls2k_pm_end(void)
{

}

static struct syscore_ops ls2k_pm_syscore_ops = {
	.suspend = mach_suspend,
	.resume = mach_resume,
};

static const struct platform_suspend_ops ls2k_pm_ops = {
	.valid	= ls2k_pm_valid_state,
	.begin	= ls2k_pm_begin,
	.enter	= ls2k_pm_enter,
	.wake	= ls2k_pm_wake,
	.end	= ls2k_pm_end,
};

static int __init ls2k_pm_init(void)
{
	register_syscore_ops(&ls2k_pm_syscore_ops);
	suspend_set_ops(&ls2k_pm_ops);

	return 0;
}
arch_initcall(ls2k_pm_init);

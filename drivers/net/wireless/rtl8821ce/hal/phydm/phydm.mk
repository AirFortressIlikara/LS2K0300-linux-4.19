EXTRA_CFLAGS += -I$(srctree)/$(src)/hal/phydm

_PHYDM_FILES := hal/phydm/phydm_debug.o	\
								hal/phydm/phydm_antdiv.o\
								hal/phydm/phydm_soml.o\
								hal/phydm/phydm_smt_ant.o\
								hal/phydm/phydm_antdect.o\
								hal/phydm/phydm_interface.o\
								hal/phydm/phydm_phystatus.o\
								hal/phydm/phydm_hwconfig.o\
								hal/phydm/phydm.o\
								hal/phydm/phydm_dig.o\
								hal/phydm/phydm_pathdiv.o\
								hal/phydm/phydm_rainfo.o\
								hal/phydm/phydm_dynamictxpower.o\
								hal/phydm/phydm_adaptivity.o\
								hal/phydm/phydm_cfotracking.o\
								hal/phydm/phydm_noisemonitor.o\
								hal/phydm/phydm_beamforming.o\
								hal/phydm/phydm_dfs.o\
								hal/phydm/txbf/halcomtxbf.o\
								hal/phydm/txbf/haltxbfinterface.o\
								hal/phydm/txbf/phydm_hal_txbf_api.o\
								hal/phydm/phydm_adc_sampling.o\
								hal/phydm/phydm_ccx.o\
								hal/phydm/phydm_psd.o\
								hal/phydm/phydm_primary_cca.o\
								hal/phydm/phydm_cck_pd.o\
								hal/phydm/phydm_rssi_monitor.o\
								hal/phydm/phydm_auto_dbg.o\
								hal/phydm/phydm_math_lib.o\
								hal/phydm/phydm_api.o\
								hal/phydm/phydm_pow_train.o\
								hal/phydm/phydm_lna_sat.o\
								hal/phydm/phydm_pmac_tx_setting.o\
								hal/phydm/phydm_mp.o\
								hal/phydm/halrf/halrf.o\
								hal/phydm/halrf/halrf_debug.o\
								hal/phydm/halrf/halphyrf_ce.o\
								hal/phydm/halrf/halrf_powertracking_ce.o\
								hal/phydm/halrf/halrf_powertracking.o\
								hal/phydm/halrf/halrf_kfree.o

ifeq ($(CONFIG_RTL8821C), y)
RTL871X = rtl8821c
_PHYDM_FILES +=	hal/phydm/$(RTL871X)/halhwimg8821c_bb.o \
								hal/phydm/$(RTL871X)/halhwimg8821c_mac.o \
								hal/phydm/$(RTL871X)/halhwimg8821c_rf.o \
								hal/phydm/$(RTL871X)/phydm_hal_api8821c.o \
								hal/phydm/$(RTL871X)/phydm_regconfig8821c.o\
								hal/phydm/halrf/$(RTL871X)/halrf_8821c.o\
								hal/phydm/halrf/$(RTL871X)/halrf_iqk_8821c.o
endif

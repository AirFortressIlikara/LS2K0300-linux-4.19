
_PHYDM_FILES :=\
	phydm/phydm.o \
	phydm/phydm_dig.o\
	phydm/phydm_antdiv.o\
	phydm/phydm_soml.o\
	phydm/phydm_smt_ant.o\
	phydm/phydm_pathdiv.o\
	phydm/phydm_rainfo.o\
	phydm/phydm_dynamictxpower.o\
	phydm/phydm_adaptivity.o\
	phydm/phydm_debug.o\
	phydm/phydm_interface.o\
	phydm/phydm_phystatus.o\
	phydm/phydm_hwconfig.o\
	phydm/phydm_dfs.o\
	phydm/phydm_cfotracking.o\
	phydm/phydm_adc_sampling.o\
	phydm/phydm_ccx.o\
	phydm/phydm_primary_cca.o\
	phydm/phydm_cck_pd.o\
	phydm/phydm_rssi_monitor.o\
	phydm/phydm_auto_dbg.o\
	phydm/phydm_math_lib.o\
	phydm/phydm_noisemonitor.o\
	phydm/phydm_api.o\
	phydm/phydm_pow_train.o\
	phydm/phydm_lna_sat.o\
	phydm/phydm_pmac_tx_setting.o\
	phydm/phydm_mp.o\
	phydm/txbf/phydm_hal_txbf_api.o\
	EdcaTurboCheck.o\
	phydm/halrf/halrf.o\
	phydm/halrf/halrf_debug.o\
	phydm/halrf/halphyrf_ap.o\
	phydm/halrf/halrf_powertracking_ap.o\
	phydm/halrf/halrf_powertracking.o\
	phydm/halrf/halrf_kfree.o

ifeq ($(CONFIG_WLAN_HAL_8821CE),y)
	_PHYDM_FILES += phydm/halrf/rtl8821c/halrf_8821c.o
	_PHYDM_FILES += phydm/halrf/rtl8821c/halrf_iqk_8821c.o
	ifeq ($(CONFIG_RTL_ODM_WLAN_DRIVER),y)
		_PHYDM_FILES += \
		phydm/rtl8821c/halhwimg8821c_bb.o\
		phydm/rtl8821c/halhwimg8821c_mac.o\
		phydm/rtl8821c/halhwimg8821c_rf.o\
		phydm/rtl8821c/phydm_regconfig8821c.o\
		phydm/rtl8821c/phydm_hal_api8821c.o
	endif
endif

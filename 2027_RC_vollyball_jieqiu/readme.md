# E-IDE项目配置

```
2027_RC_vollyball_jieqiu
├─ .clang-format
├─ .eide
│  ├─ eide.yml
│  └─ files.options.yml
├─ .mxproject
├─ .pack
│  └─ Keil
│     └─ STM32H7xx_DFP.4.1.0
│        ├─ CMSIS
│        │  ├─ Debug
│        │  │  ├─ STM32H723_725_730_733_735.dbgconf
│        │  │  ├─ STM32H742_743_753_750.dbgconf
│        │  │  ├─ STM32H743.sdf
│        │  │  ├─ STM32H745_747_755_757.dbgconf
│        │  │  ├─ STM32H747.sdf
│        │  │  └─ STM32H7A3_7B3_7B0.dbgconf
│        │  ├─ Flash
│        │  │  ├─ FlashOS.h
│        │  │  ├─ MT25TL01G_STM32H745I-DISCO.FLM
│        │  │  ├─ MT25TL01G_STM32H747I-DISCO.FLM
│        │  │  ├─ MT25TL01G_STM32H747I-EVAL.FLM
│        │  │  ├─ MT25TL01G_STM32H750B-DISCO.FLM
│        │  │  ├─ MTFC4GACAJCN_STM32H750B-DISCO.FLM
│        │  │  ├─ MX25LM51245G_STM32H735-Disco.FLM
│        │  │  ├─ MX25LM51245G_STM32H7B0-Disco.FLM
│        │  │  ├─ MX25LM51245G_STM32H7B0-EVAL.FLM
│        │  │  ├─ MX25LM51245G_STM32H7B3I-Disco.FLM
│        │  │  ├─ MX25LM51245G_STM32H7B3I-EVAL.FLM
│        │  │  ├─ STM32H723x
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashOS.h
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ STM32H723x.uvoptx
│        │  │  │  ├─ STM32H723x.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H72x-73x_1024.FLM
│        │  │  ├─ STM32H730xx_128.FLM
│        │  │  ├─ STM32H735G_Disco_OSPI
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ OSPI
│        │  │  │  │  ├─ mx25lm51245g_conf.h
│        │  │  │  │  ├─ s70kl1281_conf.h
│        │  │  │  │  ├─ stm32h735g_discovery_conf.h
│        │  │  │  │  ├─ stm32h735g_discovery_ospi.c
│        │  │  │  │  ├─ STM32H7OSPI.c
│        │  │  │  │  ├─ STM32H7OSPI.h
│        │  │  │  │  └─ stm32h7xx_hal_conf.h
│        │  │  │  ├─ STM32H735G_DISCO.uvoptx
│        │  │  │  ├─ STM32H735G_DISCO.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H743I-eval_FMC.FLM
│        │  │  ├─ STM32H747I_Disco_QSPI
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ QSPI
│        │  │  │  │  ├─ mt25tl01g.h
│        │  │  │  │  ├─ mt25tl01g_conf.h
│        │  │  │  │  ├─ stm32h747i_discovery_conf.h
│        │  │  │  │  ├─ stm32h747i_discovery_errno.h
│        │  │  │  │  ├─ stm32h747i_discovery_qspi.h
│        │  │  │  │  ├─ STM32H747I_QSPI.c
│        │  │  │  │  ├─ STM32H747I_QSPI.h
│        │  │  │  │  └─ stm32h7xx_hal_conf.h
│        │  │  │  ├─ STM32H747I_DISCO_QSPI.uvoptx
│        │  │  │  ├─ STM32H747I_DISCO_QSPI.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H747I_eval_QSPI
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ QSPI
│        │  │  │  │  ├─ mt25tl01g.h
│        │  │  │  │  ├─ mt25tl01g_conf.h
│        │  │  │  │  ├─ stm32h747i_eval_conf.h
│        │  │  │  │  ├─ stm32h747i_eval_errno.h
│        │  │  │  │  ├─ stm32h747i_EVAL_qspi.h
│        │  │  │  │  ├─ STM32H747I_QSPI.c
│        │  │  │  │  ├─ STM32H747I_QSPI.h
│        │  │  │  │  └─ stm32h7xx_hal_conf.h
│        │  │  │  ├─ STM32H747I_eval_QSPI.uvoptx
│        │  │  │  ├─ STM32H747I_eval_QSPI.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H750_FMC
│        │  │  │  ├─ CMSIS
│        │  │  │  │  ├─ cmsis_armcc.h
│        │  │  │  │  ├─ cmsis_armclang.h
│        │  │  │  │  ├─ cmsis_compiler.h
│        │  │  │  │  ├─ cmsis_gcc.h
│        │  │  │  │  ├─ core_armv8mbl.h
│        │  │  │  │  ├─ core_armv8mml.h
│        │  │  │  │  ├─ core_cm0.h
│        │  │  │  │  ├─ core_cm0plus.h
│        │  │  │  │  ├─ core_cm23.h
│        │  │  │  │  ├─ core_cm3.h
│        │  │  │  │  ├─ core_cm33.h
│        │  │  │  │  ├─ core_cm4.h
│        │  │  │  │  ├─ core_cm7.h
│        │  │  │  │  ├─ core_sc000.h
│        │  │  │  │  ├─ core_sc300.h
│        │  │  │  │  └─ tz_context.h
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashOS.h
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ FMC
│        │  │  │  │  ├─ Legacy
│        │  │  │  │  │  └─ stm32_hal_legacy.h
│        │  │  │  │  ├─ stm32h743i_eval.h
│        │  │  │  │  ├─ stm32h743i_eval_io.h
│        │  │  │  │  ├─ stm32h743i_eval_nor.h
│        │  │  │  │  ├─ stm32h743i_eval_ts.h
│        │  │  │  │  ├─ stm32h743xx.h
│        │  │  │  │  ├─ stm32h753xx.h
│        │  │  │  │  ├─ STM32H7FMC.c
│        │  │  │  │  ├─ STM32H7FMC.h
│        │  │  │  │  ├─ stm32h7xx.h
│        │  │  │  │  ├─ stm32h7xxfmc.c
│        │  │  │  │  ├─ stm32h7xxfmc.h
│        │  │  │  │  ├─ stm32h7xx_hal.c
│        │  │  │  │  ├─ stm32h7xx_hal.h
│        │  │  │  │  ├─ stm32h7xx_hal_adc.h
│        │  │  │  │  ├─ stm32h7xx_hal_adc_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_cec.h
│        │  │  │  │  ├─ stm32h7xx_hal_comp.h
│        │  │  │  │  ├─ stm32h7xx_hal_conf.h
│        │  │  │  │  ├─ stm32h7xx_hal_conf_template.h
│        │  │  │  │  ├─ stm32h7xx_hal_cortex.c
│        │  │  │  │  ├─ stm32h7xx_hal_cortex.h
│        │  │  │  │  ├─ stm32h7xx_hal_crc.h
│        │  │  │  │  ├─ stm32h7xx_hal_crc_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_cryp.h
│        │  │  │  │  ├─ stm32h7xx_hal_cryp_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_dac.h
│        │  │  │  │  ├─ stm32h7xx_hal_dac_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_dcmi.h
│        │  │  │  │  ├─ stm32h7xx_hal_def.h
│        │  │  │  │  ├─ stm32h7xx_hal_dfsdm.h
│        │  │  │  │  ├─ stm32h7xx_hal_dma.h
│        │  │  │  │  ├─ stm32h7xx_hal_dma2d.h
│        │  │  │  │  ├─ stm32h7xx_hal_dma_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_eth.h
│        │  │  │  │  ├─ stm32h7xx_hal_eth_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_fdcan.h
│        │  │  │  │  ├─ stm32h7xx_hal_flash.h
│        │  │  │  │  ├─ stm32h7xx_hal_flash_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_gpio.c
│        │  │  │  │  ├─ stm32h7xx_hal_gpio.h
│        │  │  │  │  ├─ stm32h7xx_hal_gpio_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_hash.h
│        │  │  │  │  ├─ stm32h7xx_hal_hash_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_hcd.h
│        │  │  │  │  ├─ stm32h7xx_hal_hrtim.h
│        │  │  │  │  ├─ stm32h7xx_hal_hsem.h
│        │  │  │  │  ├─ stm32h7xx_hal_i2c.h
│        │  │  │  │  ├─ stm32h7xx_hal_i2c_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_i2s.h
│        │  │  │  │  ├─ stm32h7xx_hal_i2s_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_irda.h
│        │  │  │  │  ├─ stm32h7xx_hal_irda_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_iwdg.h
│        │  │  │  │  ├─ stm32h7xx_hal_jpeg.h
│        │  │  │  │  ├─ stm32h7xx_hal_lptim.h
│        │  │  │  │  ├─ stm32h7xx_hal_ltdc.h
│        │  │  │  │  ├─ stm32h7xx_hal_mdios.h
│        │  │  │  │  ├─ stm32h7xx_hal_mdma.h
│        │  │  │  │  ├─ stm32h7xx_hal_mmc.h
│        │  │  │  │  ├─ stm32h7xx_hal_mmc_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_msp.c
│        │  │  │  │  ├─ stm32h7xx_hal_nand.h
│        │  │  │  │  ├─ stm32h7xx_hal_nor.c
│        │  │  │  │  ├─ stm32h7xx_hal_nor.h
│        │  │  │  │  ├─ stm32h7xx_hal_opamp.h
│        │  │  │  │  ├─ stm32h7xx_hal_opamp_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_pcd.h
│        │  │  │  │  ├─ stm32h7xx_hal_pcd_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_pwr.c
│        │  │  │  │  ├─ stm32h7xx_hal_pwr.h
│        │  │  │  │  ├─ stm32h7xx_hal_pwr_ex.c
│        │  │  │  │  ├─ stm32h7xx_hal_pwr_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_qspi.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc.c
│        │  │  │  │  ├─ stm32h7xx_hal_rcc.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc_ex.c
│        │  │  │  │  ├─ stm32h7xx_hal_rcc_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_rng.h
│        │  │  │  │  ├─ stm32h7xx_hal_rtc.h
│        │  │  │  │  ├─ stm32h7xx_hal_rtc_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_sai.h
│        │  │  │  │  ├─ stm32h7xx_hal_sai_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_sd.h
│        │  │  │  │  ├─ stm32h7xx_hal_sdram.h
│        │  │  │  │  ├─ stm32h7xx_hal_sd_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_smartcard.h
│        │  │  │  │  ├─ stm32h7xx_hal_smartcard_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_smbus.h
│        │  │  │  │  ├─ stm32h7xx_hal_spdifrx.h
│        │  │  │  │  ├─ stm32h7xx_hal_spi.h
│        │  │  │  │  ├─ stm32h7xx_hal_spi_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_sram.h
│        │  │  │  │  ├─ stm32h7xx_hal_swpmi.h
│        │  │  │  │  ├─ stm32h7xx_hal_tim.h
│        │  │  │  │  ├─ stm32h7xx_hal_tim_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_uart.h
│        │  │  │  │  ├─ stm32h7xx_hal_uart_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_usart.h
│        │  │  │  │  ├─ stm32h7xx_hal_usart_ex.h
│        │  │  │  │  ├─ stm32h7xx_hal_wwdg.h
│        │  │  │  │  ├─ stm32h7xx_it.c
│        │  │  │  │  ├─ stm32h7xx_it.h
│        │  │  │  │  ├─ stm32h7xx_ll_delayblock.h
│        │  │  │  │  ├─ stm32h7xx_ll_fmc.c
│        │  │  │  │  ├─ stm32h7xx_ll_fmc.h
│        │  │  │  │  ├─ stm32h7xx_ll_sdmmc.h
│        │  │  │  │  ├─ stm32h7xx_ll_usb.h
│        │  │  │  │  ├─ system_stm32h7xx.c
│        │  │  │  │  └─ system_stm32h7xx.h
│        │  │  │  ├─ STM32H743I-eval_FMC.uvoptx
│        │  │  │  ├─ STM32H743I-eval_FMC.uvprojx
│        │  │  │  ├─ Target.lin
│        │  │  │  └─ Target_Dbg.lin
│        │  │  ├─ STM32H7A-B3
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashOS.h
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ inc
│        │  │  │  │  ├─ core_cm7.h
│        │  │  │  │  ├─ core_cmSimd.h
│        │  │  │  │  ├─ core_pelican.h
│        │  │  │  │  ├─ flash_config.h
│        │  │  │  │  ├─ Loader_Src.h
│        │  │  │  │  ├─ stm32f4xx.h
│        │  │  │  │  ├─ stm32f4xx_conf.h
│        │  │  │  │  ├─ stm32f4xx_flash.h
│        │  │  │  │  ├─ stm32f745xx.h
│        │  │  │  │  ├─ stm32f746xx.h
│        │  │  │  │  ├─ stm32f756xx.h
│        │  │  │  │  ├─ stm32f7xx.h
│        │  │  │  │  ├─ stm32h700.h
│        │  │  │  │  ├─ stm32h70x.h
│        │  │  │  │  ├─ stm32h71xx.h
│        │  │  │  │  ├─ stm32h743xx.h
│        │  │  │  │  ├─ stm32h776xx.h
│        │  │  │  │  ├─ stm32h7xx.h
│        │  │  │  │  ├─ stm32h7xx_hal.h
│        │  │  │  │  ├─ stm32h7xx_hal_def.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc_ex.h
│        │  │  │  │  ├─ stm32_hal_legacy.h
│        │  │  │  │  ├─ system_stm32f4xx.h
│        │  │  │  │  ├─ system_stm32f7xx.h
│        │  │  │  │  └─ system_stm32h7xx.h
│        │  │  │  ├─ STM32H7xx_CM7.uvoptx
│        │  │  │  ├─ STM32H7xx_CM7.uvprojx
│        │  │  │  ├─ Target.lin
│        │  │  │  └─ Target_Dbg.lin
│        │  │  ├─ STM32H7A-B3_Flash_1M.FLM
│        │  │  ├─ STM32H7A-B3_Flash_2M.FLM
│        │  │  ├─ STM32H7B0_Flash.FLM
│        │  │  ├─ STM32H7B3I-B0_Disco_OSPI
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ OSPI
│        │  │  │  │  ├─ mx25lm51245g_conf.h
│        │  │  │  │  ├─ stm32h7b3i_discovery_conf.h
│        │  │  │  │  ├─ stm32h7b3i_discovery_ospi.c
│        │  │  │  │  ├─ STM32H7OSPI.c
│        │  │  │  │  ├─ STM32H7OSPI.h
│        │  │  │  │  └─ stm32h7xx_hal_conf.h
│        │  │  │  ├─ STM32H7B3I_DISCO.uvoptx
│        │  │  │  ├─ STM32H7B3I_DISCO.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H7B3I-B0_EVAL_OSPI
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ OSPI
│        │  │  │  │  ├─ mx25lm51245g_conf.h
│        │  │  │  │  ├─ stm32h7b3i_eval_conf.h
│        │  │  │  │  ├─ stm32h7b3i_eval_ospi.c
│        │  │  │  │  ├─ STM32H7OSPI.c
│        │  │  │  │  ├─ STM32H7OSPI.h
│        │  │  │  │  └─ stm32h7xx_hal_conf.h
│        │  │  │  ├─ STM32H7B3I-B0I_EVAL.uvoptx
│        │  │  │  ├─ STM32H7B3I-B0I_EVAL.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H7B3I_EVAL_FMC-NOR.FLM
│        │  │  ├─ STM32H7xx
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ STM32H7xx.uvoptx
│        │  │  │  ├─ STM32H7xx.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H7xx.FLM
│        │  │  ├─ STM32H7xx_CM4
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashOS.h
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ inc
│        │  │  │  │  ├─ cmsis_armcc.h
│        │  │  │  │  ├─ core_cm4.h
│        │  │  │  │  ├─ core_cmFunc.h
│        │  │  │  │  ├─ core_cmInstr.h
│        │  │  │  │  ├─ core_cmSimd.h
│        │  │  │  │  ├─ core_pelican.h
│        │  │  │  │  ├─ flash_config.h
│        │  │  │  │  ├─ Loader_Src.h
│        │  │  │  │  ├─ stm32f4xx.h
│        │  │  │  │  ├─ stm32f4xx_conf.h
│        │  │  │  │  ├─ stm32f4xx_flash.h
│        │  │  │  │  ├─ stm32f745xx.h
│        │  │  │  │  ├─ stm32f746xx.h
│        │  │  │  │  ├─ stm32f756xx.h
│        │  │  │  │  ├─ stm32f7xx.h
│        │  │  │  │  ├─ stm32h743xx.h
│        │  │  │  │  ├─ stm32h776xx.h
│        │  │  │  │  ├─ stm32h7xx.h
│        │  │  │  │  ├─ stm32h7xx_hal.h
│        │  │  │  │  ├─ stm32h7xx_hal_def.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc_ex.h
│        │  │  │  │  ├─ stm32_hal_legacy.h
│        │  │  │  │  ├─ system_stm32f4xx.h
│        │  │  │  │  ├─ system_stm32f7xx.h
│        │  │  │  │  └─ system_stm32h7xx.h
│        │  │  │  ├─ STM32H7xx_CM4.uvoptx
│        │  │  │  ├─ STM32H7xx_CM4.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H7xx_CM4.FLM
│        │  │  ├─ STM32H7xx_CM7
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashOS.h
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ inc
│        │  │  │  │  ├─ core_cm7.h
│        │  │  │  │  ├─ core_cmSimd.h
│        │  │  │  │  ├─ core_pelican.h
│        │  │  │  │  ├─ flash_config.h
│        │  │  │  │  ├─ Loader_Src.h
│        │  │  │  │  ├─ stm32f4xx.h
│        │  │  │  │  ├─ stm32f4xx_conf.h
│        │  │  │  │  ├─ stm32f4xx_flash.h
│        │  │  │  │  ├─ stm32f745xx.h
│        │  │  │  │  ├─ stm32f746xx.h
│        │  │  │  │  ├─ stm32f756xx.h
│        │  │  │  │  ├─ stm32f7xx.h
│        │  │  │  │  ├─ stm32h743xx.h
│        │  │  │  │  ├─ stm32h776xx.h
│        │  │  │  │  ├─ stm32h7xx.h
│        │  │  │  │  ├─ stm32h7xx_hal.h
│        │  │  │  │  ├─ stm32h7xx_hal_def.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc.h
│        │  │  │  │  ├─ stm32h7xx_hal_rcc_ex.h
│        │  │  │  │  ├─ stm32_hal_legacy.h
│        │  │  │  │  ├─ system_stm32f4xx.h
│        │  │  │  │  ├─ system_stm32f7xx.h
│        │  │  │  │  └─ system_stm32h7xx.h
│        │  │  │  ├─ STM32H7xx_CM7.uvoptx
│        │  │  │  ├─ STM32H7xx_CM7.uvprojx
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H7xx_CM7.FLM
│        │  │  ├─ STM32H7xx_Flasher
│        │  │  │  ├─ core_cmSimd.h
│        │  │  │  ├─ core_pelican.h
│        │  │  │  ├─ FlashDev.c
│        │  │  │  ├─ FlashPrg.c
│        │  │  │  ├─ stm32h776xx.h
│        │  │  │  ├─ stm32h7xx.h
│        │  │  │  ├─ STM32H7xx.uvopt
│        │  │  │  ├─ STM32H7xx.uvproj
│        │  │  │  ├─ system_stm32h7xx.h
│        │  │  │  └─ Target.lin
│        │  │  ├─ STM32H7xx_MT25TL01G.FLM
│        │  │  ├─ STM32H7xx_MT25TL01G_DUAL.FLM
│        │  │  ├─ STM32H7x_128k.FLM
│        │  │  └─ STM32H7x_2048.FLM
│        │  └─ SVD
│        │     ├─ Release_Notes.htm
│        │     ├─ Release_Notes_files
│        │     │  ├─ colorschememapping.xml
│        │     │  ├─ filelist.xml
│        │     │  └─ themedata.thmx
│        │     ├─ STM32H723.svd
│        │     ├─ STM32H725.svd
│        │     ├─ STM32H730.svd
│        │     ├─ STM32H733.svd
│        │     ├─ STM32H735.svd
│        │     ├─ STM32H73x.svd
│        │     ├─ STM32H742.svd
│        │     ├─ STM32H742x.svd
│        │     ├─ STM32H743.svd
│        │     ├─ STM32H745_CM4.svd
│        │     ├─ STM32H745_CM7.svd
│        │     ├─ STM32H747_CM4.svd
│        │     ├─ STM32H747_CM7.svd
│        │     ├─ STM32H750.svd
│        │     ├─ STM32H753.svd
│        │     ├─ STM32H755_CM4.svd
│        │     ├─ STM32H755_CM7.svd
│        │     ├─ STM32H757_CM4.svd
│        │     ├─ STM32H757_CM7.svd
│        │     ├─ STM32H7A3.svd
│        │     ├─ STM32H7B0.svd
│        │     └─ STM32H7B3.svd
│        ├─ Keil.STM32H7xx_DFP.pdsc
│        ├─ Keil.STM32H7xx_DFP.sha1
│        ├─ LICENSE-Apache-2.0
│        ├─ LICENSE-BSD-3-Clause
│        ├─ LICENSES
│        └─ Templates
│           ├─ CubeMX
│           │  ├─ CubeMX.cproject.yml
│           │  └─ CubeMX.csolution.yml
│           └─ CubeMX_DualCore
│              ├─ CM4
│              │  └─ CM4.cproject.yml
│              ├─ CM7
│              │  └─ CM7.cproject.yml
│              └─ CubeMX_DualCore.csolution.yml
├─ 2026_RC_TEST.code-workspace
├─ 2026_RC_TEST.ioc
├─ cmake
│  ├─ gcc-arm-none-eabi.cmake
│  ├─ starm-clang.cmake
│  └─ stm32cubemx
│     └─ CMakeLists.txt
├─ CMakeLists.txt
├─ CMakePresets.json
├─ Core
│  ├─ Inc
│  │  ├─ dma.h
│  │  ├─ fdcan.h
│  │  ├─ FreeRTOSConfig.h
│  │  ├─ gpio.h
│  │  ├─ main.h
│  │  ├─ stm32h7xx_hal_conf.h
│  │  ├─ stm32h7xx_it.h
│  │  ├─ tim.h
│  │  └─ usart.h
│  └─ Src
│     ├─ dma.c
│     ├─ fdcan.c
│     ├─ freertos.c
│     ├─ gpio.c
│     ├─ main.c
│     ├─ stm32h7xx_hal_msp.c
│     ├─ stm32h7xx_hal_timebase_tim.c
│     ├─ stm32h7xx_it.c
│     ├─ syscalls.c
│     ├─ sysmem.c
│     ├─ system_stm32h7xx.c
│     ├─ tim.c
│     └─ usart.c
├─ Drivers
│  ├─ CMSIS
│  │  ├─ Device
│  │  │  └─ ST
│  │  │     └─ STM32H7xx
│  │  │        ├─ Include
│  │  │        │  ├─ stm32h723xx.h
│  │  │        │  ├─ stm32h7xx.h
│  │  │        │  └─ system_stm32h7xx.h
│  │  │        └─ LICENSE.txt
│  │  ├─ Include
│  │  │  ├─ cachel1_armv7.h
│  │  │  ├─ cmsis_armcc.h
│  │  │  ├─ cmsis_armclang.h
│  │  │  ├─ cmsis_armclang_ltm.h
│  │  │  ├─ cmsis_compiler.h
│  │  │  ├─ cmsis_gcc.h
│  │  │  ├─ cmsis_iccarm.h
│  │  │  ├─ cmsis_version.h
│  │  │  ├─ core_armv81mml.h
│  │  │  ├─ core_armv8mbl.h
│  │  │  ├─ core_armv8mml.h
│  │  │  ├─ core_cm0.h
│  │  │  ├─ core_cm0plus.h
│  │  │  ├─ core_cm1.h
│  │  │  ├─ core_cm23.h
│  │  │  ├─ core_cm3.h
│  │  │  ├─ core_cm33.h
│  │  │  ├─ core_cm35p.h
│  │  │  ├─ core_cm4.h
│  │  │  ├─ core_cm55.h
│  │  │  ├─ core_cm7.h
│  │  │  ├─ core_cm85.h
│  │  │  ├─ core_sc000.h
│  │  │  ├─ core_sc300.h
│  │  │  ├─ core_starmc1.h
│  │  │  ├─ mpu_armv7.h
│  │  │  ├─ mpu_armv8.h
│  │  │  ├─ pac_armv81.h
│  │  │  ├─ pmu_armv8.h
│  │  │  └─ tz_context.h
│  │  ├─ LICENSE.txt
│  │  └─ RTOS2
│  │     └─ Include
│  │        ├─ cmsis_os2.h
│  │        └─ os_tick.h
│  └─ STM32H7xx_HAL_Driver
│     ├─ Inc
│     │  ├─ Legacy
│     │  │  └─ stm32_hal_legacy.h
│     │  ├─ stm32h7xx_hal.h
│     │  ├─ stm32h7xx_hal_cortex.h
│     │  ├─ stm32h7xx_hal_def.h
│     │  ├─ stm32h7xx_hal_dma.h
│     │  ├─ stm32h7xx_hal_dma_ex.h
│     │  ├─ stm32h7xx_hal_exti.h
│     │  ├─ stm32h7xx_hal_fdcan.h
│     │  ├─ stm32h7xx_hal_flash.h
│     │  ├─ stm32h7xx_hal_flash_ex.h
│     │  ├─ stm32h7xx_hal_gpio.h
│     │  ├─ stm32h7xx_hal_gpio_ex.h
│     │  ├─ stm32h7xx_hal_hsem.h
│     │  ├─ stm32h7xx_hal_i2c.h
│     │  ├─ stm32h7xx_hal_i2c_ex.h
│     │  ├─ stm32h7xx_hal_mdma.h
│     │  ├─ stm32h7xx_hal_pcd.h
│     │  ├─ stm32h7xx_hal_pcd_ex.h
│     │  ├─ stm32h7xx_hal_pwr.h
│     │  ├─ stm32h7xx_hal_pwr_ex.h
│     │  ├─ stm32h7xx_hal_rcc.h
│     │  ├─ stm32h7xx_hal_rcc_ex.h
│     │  ├─ stm32h7xx_hal_tim.h
│     │  ├─ stm32h7xx_hal_tim_ex.h
│     │  ├─ stm32h7xx_hal_uart.h
│     │  ├─ stm32h7xx_hal_uart_ex.h
│     │  ├─ stm32h7xx_ll_bus.h
│     │  ├─ stm32h7xx_ll_cortex.h
│     │  ├─ stm32h7xx_ll_crs.h
│     │  ├─ stm32h7xx_ll_dma.h
│     │  ├─ stm32h7xx_ll_dmamux.h
│     │  ├─ stm32h7xx_ll_exti.h
│     │  ├─ stm32h7xx_ll_gpio.h
│     │  ├─ stm32h7xx_ll_hsem.h
│     │  ├─ stm32h7xx_ll_lpuart.h
│     │  ├─ stm32h7xx_ll_pwr.h
│     │  ├─ stm32h7xx_ll_rcc.h
│     │  ├─ stm32h7xx_ll_system.h
│     │  ├─ stm32h7xx_ll_tim.h
│     │  ├─ stm32h7xx_ll_usart.h
│     │  ├─ stm32h7xx_ll_usb.h
│     │  └─ stm32h7xx_ll_utils.h
│     ├─ LICENSE.txt
│     └─ Src
│        ├─ stm32h7xx_hal.c
│        ├─ stm32h7xx_hal_cortex.c
│        ├─ stm32h7xx_hal_dma.c
│        ├─ stm32h7xx_hal_dma_ex.c
│        ├─ stm32h7xx_hal_exti.c
│        ├─ stm32h7xx_hal_fdcan.c
│        ├─ stm32h7xx_hal_flash.c
│        ├─ stm32h7xx_hal_flash_ex.c
│        ├─ stm32h7xx_hal_gpio.c
│        ├─ stm32h7xx_hal_hsem.c
│        ├─ stm32h7xx_hal_i2c.c
│        ├─ stm32h7xx_hal_i2c_ex.c
│        ├─ stm32h7xx_hal_mdma.c
│        ├─ stm32h7xx_hal_pcd.c
│        ├─ stm32h7xx_hal_pcd_ex.c
│        ├─ stm32h7xx_hal_pwr.c
│        ├─ stm32h7xx_hal_pwr_ex.c
│        ├─ stm32h7xx_hal_rcc.c
│        ├─ stm32h7xx_hal_rcc_ex.c
│        ├─ stm32h7xx_hal_tim.c
│        ├─ stm32h7xx_hal_tim_ex.c
│        ├─ stm32h7xx_hal_uart.c
│        ├─ stm32h7xx_hal_uart_ex.c
│        └─ stm32h7xx_ll_usb.c
├─ ld
│  ├─ startup_stm32h723xx.s
│  └─ STM32H723VGTx_FLASH.ld
├─ Middlewares
│  ├─ ST
│  │  └─ STM32_USB_Device_Library
│  │     ├─ Class
│  │     │  └─ CDC
│  │     │     ├─ Inc
│  │     │     │  └─ usbd_cdc.h
│  │     │     └─ Src
│  │     │        └─ usbd_cdc.c
│  │     ├─ Core
│  │     │  ├─ Inc
│  │     │  │  ├─ usbd_core.h
│  │     │  │  ├─ usbd_ctlreq.h
│  │     │  │  ├─ usbd_def.h
│  │     │  │  └─ usbd_ioreq.h
│  │     │  └─ Src
│  │     │     ├─ usbd_core.c
│  │     │     ├─ usbd_ctlreq.c
│  │     │     └─ usbd_ioreq.c
│  │     └─ LICENSE.txt
│  └─ Third_Party
│     └─ FreeRTOS
│        └─ Source
│           ├─ CMSIS_RTOS_V2
│           │  ├─ cmsis_os.h
│           │  ├─ cmsis_os2.c
│           │  ├─ freertos_mpool.h
│           │  └─ freertos_os2.h
│           ├─ croutine.c
│           ├─ event_groups.c
│           ├─ include
│           │  ├─ atomic.h
│           │  ├─ croutine.h
│           │  ├─ deprecated_definitions.h
│           │  ├─ event_groups.h
│           │  ├─ FreeRTOS.h
│           │  ├─ list.h
│           │  ├─ message_buffer.h
│           │  ├─ mpu_prototypes.h
│           │  ├─ mpu_syscall_numbers.h
│           │  ├─ mpu_wrappers.h
│           │  ├─ newlib-freertos.h
│           │  ├─ picolibc-freertos.h
│           │  ├─ portable.h
│           │  ├─ projdefs.h
│           │  ├─ queue.h
│           │  ├─ semphr.h
│           │  ├─ StackMacros.h
│           │  ├─ stack_macros.h
│           │  ├─ stream_buffer.h
│           │  ├─ task.h
│           │  └─ timers.h
│           ├─ LICENSE
│           ├─ list.c
│           ├─ portable
│           │  ├─ Common
│           │  │  └─ mpu_wrappers_v2.c
│           │  ├─ GCC
│           │  │  └─ ARM_CM4F
│           │  │     ├─ port.c
│           │  │     └─ portmacro.h
│           │  └─ MemMang
│           │     └─ heap_4.c
│           ├─ queue.c
│           ├─ stream_buffer.c
│           ├─ tasks.c
│           └─ timers.c
├─ readme.md
├─ RTE_Components.h
├─ startup_stm32h723xx.s
├─ STM32H723XG_FLASH.ld
├─ USB_DEVICE
│  ├─ App
│  │  ├─ usbd_cdc_if.c
│  │  ├─ usbd_cdc_if.h
│  │  ├─ usbd_desc.c
│  │  ├─ usbd_desc.h
│  │  ├─ usb_device.c
│  │  └─ usb_device.h
│  └─ Target
│     ├─ usbd_conf.c
│     └─ usbd_conf.h
├─ User_Ctrl
│  ├─ Inc
│  │  ├─ common.h
│  │  ├─ user_communication.h
│  │  ├─ user_control.h
│  │  ├─ user_task.h
│  │  ├─ user_usart.h
│  │  └─ zlr_math.h
│  └─ Src
│     ├─ user_communication.c
│     ├─ user_control.c
│     ├─ user_task.c
│     ├─ user_usart.c
│     └─ zlr_math.c
├─ User_Driver
│  ├─ Inc
│  │  ├─ chassis_driver.h
│  │  ├─ motor_driver.h
│  │  └─ user_FDCAN.h
│  └─ Src
│     ├─ chassis_driver.c
│     ├─ motor_driver.c
│     └─ user_FDCAN.c
└─ Vofa
   ├─ vofa.c
   └─ vofa.h

```
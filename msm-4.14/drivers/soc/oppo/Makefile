obj-$(CONFIG_OPPO_COMMON_SOFT)	+=  oppo_project/
obj-$(CONFIG_OPPO_DEVICE_IFNO)	+=  oppo_devinfo/
obj-$(CONFIG_OPPO_DEBUG_ASSERT)	+=  oppo_trace/
obj-$(CONFIG_RECORD_MDMRST)     +=  oppo_mdmrst/
obj-$(CONFIG_OPPO_HYPNUS)     +=  oppo_hypnus/
#//jie.cheng@swdh.sh, 2016/04/15, Add hypnus helper
#obj-y     +=  oppo_hypnus/
#ifdef VENDOR_EDIT
#//wenbin.liu@PSW.Kernel.MM, 2018/05/23, Add for Healthinfo monitor
obj-y     +=  oppo_healthinfo/
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
#wen.luo@PSW.Kernel.MM, 2018/05/23, Add for Healthinfo monitor
#Liang.Zhang@TECH.Storage.Stability.OPPO_SHUTDOWN_DETECT, 2019/04/28, Add for shutdown detect
obj-$(CONFIG_OPPO_SHUTDOWN_DETECT)	+=  oppo_shutdown_detect/
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
#//Zhaoan.Xu@PSW.MM.AudioDriver.Other, 2019/01/16, Add for criticallog
obj-y +=  oppo_criticallog/

#GaoTing.Gan@PSW.MultiMedia.MediaServer, 2019/03/07, Add for kevent feedback

obj-$(CONFIG_OPPO_KEVENT_UPLOAD) +=  oppo_kevent_feedback/
#endif /* VENDOR_EDIT */

#Kun.Hu@TECH.BSP.Stability.PHOENIX_PROJECT 2019/06/15, Add for PHOENIX_PROJECT
obj-$(CONFIG_OPPO_PHOENIX)     +=  oppo_phoenix/

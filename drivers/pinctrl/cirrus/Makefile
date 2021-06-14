# Cirrus Logic pinctrl drivers
madera-pinctrl-objs		:= pinctrl-madera.o
ifeq ($(CONFIG_PINCTRL_CS47L35),y)
madera-pinctrl-objs		+= pinctrl-cs47l35.o
endif
ifeq ($(CONFIG_PINCTRL_CS47L85),y)
madera-pinctrl-objs		+= pinctrl-cs47l85.o
endif
ifeq ($(CONFIG_PINCTRL_CS47L90),y)
madera-pinctrl-objs		+= pinctrl-cs47l90.o
endif

obj-$(CONFIG_PINCTRL_MADERA)	+= madera-pinctrl.o

#
# (C) Copyright 2000-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# (C) Copyright 2011 Freescale Semiconductor, Inc.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB     = $(obj)libimx-common.o

ifeq ($(SOC),$(filter $(SOC),mx5 mx6))
COBJS-y	= iomux-v3.o timer.o cpu.o speed.o
COBJS-$(CONFIG_I2C_MXC) += i2c-mxv7.o
endif
COBJS-$(CONFIG_CMD_BMODE) += cmd_bmode.o
COBJS-$(CONFIG_CMD_HDMIDETECT) += cmd_hdmidet.o
COBJS	:= $(sort $(COBJS-y))

SRCS	:= $(SOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(SOBJS) $(COBJS))

all:	$(obj).depend $(LIB)

$(LIB):	$(OBJS)
	$(call cmd_link_o_target, $(OBJS))

$(OBJTREE)/$(patsubst "%",%,$(CONFIG_IMX_CONFIG)).cfgtmp: $(OBJTREE)/%.cfgtmp : $(SRCTREE)/%
	mkdir -p $(dir $@)
	$(CC) -E -x c $< $(CPPFLAGS) -o $@

$(OBJTREE)/u-boot.imx: $(OBJTREE)/u-boot.bin $(OBJTREE)/$(patsubst "%",%,$(CONFIG_IMX_CONFIG)).cfgtmp
	$(OBJTREE)/tools/mkimage -n $(filter-out %.bin,$^) -T imximage \
	-e $(CONFIG_SYS_TEXT_BASE) -d $< $@

$(OBJTREE)/SPL: $(OBJTREE)/spl/u-boot-spl.bin $(OBJTREE)/$(patsubst "%",%,$(CONFIG_IMX_CONFIG)).cfgtmp
	$(OBJTREE)/tools/mkimage -n $(filter-out %.bin,$^) -T imximage \
	-e $(CONFIG_SPL_TEXT_BASE) -d $< $@

$(OBJTREE)/u-boot-with-spl.imx: $(OBJTREE)/SPL $(OBJTREE)/u-boot.bin
	$(OBJCOPY) ${OBJCFLAGS} --pad-to=$(CONFIG_SPL_PAD_TO) \
		-I binary -O binary $< $(OBJTREE)/spl/u-boot-spl-pad.imx
	cat $(OBJTREE)/spl/u-boot-spl-pad.imx $(OBJTREE)/u-boot.bin > $@
	rm $(OBJTREE)/spl/u-boot-spl-pad.imx

$(OBJTREE)/u-boot-with-nand-spl.imx: $(OBJTREE)/SPL $(OBJTREE)/u-boot.bin
	(echo -ne '\x00\x00\x00\x00\x46\x43\x42\x20\x01' && \
			dd bs=1015 count=1 if=/dev/zero 2>/dev/null) | \
		cat - $< > $(OBJTREE)/spl/u-boot-nand-spl.imx
	$(OBJCOPY) ${OBJCFLAGS} --pad-to=$(CONFIG_SPL_PAD_TO) \
		-I binary -O binary $(OBJTREE)/spl/u-boot-nand-spl.imx \
		$(OBJTREE)/spl/u-boot-nand-spl-pad.imx
	rm $(OBJTREE)/spl/u-boot-nand-spl.imx
	cat $(OBJTREE)/spl/u-boot-nand-spl-pad.imx $(OBJTREE)/u-boot.bin > $@
	rm $(OBJTREE)/spl/u-boot-nand-spl-pad.imx


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

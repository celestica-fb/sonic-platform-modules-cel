/*
 * Intel PCH/PCU SPI flash PCI driver.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "intel-spi.h"

#define BCR		0xdc
#define BCR_WPD		BIT(0)

static const struct intel_spi_boardinfo bxt_info = {
	.type = INTEL_SPI_BXT,
};

/* ++ add by maxwill for tpm reg access ++ */
struct intel_spi *intel_spi_ctrl;
u8 spi_tpm_opcode;
static ssize_t getreg_tpm_store(struct device *dev, struct device_attribute *devattr,
                const char *buf, size_t count)
{
    u8 opcode;
    char *last;

    opcode = (uint16_t)strtoul(buf,&last,16);
    if(opcode == 0 && buf == last){
		printk(KERN_INFO "getreg_tpm: get opcode error!\n");
        return -EINVAL;
    }
    spi_tpm_opcode = opcode;
    return count;
}

static ssize_t getreg_tpm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = 0;
	u8 reg_val = 0;

    intel_spi_ctrl->nor.read_reg(&intel_spi_ctrl->nor, spi_tpm_opcode, &reg_val, 1);
    len = sprintf(buf, "0x%2.2x\n", reg_val);

    return len;
}
static DEVICE_ATTR_RW(getreg_tpm);

static ssize_t setreg_tpm_store(struct device *dev, struct device_attribute *devattr,
                const u8 *buf, size_t count)
{
    u8 opcode;
    u8 value;
    u8 *tok;
    u8 clone[count];
    u8 *pclone = clone;
    u8 *last;

    strcpy(clone, buf);

    tok = strsep((char**)&pclone, " ");
    if(tok == NULL){
		printk(KERN_INFO "setreg_tpm: get opcode error1!\n");
        return -EINVAL;
    }
    opcode = (uint16_t)strtoul(tok,&last,16);
    if(opcode == 0 && tok == last){
		printk(KERN_INFO "setreg_tpm: get opcode error2!\n");
        return -EINVAL;
    }

    tok = strsep((char**)&pclone, " ");
    if(tok == NULL){
		printk(KERN_INFO "setreg_tpm: get value error1!\n");
        return -EINVAL;
    }
    value = (uint8_t)strtoul(tok,&last,16);
    if(value == 0 && tok == last){
        printk(KERN_INFO "setreg_tpm: get value error2!\n");
        return -EINVAL;
    }

	intel_spi_ctrl->nor.write_reg(&intel_spi_ctrl->nor, spi_tpm_opcode, &value, 1);

    return count;
}
static DEVICE_ATTR_WO(setreg_tpm);
static struct attribute *spi_tpm_attrs[] = {
    &dev_attr_getreg.attr,
    &dev_attr_setreg.attr,
    NULL,
};
static struct attribute_group spi_tpm_attrs_grp = {
    .attrs = spi_tpm_attrs,
};

/* -- add by maxwill for tpm reg access   --  */

static int intel_spi_pci_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	struct intel_spi_boardinfo *info;
	struct intel_spi *ispi;
	u32 bcr;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	info = devm_kmemdup(&pdev->dev, (void *)id->driver_data, sizeof(*info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* Try to make the chip read/write */
	pci_read_config_dword(pdev, BCR, &bcr);
	if (!(bcr & BCR_WPD)) {
		bcr |= BCR_WPD;
		pci_write_config_dword(pdev, BCR, bcr);
		pci_read_config_dword(pdev, BCR, &bcr);
	}
	info->writeable = !!(bcr & BCR_WPD);

	ispi = intel_spi_probe(&pdev->dev, &pdev->resource[0], info);

	/* ++ add by maxwill for tpm reg access   ++  */
    intel_spi_ctrl = ispi;
	ret = sysfs_create_group(&pdev->dev.kobj, &spi_tpm_attrs_grp);
    if (ret) {
        printk(KERN_ERR "Cannot create sysfs for spi-tpm device.\n");
        return ret;
    }
	/* -- add by maxwill for tpm reg access   --  */
	
	if (IS_ERR(ispi))
		return PTR_ERR(ispi);

	pci_set_drvdata(pdev, ispi);
	return 0;
}

static void intel_spi_pci_remove(struct pci_dev *pdev)
{
	intel_spi_remove(pci_get_drvdata(pdev));
}

static const struct pci_device_id intel_spi_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x18e0), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x19e0), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0x34a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0xa1a4), (unsigned long)&bxt_info },
	{ PCI_VDEVICE(INTEL, 0xa224), (unsigned long)&bxt_info },
	{ },
};
MODULE_DEVICE_TABLE(pci, intel_spi_pci_ids);

static struct pci_driver intel_spi_pci_driver = {
	.name = "intel-spi",
	.id_table = intel_spi_pci_ids,
	.probe = intel_spi_pci_probe,
	.remove = intel_spi_pci_remove,
};

module_pci_driver(intel_spi_pci_driver);

MODULE_DESCRIPTION("Intel PCH/PCU SPI flash PCI driver");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_LICENSE("GPL v2");

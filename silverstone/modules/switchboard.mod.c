#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xd1a11d1c, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x9c87d754, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0x9b0a71de, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0x7a4497db, __VMLINUX_SYMBOL_STR(kzfree) },
	{ 0x873b57aa, __VMLINUX_SYMBOL_STR(device_create_with_groups) },
	{ 0xde1f1681, __VMLINUX_SYMBOL_STR(i2c_new_device) },
	{ 0xa9906b1d, __VMLINUX_SYMBOL_STR(i2c_add_numbered_adapter) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0xe52cb2e9, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x660b9bcf, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xf2cd970c, __VMLINUX_SYMBOL_STR(sysfs_create_link) },
	{ 0xae59a99, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0x691b6183, __VMLINUX_SYMBOL_STR(kobject_create_and_add) },
	{ 0x888680a0, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x32a81c89, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x222e7ce2, __VMLINUX_SYMBOL_STR(sysfs_streq) },
	{ 0xe638537a, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0xf10de535, __VMLINUX_SYMBOL_STR(ioread8) },
	{ 0x15ba50a6, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x8a2b6c2c, __VMLINUX_SYMBOL_STR(__platform_driver_register) },
	{ 0x615b186c, __VMLINUX_SYMBOL_STR(platform_device_register) },
	{ 0xbaaf840a, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0xc01673d6, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0xeeeef437, __VMLINUX_SYMBOL_STR(__register_chrdev) },
	{ 0x4b82759, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xb81c3e89, __VMLINUX_SYMBOL_STR(pci_iomap) },
	{ 0xd8f334bc, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0x260fe3ff, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x76cc02fc, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0xed93ccc6, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0xed11a40, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0x241428f1, __VMLINUX_SYMBOL_STR(pci_iounmap) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x6bc3fbc0, __VMLINUX_SYMBOL_STR(__unregister_chrdev) },
	{ 0x506ef310, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x3a6b90e8, __VMLINUX_SYMBOL_STR(class_unregister) },
	{ 0xd279ba63, __VMLINUX_SYMBOL_STR(platform_device_unregister) },
	{ 0xa0433898, __VMLINUX_SYMBOL_STR(platform_driver_unregister) },
	{ 0xd7cd578b, __VMLINUX_SYMBOL_STR(devm_kfree) },
	{ 0xf3e1947, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0xf2e07ed1, __VMLINUX_SYMBOL_STR(kobject_put) },
	{ 0x9b242c96, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x79198d4, __VMLINUX_SYMBOL_STR(put_device) },
	{ 0x8d240dc5, __VMLINUX_SYMBOL_STR(device_unregister) },
	{ 0xefd0a108, __VMLINUX_SYMBOL_STR(i2c_del_adapter) },
	{ 0xf9638bb0, __VMLINUX_SYMBOL_STR(i2c_unregister_device) },
	{ 0xfdeafc1d, __VMLINUX_SYMBOL_STR(sysfs_remove_link) },
	{ 0xc671e369, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xb5419b40, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0x1b17e06c, __VMLINUX_SYMBOL_STR(kstrtoll) },
	{ 0x436c2179, __VMLINUX_SYMBOL_STR(iowrite32) },
	{ 0x727c4f3, __VMLINUX_SYMBOL_STR(iowrite8) },
	{ 0x85df9b6c, __VMLINUX_SYMBOL_STR(strsep) },
	{ 0xe914e41e, __VMLINUX_SYMBOL_STR(strcpy) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x94218140, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0xe484e35f, __VMLINUX_SYMBOL_STR(ioread32) },
	{ 0x86b8d7be, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v000010EEd00007021sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00001AF4d00001110sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "AC8F9A3E5DEE6E1D1BC429F");

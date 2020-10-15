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
	{ 0xd279ba63, __VMLINUX_SYMBOL_STR(platform_device_unregister) },
	{ 0xa0433898, __VMLINUX_SYMBOL_STR(platform_driver_unregister) },
	{ 0x8a2b6c2c, __VMLINUX_SYMBOL_STR(__platform_driver_register) },
	{ 0x615b186c, __VMLINUX_SYMBOL_STR(platform_device_register) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xf2cd970c, __VMLINUX_SYMBOL_STR(sysfs_create_link) },
	{ 0x873b57aa, __VMLINUX_SYMBOL_STR(device_create_with_groups) },
	{ 0xc01673d6, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0xae59a99, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0x1116e1de, __VMLINUX_SYMBOL_STR(platform_get_resource) },
	{ 0x888680a0, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x32a81c89, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x1b17e06c, __VMLINUX_SYMBOL_STR(kstrtoll) },
	{ 0x85df9b6c, __VMLINUX_SYMBOL_STR(strsep) },
	{ 0xe914e41e, __VMLINUX_SYMBOL_STR(strcpy) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0x222e7ce2, __VMLINUX_SYMBOL_STR(sysfs_streq) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x94218140, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x86b8d7be, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x506ef310, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x9b242c96, __VMLINUX_SYMBOL_STR(sysfs_remove_group) },
	{ 0x79198d4, __VMLINUX_SYMBOL_STR(put_device) },
	{ 0x8d240dc5, __VMLINUX_SYMBOL_STR(device_unregister) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "24C65014F71176DAA1B4B31");

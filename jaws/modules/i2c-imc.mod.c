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
	{ 0x8f71ce78, __VMLINUX_SYMBOL_STR(param_ops_bool) },
	{ 0x9c87d754, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0x9b0a71de, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0x22ae8055, __VMLINUX_SYMBOL_STR(i2c_scan_dimm_bus) },
	{ 0xcaf2c02c, __VMLINUX_SYMBOL_STR(i2c_add_adapter) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x888680a0, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xe561bce0, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0x32a81c89, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x260fe3ff, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x1e047854, __VMLINUX_SYMBOL_STR(warn_slowpath_fmt) },
	{ 0xe638537a, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x2d768410, __VMLINUX_SYMBOL_STR(pci_bus_write_config_dword) },
	{ 0xb20bf0c1, __VMLINUX_SYMBOL_STR(pci_bus_read_config_dword) },
	{ 0xeae3dfd6, __VMLINUX_SYMBOL_STR(__const_udelay) },
	{ 0xefd0a108, __VMLINUX_SYMBOL_STR(i2c_del_adapter) },
	{ 0x94218140, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x86b8d7be, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=dimm-bus";

MODULE_ALIAS("pci:v00008086d00003CA8sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v00008086d00006FA8sv*sd*bc*sc*i*");

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
	{ 0x909fe633, __VMLINUX_SYMBOL_STR(pmbus_do_remove) },
	{ 0x8308dcad, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x851c3bfc, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xa876fc0, __VMLINUX_SYMBOL_STR(pmbus_read_word_data) },
	{ 0x940f4a88, __VMLINUX_SYMBOL_STR(pmbus_do_probe) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=pmbus_core";

MODULE_ALIAS("i2c:dps200");

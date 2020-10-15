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
	{ 0xb31ef47d, __VMLINUX_SYMBOL_STR(param_ops_ushort) },
	{ 0xf2e07ed1, __VMLINUX_SYMBOL_STR(kobject_put) },
	{ 0xd279ba63, __VMLINUX_SYMBOL_STR(platform_device_unregister) },
	{ 0xa0433898, __VMLINUX_SYMBOL_STR(platform_driver_unregister) },
	{ 0xa04ee664, __VMLINUX_SYMBOL_STR(platform_device_put) },
	{ 0x1f4b1f17, __VMLINUX_SYMBOL_STR(platform_device_add) },
	{ 0x3f4d49bd, __VMLINUX_SYMBOL_STR(platform_device_add_data) },
	{ 0x5e068353, __VMLINUX_SYMBOL_STR(platform_device_alloc) },
	{ 0x8a2b6c2c, __VMLINUX_SYMBOL_STR(__platform_driver_register) },
	{ 0x8efb1ee3, __VMLINUX_SYMBOL_STR(__dynamic_pr_debug) },
	{ 0x42c8de35, __VMLINUX_SYMBOL_STR(ioremap_nocache) },
	{ 0xc364ae22, __VMLINUX_SYMBOL_STR(iomem_resource) },
	{ 0xae59a99, __VMLINUX_SYMBOL_STR(sysfs_create_group) },
	{ 0x5e25804, __VMLINUX_SYMBOL_STR(__request_region) },
	{ 0x400390fb, __VMLINUX_SYMBOL_STR(acpi_check_resource_conflict) },
	{ 0xb20bf0c1, __VMLINUX_SYMBOL_STR(pci_bus_read_config_dword) },
	{ 0x522106f7, __VMLINUX_SYMBOL_STR(pci_get_device) },
	{ 0x95bd6e26, __VMLINUX_SYMBOL_STR(acpi_install_sci_handler) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x60ea2d6, __VMLINUX_SYMBOL_STR(kstrtoull) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x5464d3f6, __VMLINUX_SYMBOL_STR(acpi_remove_sci_handler) },
	{ 0xedc03953, __VMLINUX_SYMBOL_STR(iounmap) },
	{ 0x8d15114a, __VMLINUX_SYMBOL_STR(__release_region) },
	{ 0x150ad92b, __VMLINUX_SYMBOL_STR(ioport_resource) },
	{ 0x50277c2e, __VMLINUX_SYMBOL_STR(sysfs_notify) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


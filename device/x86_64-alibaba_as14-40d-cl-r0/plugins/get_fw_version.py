#!/usr/bin/python
try:
   import imp
   import os
except ImportError, e:
    raise ImportError (str(e) + " - required module not find!")

FWMGR_PY_PATH = "/usr/share/sonic/device/x86_64-alibaba_as14-40d-cl-r0/plugins/fwmgrutil.py"
FWMGR_MODULE_NAME = "fwmgrutil"
FWMGR_CLASS_NAME = "FwMgrUtil"

def get_all_versions():
    try:
        module = imp.load_source(FWMGR_MODULE_NAME, FWMGR_PY_PATH)
    except IOError as e:
        print("load module %s failed, ERR: %s" % (FWMGR_MODULE_NAME, str(e)))
        return -1

    try:
        fwmgrutil_class = getattr(module, FWMGR_CLASS_NAME)
        fwmgrutil = fwmgrutil_class()
    except AttributeError as e:
        print("instantiate class %s failed, ERR: %s " % (FWMGR_CLASS_NAME, str(e)))
        return -2

    version_str = ""
    version_str = fwmgrutil.get_bmc_version()
    print("bmc_version: %s " % (version_str) )

    version_str = ""
    version_str = fwmgrutil.get_cpld_version()
    print("cpld_version: %s " % (version_str) )

    version_str = ""
    version_str = fwmgrutil.get_bios_version()
    print("bios_version: %s " % (version_str) )

    version_str = ""
    version_str = fwmgrutil.get_onie_version()
    print("onie_version: %s " % (version_str) )

    version_str = ""
    version_str = fwmgrutil.get_pcie_version()
    print("pcie_version: %s " % (version_str) )

    version_str = ""
    version_str = fwmgrutil.get_fpga_version()
    print("fpga_version: %s " % (version_str) )

    return 0

if __name__ == '__main__':
    get_all_versions()

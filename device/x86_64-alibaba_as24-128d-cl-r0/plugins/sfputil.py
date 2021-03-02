#!/usr/bin/env python
#
# Platform-specific SFP transceiver interface for SONiC
#

try:
    import time
    import json
    from sonic_sfp.sfputilbase import SfpUtilBase
except ImportError as e:
    raise ImportError("%s - required module not found" % str(e))


class SfpUtil(SfpUtilBase):
    """Platform-specific SfpUtil class"""

    PORT_START = 1
    PORT_END = 128
    QSFP_PORT_START = 1
    QSFP_PORT_END = 128

    EEPROM_OFFSET = 60
    PORT_INFO_PATH = '/sys/class/th4_fpga'

    _port_name = ""
    _port_to_eeprom_mapping = {}
    _port_to_i2cbus_mapping = {}

    @property
    def port_start(self):
        return self.PORT_START

    @property
    def port_end(self):
        return self.PORT_END

    @property
    def qsfp_ports(self):
        return range(self.QSFP_PORT_START, self.QSFP_PORT_END + 1)

    @property
    def port_to_eeprom_mapping(self):
        return self._port_to_eeprom_mapping

    @property
    def port_to_i2cbus_mapping(self):
        return self._port_to_i2cbus_mapping

    def get_port_name(self, port_num):
        if port_num in self.qsfp_ports:
            self._port_name = "QSFP" + str(port_num - self.QSFP_PORT_START + 1)
        else:
            self._port_name = "SFP" + str(port_num)
        return self._port_name

    def get_eeprom_dom_raw(self, port_num):
        if port_num in self.qsfp_ports:
            # QSFP DOM EEPROM is also at addr 0x50 and thus also stored in eeprom_ifraw
            return None
        else:
            # Read dom eeprom at addr 0x51
            return self._read_eeprom_devid(port_num, self.DOM_EEPROM_ADDR, 256)

    def __init__(self):
        # Override port_to_eeprom_mapping for class initialization
        eeprom_path = '/sys/bus/i2c/devices/i2c-{0}/{0}-0050/eeprom'

        for x in range(self.PORT_START, self.PORT_END+1):
            self.port_to_i2cbus_mapping[x] = (x + self.EEPROM_OFFSET)
            self.port_to_eeprom_mapping[x] = eeprom_path.format(
                x + self.EEPROM_OFFSET)
        SfpUtilBase.__init__(self)

    def _do_write_file(self, file_handle, offset, value):
        file_handle.seek(offset)
        file_handle.write(hex(value))
        file_handle.close()

    def get_presence(self, port_num):

        # Check for invalid port_num
        if port_num not in range(self.port_start, self.port_end + 1):
            return False

        # Get path for access port presence status
        port_name = self.get_port_name(port_num)
        sysfs_filename = "qsfp_modprs" if port_num in self.qsfp_ports else "sfp_modabs"
        reg_path = "/".join([self.PORT_INFO_PATH, port_name, sysfs_filename])

        # Read status
        try:
            reg_file = open(reg_path)
            content = reg_file.readline().rstrip()
            reg_value = int(content)
        except IOError as e:
            print "Error: unable to open file: %s" % str(e)
            return False

        # Module present is active low
        if reg_value == 0:
            return True

        return False

    def get_low_power_mode(self, port_num):
        return NotImplementedError

    def set_low_power_mode(self, port_num, lpmode):
        # Check for invalid QSFP port_num
        if port_num not in self.qsfp_ports:
            return False

        try:
            port_name = self.get_port_name(port_num)
            reg_file = open(
                "/".join([self.PORT_INFO_PATH, port_name, "qsfp_lpmode"]), "r+")
        except IOError as e:
            print "Error: unable to open file: %s" % str(e)
            return False

        content = hex(lpmode)

        reg_file.seek(0)
        reg_file.write(content)
        reg_file.close()

        return True

    def reset(self, port_num):
        # Check for invalid QSFP port_num
        if port_num not in self.qsfp_ports:
            return False

        try:
            port_name = self.get_port_name(port_num)
            reg_file = open(
                "/".join([self.PORT_INFO_PATH, port_name, "qsfp_reset"]), "w")
        except IOError as e:
            print "Error: unable to open file: %s" % str(e)
            return False

        # Convert our register value back to a hex string and write back
        reg_file.seek(0)
        reg_file.write(hex(0))
        reg_file.close()

        # Sleep 1 second to allow it to settle
        time.sleep(1)

        # Flip the bit back high and write back to the register to take port out of reset
        try:
            reg_file = open(
                "/".join([self.PORT_INFO_PATH, port_name, "qsfp_reset"]), "w")
        except IOError as e:
            print "Error: unable to open file: %s" % str(e)
            return False

        reg_file.seek(0)
        reg_file.write(hex(1))
        reg_file.close()

        return True

    def get_transceiver_change_event(self, timeout=0):
        """
        TBD
        """
        return NotImplementedError

    def tx_disable(self, port_num, disable):
        """
        @param port_num index of physical port
        @param disable, True  -- disable port tx signal
                        False -- enable port tx signal
        @return True when operation success, False on failure.
        """
        TX_DISABLE_BYTE_OFFSET = 86
        if port_num not in range(self.port_start, self.port_end + 1) or type(disable) != bool:
            return False

        # QSFP, set eeprom to disable tx
        if port_num in self.qsfp_ports:
            presence = self.get_presence(port_num)
            if not presence:
                return True

            disable = b'\x0f' if disable else b'\x00'
            # open eeprom
            try:
                with open(self.port_to_eeprom_mapping[port_num], mode="wb", buffering=0) as sysfsfile:
                    sysfsfile.seek(TX_DISABLE_BYTE_OFFSET)
                    sysfsfile.write(bytearray(disable))
            except IOError:
                return False
            except:
                return False

        # SFP, set tx_disable pin
        else:
            try:
                disable = hex(1) if disable else hex(0)
                port_name = self.get_port_name(port_num)
                reg_file = open(
                    "/".join([self.PORT_INFO_PATH, port_name, "sfp_txdisable"]), "w")
                reg_file.write(disable)
                reg_file.close()
            except IOError as e:
                print "Error: unable to open file: %s" % str(e)
                return False

        return True

    def reset_all(self):
        result = True
        port_sysfs_path = []
        for port in range(self.port_start, self.port_end+1):
            if port not in self.qsfp_ports:
                continue

            presence = self.get_presence(port)
            if not presence:
                continue

            try:
                port_name = self.get_port_name(port)
                sysfs_path = "/".join([self.PORT_INFO_PATH,
                                       port_name, "qsfp_reset"])
                reg_file = open(sysfs_path, "w")
                port_sysfs_path.append(sysfs_path)
            except IOError as e:
                result = False
                continue

            self._do_write_file(reg_file, 0, 0)

        time.sleep(1)

        for sysfs_path in port_sysfs_path:
            try:
                reg_file = open(sysfs_path, "w")
            except IOError as e:
                result = False
                continue

            self._do_write_file(reg_file, 0, 1)

        return result

# +++ add for optical module eeprom content validation +++ #
    # load reference stand file
    def load_stand_sfp_table(self):
        try:
            with open(self.stand_json_path) as stand_sfp_jason:
                self.stand_sfp_table = json.load(stand_sfp_jason)
                return self.stand_sfp_table
        except IOError:
            return None

    def _generate_stand_port_sfp_table(self):
        self.stand_sfp_table = [0] * len(self.qsfp_ports)
        for port_num in self.qsfp_ports:
            if self.get_presence(port_num):
                print("port_num==============%s" % port_num)
                self.stand_sfp_table[port_num - 1] = self.get_eeprom_dict(port_num)
                print("Successfully generated dict of eeprom.")
                stand_sfp_thr = self.get_transceiver_dom_threshold_info_dict(port_num)
                self.stand_sfp_table[port_num - 1]['threshold'] = stand_sfp_thr
                print("Successfully get threshold data.")
                print("Done! Successfully dumped E2PROM information of port %s" % port_num)
        try:
            if not os.path.exists(self.stand_json_path):
                with open(self.stand_json_path, "w") as f:
                    json.dump(self.stand_sfp_table, f, indent=2)
        except IOError:
            return False


    def generate_stand_sfp_table(self):
        self.stand_sfp_table = self.load_stand_sfp_table()
        if self.stand_sfp_table:
            return None

        if self.osfp_ports:
            self._generate_stand_port_sfp_table()
        if self.qsfp_ports:
            self._generate_stand_port_sfp_table()

    def check_sfp_fixed(self, cur_sfp_fix_info, std_sfp_fix_info):
        log_info = ""
        if std_sfp_fix_info == cur_sfp_fix_info:
            return log_info
        elif std_sfp_fix_info.keys() == cur_sfp_fix_info.keys():
            for key in std_sfp_fix_info.keys():
                if std_sfp_fix_info[key] != cur_sfp_fix_info[key]:
                    log_info += "\tError  : error in key: %s, expect value: %s  current value:%s\n"\
                                %(key, std_sfp_fix_info[key],cur_sfp_fix_info[key])
        elif std_sfp_fix_info.keys() in cur_sfp_fix_info.keys():
            log_info += "\tError  : Missing fix key : %s in current sfp dump\n"\
                        %(std_sfp_fix_info.keys() - cur_sfp_fix_info.keys())
        elif cur_sfp_fix_info.keys() in std_sfp_fix_info.keys():
            log_info += "\tError  : Additional fix key : %s in current sfp dump\n"\
                        %(cur_sfp_fix_info.keys() - std_sfp_fix_info.keys())
        else:
            log_info += "\tError  : Unknow error in fix date check\n"
            # pdb.set_trace()
        return log_info

    def check_sfp_var_with_thr(self, item, value, std_sfp_thr_info):
        log_info = ""
        re_rule = r"-?\d+(?:\.\d+)?"
        data_valid = True

        if bool(re.search(r'\d', value)):
            value_val = float(re.findall(re_rule, value)[0])
        else:
            # print(item,value)
            log_info += "\tWarning: Invalid %slowwarning data:%s\n" %(item, value)
            return log_info

        highalarm =  std_sfp_thr_info.get(item + 'highalarm')
        if bool(re.search(r'\d', highalarm)):
            highalarm_val =  float(re.findall(re_rule, highalarm)[0])
            if value_val > highalarm_val:
                log_info += "\tAlarm  : current %s value: %s  is higher than highalarm: %s\n"\
                            %(item, value, highalarm)
        else:
            log_info += "\tWarning: Invalid %shighalarm data:%s\n" %(item, highalarm)

        lowalarm =  std_sfp_thr_info.get(item + 'lowalarm')
        if bool(re.search(r'\d', lowalarm)):
            lowalarm_val =  float(re.findall(re_rule, lowalarm)[0])
            if value_val < lowalarm_val:
                log_info += "\tAlarm  : current %s value: %s  is lower than lowalarm: %s\n"\
                            %(item, value, lowalarm)
        else:
            log_info += "\tWarning: Invalid %slowalarm data:%s\n" %(item, lowalarm)

        highwarning =  std_sfp_thr_info.get(item + 'highwarning')
        if bool(re.search(r'\d', highwarning)):
            highwarning_val =  float(re.findall(re_rule, highwarning)[0])
            if value_val > highwarning_val:
                log_info += "\tWarning: current %s value: %s  is higher than highwarning: %s\n"\
                            %(item, value, highwarning)
        else:
            log_info += "\tWarning: Invalid %shighwarning data:%s\n" %(item, highwarning)

        lowwarning =  std_sfp_thr_info.get(item + 'lowwarning')
        if bool(re.search(r'\d', lowwarning)):
            lowwarning_val =  float(re.findall(re_rule, lowwarning)[0])
            if value_val < lowwarning_val:
                log_info += "\tWarning: current %s value: %s  is lower than lowwarning: %s\n"\
                            %(item, value, highwarning)
        else:
            log_info += "\tWarning: Invalid %slowwarning data:%s\n" %(item, lowwarning)
        return log_info

    def check_sfp_vars(self, cur_sfp_var_info, std_sfp_thr_info):
        log_info = ""
        cur_sfp_var_info_channel = cur_sfp_var_info['ChannelMonitorValues']
        std_sfp_thr_info_channel = std_sfp_thr_info
        # print(cur_sfp_var_info_channel,std_sfp_thr_info_channel)
        for key,value in cur_sfp_var_info_channel.items():
            if re.match(r"RX\dPower", key):
                log_info += self.check_sfp_var_with_thr('rxpower', value, std_sfp_thr_info_channel)
            if re.match(r"TX\dPower", key):
                log_info += self.check_sfp_var_with_thr('txpower', value, std_sfp_thr_info_channel)
            if re.match(r"TX\dBias", key):
                log_info += self.check_sfp_var_with_thr('txbias', value, std_sfp_thr_info_channel)

        cur_sfp_var_info_module = cur_sfp_var_info['ModuleMonitorValues']
        std_sfp_thr_info_module = std_sfp_thr_info
        for key,value in cur_sfp_var_info_module.items():
            if key == 'Temperature':
                log_info += self.check_sfp_var_with_thr('temp', value, std_sfp_thr_info_module)
            if key == 'Vcc':
                log_info += self.check_sfp_var_with_thr('vcc', value, std_sfp_thr_info_module)

        return log_info


    def do_check_sfp(self, port_num):
        print ("port_num============================================%d",port_num)
        ELP_flag = 0
        if self.get_presence(port_num) and self.stand_sfp_table[port_num-1]:
            if self.stand_sfp_table[port_num-1]['interface']['data']['Connector'] == 'No separable connector':
                ELP_flag = 1

            cur_sfp_table = self.get_eeprom_dict(port_num)
            cur_sfp_fix_info = cur_sfp_table["interface"]['data']
            cur_sfp_fix_threshold_info = self.get_transceiver_dom_threshold_info_dict(port_num)
            # print(cur_sfp_fix_info)
            if not ELP_flag:
                cur_sfp_fix_info.update(cur_sfp_fix_threshold_info)
            # print(cur_sfp_fix_info)
            cur_sfp_var_info = cur_sfp_table["dom"]['data']
            std_sfp_fix_info = self.stand_sfp_table[port_num - 1]["interface"]['data']
            if not ELP_flag:
                std_sfp_fix_info.update(self.stand_sfp_table[port_num - 1]["threshold"])
            std_sfp_thr_info = self.stand_sfp_table[port_num - 1]["threshold"]

            fix_result = self.check_sfp_fixed(cur_sfp_fix_info, std_sfp_fix_info)
            var_result = self.check_sfp_vars(cur_sfp_var_info, std_sfp_thr_info)

            if len(fix_result) > 0:
                print('Most recently dumped info is:\n',cur_sfp_fix_info)
                print('Stored dumped info is:\n',std_sfp_thr_info)
                return fix_result + var_result + "\tSUMMARY: SFP EEPROM Test Failed!"
            return var_result + "\tSUMMARY: SFP EEPROM Test Pass!"
        elif not self.stand_sfp_table[port_num-1]:
             return "invalide stand_sfp.json file, please check it!"

# --- add for optical module eeprom content validation --- #

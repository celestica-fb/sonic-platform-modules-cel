#!/usr/bin/env python

#############################################################################
#                                                                           #
# Service to control CPU watchdog                                           #
#                                                                           #
#############################################################################

import os
import time
import logging
import requests
import argparse
import subprocess

HEARTBEAT_TIME = 20
WDT_TIMEOUT = 60
HOSTNAME = "240.1.1.1"
WDT_URL = "http://240.1.1.1:8080/api/sys/watchdog"
BMC_WDT_LOG = '/var/log/bmc_feed_watchdog.log'
logging.basicConfig(filename=BMC_WDT_LOG,
                    filemode='a',
                    format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                    datefmt='%b %d %H:%M:%S',
                    level=logging.INFO)


def set_wdt_timeout(timeout):
    data = dict()
    data["wdt"] = str(timeout)
    status_code = -1
    message = None
    try:
        res = requests.post(WDT_URL, json=data, timeout=5)
        status_code = res.status_code
        message = res.json().get('result')
    except:
        message = "Unable set watchdog timeout"

    return status_code, message


def ping():
    try:
        response = subprocess.check_output(
            ['ping', '-c', '3', HOSTNAME],
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
    except subprocess.CalledProcessError:
        response = None
    return response != None


def start():
    logging.info("Started CPU watchdog")
    error_flag = 1
    status_code = -1
    while True:
        status_code, message = set_wdt_timeout(WDT_TIMEOUT)

        # Error checking
        if status_code == 200 and message != 'success':
            logging.error(message)
            error_flag = 1
        elif status_code != 200 and not ping():
            logging.error("Unable to connect to BMC")
            error_flag = 1
        elif status_code != 200 and ping():
            if not error_flag:
                logging.error(message)
            time.sleep(1)
            error_flag = 1
            continue

        # Pass error
        if error_flag and status_code == 200 and message == 'success':
            error_flag = 0
            logging.info("BMC connection successful")

        time.sleep(HEARTBEAT_TIME)


def stop():
    logging.info("Stopping CPU watchdog")
    status_code = -1
    while status_code != 200:
        status_code, message = set_wdt_timeout(0)
        if status_code == 200 and message != 'success':
            logging.error(message)
        elif status_code != 200 and not ping():
            logging.error("Unable to connect to BMC")
        elif ping():
            time.sleep(1)
            continue

    logging.info("Stopped CPU watchdog")


def main():
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('option', choices=["start", "stop"])
    args = parser.parse_args()
    if args.option == "start":
        start()
    stop()


if __name__ == "__main__":
    main()

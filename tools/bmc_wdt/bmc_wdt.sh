#!/bin/sh
#
# watchdog     Script to control bmc watchdog

# start bmcwdt
start(){
       curl -m 10 -d '{"wdt":"60"}' http://240.1.1.1:8080/api/sys/watchdog >/dev/null 2>&1
}

# stop bmcwdt
stop(){
       curl -m 10 -d '{"wdt":"0"}' http://240.1.1.1:8080/api/sys/watchdog >/dev/null 2>&1
}

case "$1" in
       start)
               start
               ;;
       stop)
               stop
               ;;
       *)
       echo $"Usage: $0 {start|stop}"
esac

#! /bin/sh

### BEGIN INIT INFO
# Provides:          listen-for-shutdown.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
### END INIT INFO

# move to /etc/init.d/ and make executable.
# register to run on boot:
#  `sudo update-rc.d listen-for-shutdown.sh defaults`

# If you want a command to always run, put it here

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    echo "Starting listen_for_shutdown.py"
    /usr/local/bin/listen_for_shutdown.py &
    ;;
  stop)
    echo "Stopping listen_for_shutdown.py"
    pkill -f /usr/local/bin/listen_for_shutdown.py
    ;;
  *)
    echo "Usage: /etc/init.d/listen_for_shutdown.sh {start|stop}"
    exit 1
    ;;
esac

exit 0
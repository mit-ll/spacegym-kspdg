#!/bin/bash
set -e

SERVER_PORT=8800

# cleanup by unsetting firewall and iptable rules
function cleanup {
    echo $'\n\nCLEANING UP...\n'

    # disable packet filter (pf) to allow normal traffic
    echo "Disabling packet filter (pf) rules..."
    pfctl -d

}


function start_lmp_server {
    echo $'\nStarting LMP server'
    # dotnet /home/ro27711/Projects/MAST/KSP/LunaMultiplayer/LMPServer/Server.dll &
    echo $'\nTODO: install LMP and dotnet'
}

# gracefully terminate all child processes on SIGNINT or SIGTERM
# ref: https://linuxconfig.org/how-to-propagate-a-signal-to-child-processes-from-a-bash-script
trap 'trap " " SIGTERM; kill 0; wait; cleanup' SIGINT SIGTERM

# enable packet filter (pf) to block all non-local traffic
echo $'\nEnabling packet filter (pf) to block all non-local traffic'
pfctl -f ~/Projects/MAST/kspdg/scripts/pf_lmp.conf
pfctl -E

# start LMP server
start_lmp_server

echo $'\n\nRunning packet-filter protected KSP+LMP\nCTRL+C to exit and disable packet filter'

while true; do 
    sleep 1
done

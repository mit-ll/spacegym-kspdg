#!/bin/bash
set -e

SERVER_PORT=8800

# cleanup by unsetting firewall and iptable rules
function cleanup {
    echo $'\n\nCLEANING UP...\n'

    # disable firewall to allow internet connection
    echo "Disabling ufw..."
    ufw disable

    # unset iptable rules to allow normal traffic
    echo $'\nUnsetting iptable rules...'
    iptables -D INPUT -p udp -s 127.0.0.1 --dport $SERVER_PORT -j ACCEPT # accept incomping udp traffic from localhost
    iptables -D INPUT -p udp --dport $SERVER_PORT -j DROP # drop incoming udp traffic from everything else
    iptables -D OUTPUT -p udp -s 127.0.0.1 --dport $SERVER_PORT -j ACCEPT # accept outgoing udp traffic to localhost
    iptables -D OUTPUT -p udp --dport $SERVER_PORT -j DROP # drop outgoing udp traffic to everything else
}


function start_lmp_server {
    echo $'\nStarting LMP server'
    dotnet /home/ro27711/Projects/MAST/KSP/LunaMultiplayer/LMPServer/Server.dll &
}

# gracefully terminate all child processes on SIGNINT or SIGTERM
# ref: https://linuxconfig.org/how-to-propagate-a-signal-to-child-processes-from-a-bash-script
trap 'trap " " SIGTERM; kill 0; wait; cleanup' SIGINT SIGTERM

# establish universal firewall to prevent external traffic
echo $'\nEnabling ufw to block all network traffic'
ufw default deny outgoing
ufw default deny incoming
ufw enable

# specify iptable rule to further control internal traffic
echo $'\nSetting iptable rules to control traffic to server ports'
iptables -A INPUT -p udp -s 127.0.0.1 --dport $SERVER_PORT -j ACCEPT # accept incomping udp traffic from localhost
iptables -A INPUT -p udp --dport $SERVER_PORT -j DROP # drop incoming udp traffic from everything else
iptables -A OUTPUT -p udp -s 127.0.0.1 --dport $SERVER_PORT -j ACCEPT # accept outgoing udp traffic to localhost
iptables -A OUTPUT -p udp --dport $SERVER_PORT -j DROP # drop outgoing udp traffic to everything else


# start LMP server
start_lmp_server

echo $'\n\nRunning firewall-protected KSP+LMP\nCTRL+C to exit and disable firewall'

while true; do 
    sleep 1
done

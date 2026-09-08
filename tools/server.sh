#!/usr/bin/env bash
# Manage the maze hub on a remote host, over SSH.
#
# EDIT THESE for your own server:
#   HOST=you@your-server-ip-or-hostname
#   KEY=~/.ssh/your_key
#   URL=http://your-server-ip-or-hostname:8787
#
# Assumes telemetry_receiver.py runs there as a systemd service named
# "weirdrobo" (Restart=always, enabled at boot) with its working directory at
# ~/weirdrobo/tools and its log redirected to ~/weirdrobo/hub.log. Adjust the
# ssh/scp commands below if your setup differs.
#
#   ./tools/server.sh status     is it up?
#   ./tools/server.sh log        last 40 lines of hub.log
#   ./tools/server.sh restart    restart the service
#   ./tools/server.sh deploy     push local tools/ changes and restart
#   ./tools/server.sh db         run/event counts in the database
set -eu
HOST=you@your-server
KEY=~/.ssh/id_ed25519
URL=http://your-server:8787
cd "$(dirname "$0")/.."

case "${1:-status}" in
  status)
    ssh -i "$KEY" "$HOST" 'systemctl is-active weirdrobo; systemctl is-enabled weirdrobo'
    curl -s -o /dev/null -m 10 -w "public: %{http_code}  $URL/maze_runner\n" "$URL/maze_runner"
    ;;
  log)     ssh -i "$KEY" "$HOST" 'tail -40 ~/weirdrobo/hub.log' ;;
  restart) ssh -i "$KEY" "$HOST" 'sudo systemctl restart weirdrobo && sleep 2 && systemctl is-active weirdrobo' ;;
  deploy)
    scp -i "$KEY" -q tools/telemetry_receiver.py tools/maze_db.py tools/dashboard.html \
        "$HOST:~/weirdrobo/tools/"
    ssh -i "$KEY" "$HOST" 'sudo systemctl restart weirdrobo && sleep 2 && systemctl is-active weirdrobo'
    curl -s -o /dev/null -m 10 -w "deployed, public: %{http_code}\n" "$URL/maze_runner"
    ;;
  db)
    ssh -i "$KEY" "$HOST" 'python3 -c "
import os, sqlite3
c = sqlite3.connect(os.path.expanduser(\"~/weirdrobo/tools/weirdrobo.db\"))
print(\"runs:\",c.execute(\"SELECT COUNT(*) FROM runs\").fetchone()[0],
      \"events:\",c.execute(\"SELECT COUNT(*) FROM events\").fetchone()[0])"'
    ;;
  *) echo "usage: $0 {status|log|restart|deploy|db}"; exit 1 ;;
esac

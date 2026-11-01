#!/bin/sh
# Render the README diagrams (mermaid sources *.mmd) to PNG with
# mermaid-cli.  GitHub could not render these mermaid blocks, so the README
# shows the PNGs.  Needs node; PUPPETEER_CONFIG may point to a puppeteer
# config (e.g. {"browser":"firefox","executablePath":"/usr/bin/firefox"}).
cd "$(dirname "$0")" || exit 1
for f in *.mmd; do
    npx -y @mermaid-js/mermaid-cli@11.4.2 ${PUPPETEER_CONFIG:+-p "$PUPPETEER_CONFIG"} \
        -c mermaid.json -i "$f" -o "${f%.mmd}.png" -b white -s 2
done

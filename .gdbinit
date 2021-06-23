tar ext localhost:3333
file ./target/thumbv7em-none-eabihf/debug/black-pill-http
monitor rtt server start 9090 0
monitor rtt setup 0x20000d00 0x00000030 "SEGGER RTT"
monitor rtt start

define restart_rtt
monitor rtt stop
monitor rtt start
monitor rtt setup 0x20000d00 0x00000030 "SEGGER RTT"
end

define restart_rtt_release
monitor rtt stop
monitor rtt start
monitor rtt setup 0x200011ac 0x00000030 "SEGGER RTT"
end

"""Phase 0 smoke test for a Prime/Inventor Hub running Pybricks."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color
from pybricks.tools import wait


hub = PrimeHub()
hub.light.on(Color.GREEN)
print("HELLO,pybricks")
wait(1000)
hub.light.off()

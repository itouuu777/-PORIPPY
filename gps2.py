import melopero_samm8q as mp
import melopero_ubx as ubx
import time
import sys

dev = mp.SAM_M8Q()
dev.ubx_only()
dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_PRT)
dev.set_message_frequency(ubx.NAV_CLASS, ubx.NAV_PVT, 1)
dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_MSG)
dev.set_measurement_freq(500, 1)
dev.wait_for_acknowledge(ubx.CFG_CLASS, ubx.CFG_RATE)

# One Measurement every 5 seconds for 300 times
# means 5 * 300 seconds = 1500 seconds = 25 minutes
for i in range(300):
    print("Measurement {} / 300".format(i))
    try:
        info = dev.get_pvt()  # returns a dictionary containing the PVT data
        if info is not None:
            print("[{}/{}/{}] {}h:{}m:{}s".format(info['year'], info['month'], info['day'],
                                                   info['hour'], info['minute'], info['second']))
            print("Longitude : {}".format(info["longitude"]))
            print("Latitude : {}".format(info["latitude"]))
            print('\n')
            sys.stdout.flush()
    except Exception as e:
        print("Unexpected error:", e)

    time.sleep(5)

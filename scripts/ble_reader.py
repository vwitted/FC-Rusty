from asyncio import sleep
from asyncio import selector_events
from asyncio import selector_events
from asyncio import exceptions
from bleak.exc import BleakDeviceNotFoundError
import subprocess
import asyncio
import subprocess  
from bleak import BleakClient

ADDRESS = "20:6E:F1:6E:44:7E"  # FC's BLE MAC
CHAR_UUID = "0000abf2-0000-1000-8000-00805f9b34fb"              # the one you identified
ELF_PATH = "/home/phil/Documents/claude code/FC-Rusty/target/thumbv7em-none-eabihf/release/fc-firmware"

async def main():
    proc = subprocess.Popen(
        ["defmt-print", "-e", ELF_PATH, "stdin"],
        stdin=subprocess.PIPE,
        # stdout/stderr inherit, so decoded logs go straight to your terminal
    )

    try:
        client = BleakClient(ADDRESS)
    except BleakDeviceNotFoundError as e:
        print(f"FC device not found: {e}")
        await sleep(30)
        await main()  
    try:
        async with client:
            def cb(_sender, data):
                if proc.poll() is not None:
                    return   # defmt-print has exited, drop bytes
                try:
                    proc.stdin.write(data)
                    proc.stdin.flush()
                except BrokenPipeError:
                    pass     # defmt-print just exited, will be picked up next iteration
            
            await client.start_notify(CHAR_UUID, cb)
            await asyncio.Event().wait()
            
    except BleakDeviceNotFoundError:
        print(f"Device not found: {ADDRESS}") 
        await sleep(30)
        await main()

    finally:
        if proc.stdin:
            proc.stdin.close()
        proc.wait(timeout=10)
            
asyncio.run(main())
import time
from Tufts_ble import Sniff, Yell
from human import Human
import asyncio

RANGE = -60
carla = Human(infected = 0, team = 4)

async def run_human(human, sniffer):
    '''Check for zombie tags and update human status.'''
    while not human.infected:
        latest = sniffer.last
        curr_time_ms = time.time_ns() // 1_000_000  # Current time in milliseconds

        if latest and latest[1:].isdigit():
            message = int(latest[1:])
            
            # If RSSI is in range and it's a different team
            if sniffer.rssi > RANGE and message != human.team:
                # If this is the first tag from this zombie
                if human.current_time[message - 1] == 0:
                    human.current_time[message - 1] = curr_time_ms
                    human.tag[message - 1] = curr_time_ms
                    print(f'First tag from team {message}, time set: {human.current_time[message - 1]}')
                else:
                    # Check if 3 seconds have passed since first hit
                    time_diff = curr_time_ms - human.tag[message - 1]
                    if time_diff >= 3000:
                        print(f'Tag confirmed from team {message}, time diff: {time_diff}')
                        human.tagged[message - 1] += 1
                        human.current_time[message - 1] = 0  # Reset the current time
                        human.tag[message - 1] = 0  # Reset first hit time

                        # Check if the human should become infected
                        human.status()
                    else:
                        # Update the last seen time if within range, but not a full 3 seconds
                        human.current_time[message - 1] = curr_time_ms
                        print(f'Tag in progress from team {message}, {time_diff}ms elapsed.')

        # Reset the sniffer's last message flag
        sniffer.last = ''
        await asyncio.sleep(0.1)

async def run_zombie(human):
    broadcaster = Yell()
    while human.infected:
        print(f'!{human.team}')
        broadcaster.advertise(f'!{human.team}')
        await asyncio.sleep(0.1)
        broadcaster.stop_advertising()

async def main():
    human = Human(infected=0, team=4)  # Initialize a human instance
    sniffer = Sniff('!', verbose=True)
    sniffer.scan(0)  # Start scanning for messages

    # Start visual and buzz tasks for human feedback 
    asyncio.create_task(human.visual())
    asyncio.create_task(human.buzz())

    # Start human and zombie behavior loops
    while True:
        if not human.infected:
            await run_human(human, sniffer)
        else:
            await run_zombie(human)
        await asyncio.sleep(0.01)

asyncio.run(main())


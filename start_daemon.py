"""Start Reachy Mini daemon with Placo kinematics for gravity compensation."""
import asyncio
import logging

logging.basicConfig(level=logging.INFO)

async def main():
    from reachy_mini.daemon.daemon import Daemon

    print("Starting daemon with PLACO kinematics engine...")
    print("Gravity compensation will be available!")

    daemon = Daemon(log_level="INFO")
    await daemon.run4ever(sim=False, headless=True, kinematics_engine="Placo")

if __name__ == "__main__":
    asyncio.run(main())

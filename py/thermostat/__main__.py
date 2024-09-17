#!/usr/bin/python3
"""
Tool to configure one Thermostat-EEM output channel to the specified settings with
the correct filter coefficients.
"""
import argparse
import asyncio
import logging
import sys
import os

from miniconf import Client, Miniconf, MQTTv5, discover, MiniconfException

if sys.platform.lower() == "win32" or os.name.lower() == "nt":
    from asyncio import set_event_loop_policy, WindowsSelectorEventLoopPolicy

    set_event_loop_policy(WindowsSelectorEventLoopPolicy())


logger = logging.getLogger(__name__)


def _main():
    parser = argparse.ArgumentParser(description="Configure Thermostat output channel.")
    parser.add_argument(
        "-v", "--verbose", action="count", default=0, help="Increase logging verbosity"
    )
    parser.add_argument(
        "--broker",
        "-b",
        type=str,
        default="mqtt",
        help="The MQTT broker to use to communicate with "
        "Thermostat-EEM (%(default)s)",
    )
    parser.add_argument(
        "--prefix",
        "-p",
        type=str,
        help="The Thermostat device prefix in MQTT, "
        "wildcards allowed as long as the match is unique "
        "(%(default)s)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=int,
        choices=[0, 1, 2, 3],
        required=True,
        help="The output channel to configure.",
    )
    parser.add_argument(
        "--min",
        type=float,
        default=-1,
        help="The channel minimum output (%(default)s A)",
    )
    parser.add_argument(
        "--max",
        type=float,
        default=1,
        help="The channel maximum output (%(default)s A)",
    )
    parser.add_argument(
        "--setpoint",
        type=float,
        default=25.0,
        help="The channel output offset (%(default)s A)",
    )
    parser.add_argument(
        "--weights",
        "-w",
        type=float,
        nargs="+",
        default=[[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
        help="Input channel weights (%(default)s)",
    )
    parser.add_argument(
        "--state",
        type=str,
        default="Off",
        help="Channel State: [On, Hold, Off] (%(default)s)",
    )
    parser.add_argument(
        "--voltage-limit",
        type=float,
        default=5.0,
        help="Channel voltage limit (%(default)s V)",
    )

    parser.add_argument(
        "--ki",
        type=float,
        default=0.0,
        help="Integrator gain (%(default)s A/Ks)",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=0.0,
        help="Proportional gain (%(default)s A/K)",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=0.0,
        help="Derivative gain (%(default)s As/K)",
    )
    parser.add_argument(
        "--li",
        type=float,
        default=1e30,
        help="Integrator limit (%(default)s A/K)",
    )
    parser.add_argument(
        "--ld",
        type=float,
        default=1e30,
        help="Derivative limit (%(default)s A/K)",
    )

    args = parser.parse_args()

    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.WARN - 10 * args.verbose,
    )

    async def run():
        async with Client(
            args.broker, protocol=MQTTv5, logger=logging.getLogger("aiomqtt-client")
        ) as client:
            if not args.prefix:
                devices = await discover(client, "dt/sinara/thermostat-eem/+")
                if len(devices) != 1:
                    raise MiniconfException(
                        "Discover", f"No unique Miniconf device (found `{devices}`)."
                    )
                prefix = list(devices.keys())[0]
                logging.info("Found device prefix: %s", prefix)
            else:
                prefix = args.prefix

            thermostat = Miniconf(client, prefix)

            # TODO: sequence!
            await thermostat.set(
                f"/output/{args.output}/state",
                "Hold",
            )
            for (
                k
            ) in "pid/min pid/max pid/setpoint pid/ki pid/kp pid/kd pid/li pid/ld voltage_limit weights state".split():
                await thermostat.set(
                    f"/output/{args.output}/{k}",
                    getattr(args, k.split("/")[-1]),
                )

    asyncio.run(run())


if __name__ == "__main__":
    _main()

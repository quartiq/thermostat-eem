#!/usr/bin/python3
"""
Adapted from Stabilizer iir_coefficients.py.

Description: Algorithms to generate biquad (second order IIR) coefficients for Thermostat-EEM.
The tool configures one Thermostat-EEM output channel to the specified settings with
the correct filter coefficients.
"""
import argparse
import asyncio
import logging

import miniconf
from stabilizer.iir_coefficients import get_filters
import thermostat

logger = logging.getLogger(__name__)

def _main():
    parser = argparse.ArgumentParser(
        description="Configure Thermostat filter parameters."
    )
    parser.add_argument(
        "-v", "--verbose", action="count", default=0, help="Increase logging verbosity"
    )
    parser.add_argument(
        "--broker",
        "-b",
        type=str,
        default="10.42.0.1",
        help="The MQTT broker to use to communicate with "
        "Thermostat-EEM (%(default)s)",
    )
    parser.add_argument(
        "--prefix",
        "-p",
        type=str,
        default="dt/sinara/thermostat-eem/+",
        help="The Thermostat device prefix in MQTT, "
        "wildcards allowed as long as the match is unique "
        "(%(default)s)",
    )
    parser.add_argument(
        "--no-discover",
        "-d",
        action="store_true",
        help="Do not discover device prefix.",
    )
    parser.add_argument(
        "--channel",
        "-c",
        type=int,
        choices=[0, 1, 2, 3],
        required=True,
        help="The output channel to configure.",
    )
    parser.add_argument(
        "--sample-period",
        "-s",
        type=float,
        default=thermostat.SAMPLE_PERIOD,
        help="Sample period in seconds (%(default)s s)",
    )
    parser.add_argument(
        "--x-offset",
        type=float,
        default=0,
        help="The channel input offset (%(default)s V)",
    )
    parser.add_argument(
        "--y-min",
        type=float,
        default=-1,
        help="The channel minimum output (%(default)s V)",
    )
    parser.add_argument(
        "--y-max",
        type=float,
        default=1,
        help="The channel maximum output (%(default)s V)",
    )
    parser.add_argument(
        "--y-offset",
        type=float,
        default=0,
        help="The channel output offset (%(default)s V)",
    )
    parser.add_argument(
        "--input-weights",
        type=float,
        default=[1, 0, 0, 0, 0, 0, 0, 0],
        help="Input channel weights (%(default)s V)",
    )
    parser.add_argument(
        "--shutdown",
        type=bool,
        default=False,
        help="Channel TEC shutdown (%(default)s V)",
    )
    parser.add_argument(
        "--hold", type=bool, default=False, help="Channel IIR hold (%(default)s V)"
    )
    parser.add_argument(
        "--voltage-limit",
        type=float,
        default=1,
        help="Channel voltage limit (%(default)s V)",
    )

    # Next, add subparsers and their arguments.
    subparsers = parser.add_subparsers(
        help="Filter-specific design parameters", dest="filter_type", required=True
    )

    filters = get_filters()

    for (filter_name, filt) in filters.items():
        subparser = subparsers.add_parser(filter_name, help=filt.help)
        for arg in filt.arguments:
            subparser.add_argument(*arg.positionals, **arg.keywords)

    args = parser.parse_args()

    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.WARN - 10 * args.verbose,
    )

    # Calculate the IIR coefficients for the filter.
    coefficients = filters[args.filter_type].coefficients(args)

    # The feed-forward gain of the IIR filter is the summation
    # of the "b" components of the filter.
    forward_gain = sum(coefficients[:3])
    if forward_gain == 0 and args.x_offset != 0:
        logger.warning("Filter has no DC gain but x_offset is non-zero")

    if args.no_discover:
        prefix = args.prefix
    else:
        devices = asyncio.run(miniconf.discover(args.broker, args.prefix))
        if not devices:
            raise ValueError("No prefixes discovered.")
        if len(devices) > 1:
            raise ValueError(
                f"Multiple prefixes discovered ({devices})."
                "Please specify a more specific --prefix"
            )
        prefix = devices[0]
        logger.info("Automatically using detected device prefix: %s", prefix)

    async def configure():
        logger.info("Connecting to broker")
        interface = await miniconf.Miniconf.create(prefix, args.broker)

        # Set the filter coefficients.
        await interface.command(
            f"output_channel/{args.channel}",
            {
                "shutdown": args.shutdown,
                "hold": args.hold,
                "voltage_limit": args.voltage_limit,
                "iir": {
                    "ba": coefficients,
                    "y_offset": args.y_offset + forward_gain * args.x_offset,
                    "y_min": args.y_min,
                    "y_max": args.y_max,
                },
                "weights": args.input_weights,
            },
        )

    asyncio.run(configure())


if __name__ == "__main__":
    _main()

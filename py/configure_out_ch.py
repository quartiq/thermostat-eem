#!/usr/bin/python3
"""
Adapted from Stabilizer iir_coefficients.py.

Description: Algorithms to generate biquad (second order IIR) coefficients for Thermostat-EEM.
The tool configures one Thermostat-EEM output channel to the specified settings with the correct filter coefficients.
"""
import argparse
import asyncio
import collections
import logging

from math import pi

import miniconf

logger = logging.getLogger(__name__)

# Sample period in seconds for all channels.
SAMPLE_PERIOD = 1/503.5  # ADC ODR 1007 split between two channels per ADC


# Generic type for containing a command-line argument.
# Use `add_argument` for simple construction.
Argument = collections.namedtuple("Argument", ["positionals", "keywords"])


def add_argument(*args, **kwargs):
    """ Convert arguments into an Argument tuple. """
    return Argument(args, kwargs)


# Represents a generic filter that can be implemented by a biquad.
#
# Fields
#     * `help`: This field specifies helpful human-readable information that
#       will be presented to users on the command line.
#     * `arguments`: A list of `Argument` objects representing available
#       command-line arguments for the filter. Use the `add_argument()`
#       function to easily parse options as they would be provided to
#       argparse.
#     * `coefficients`: A function, provided with parsed arguments, that
#       returns the IIR coefficients. See below for more information on this
#       function.
#
# # Coefficients Calculation Function
#     Description:
#       This function takes in two input arguments and returns the IIR filter
#       coefficients for Thermostat to represent the necessary filter.
#
#     Args:
#       args: The filter command-line arguments. Any filter-related arguments
#       may be accessed via their name. E.g. `args.K`.
#
#     Returns:
#       [b0, b1, b2, -a1, -a2] IIR coefficients to be programmed into a
#       Thermostat IIR filter configuration.
Filter = collections.namedtuple(
    "Filter", ["help", "arguments", "coefficients"])


def get_filters():
    """ Get a dictionary of all available filters.

    Note:
        Calculations coefficient largely taken using the derivations in
        page 9 of https://arxiv.org/pdf/1508.06319.pdf

        PII/PID coefficient equations are taken from the PID-IIR primer
        written by Robert Jördens at https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw
    """
    return {
        "lowpass": Filter(help="Gain-limited low-pass filter",
                          arguments=[
                              add_argument("--f0", required=True, type=float,
                                           help="Corner frequency (Hz)"),
                              add_argument("--K", required=True, type=float,
                                           help="Lowpass filter gain"),
                          ],
                          coefficients=lowpass_coefficients),
        "highpass": Filter(help="Gain-limited high-pass filter",
                           arguments=[
                               add_argument("--f0", required=True, type=float,
                                            help="Corner frequency (Hz)"),
                               add_argument("--K", required=True, type=float,
                                            help="Highpass filter gain"),
                           ],
                           coefficients=highpass_coefficients),
        "allpass": Filter(help="Gain-limited all-pass filter",
                          arguments=[
                              add_argument("--f0", required=True, type=float,
                                           help="Corner frequency (Hz)"),
                              add_argument("--K", required=True, type=float,
                                           help="Allpass filter gain"),
                          ],
                          coefficients=allpass_coefficients),
        "notch": Filter(help="Notch filter",
                        arguments=[
                            add_argument("--f0", required=True, type=float,
                                         help="Corner frequency (Hz)"),
                            add_argument("--Q", required=True, type=float,
                                         help="Filter quality factor"),
                            add_argument("--K", required=True, type=float,
                                         help="Filter gain"),
                        ],
                        coefficients=notch_coefficients),
        "pid": Filter(help="PID controller. Gains at 1 Hz and often negative.",
                      arguments=[
                          add_argument("--Kii", default=0, type=float,
                                       help="Double Integrator (I^2) gain"),
                          add_argument("--Kii_limit", default=float('inf'), type=float,
                                       help="Integral gain limit"),
                          add_argument("--Ki", default=0, type=float,
                                       help="Integrator (I) gain"),
                          add_argument("--Ki_limit", default=float('inf'), type=float,
                                       help="Integral gain limit"),
                          add_argument("--Kp", default=0, type=float,
                                       help="Proportional (P) gain"),
                          add_argument("--Kd", default=0, type=float,
                                       help="Derivative (D) gain"),
                          add_argument("--Kd_limit", default=float('inf'), type=float,
                                       help="Derivative gain limit"),
                          add_argument("--Kdd", default=0, type=float,
                                       help="Double Derivative (D^2) gain"),
                          add_argument("--Kdd_limit", default=float('inf'), type=float,
                                       help="Derivative gain limit"),
                      ],
                      coefficients=pid_coefficients),
    }


def lowpass_coefficients(args):
    """Calculate low-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = args.K * f0_bar / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def highpass_coefficients(args):
    """Calculate high-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = - args.K / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def allpass_coefficients(args):
    """Calculate all-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)

    b0 = args.K * (1 - f0_bar) / (1 + f0_bar)
    b1 = - args.K

    return [b0, b1, 0, a1, 0]


def notch_coefficients(args):
    """Calculate notch IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    denominator = (1 + f0_bar / args.Q + f0_bar ** 2)

    a1 = 2 * (1 - f0_bar ** 2) / denominator
    a2 = - (1 - f0_bar / args.Q + f0_bar ** 2) / denominator
    b0 = args.K * (1 + f0_bar ** 2) / denominator
    b1 = - (2 * args.K * (1 - f0_bar ** 2)) / denominator
    b2 = args.K * (1 + f0_bar ** 2) / denominator

    return [b0, b1, b2, a1, a2]


def pid_coefficients(args):
    """Calculate PID IIR filter coefficients."""

    # Determine filter order
    if args.Kii != 0:
        assert (args.Kdd, args.Kd) == (0, 0), \
            "IIR filters I^2 and D or D^2 gain are unsupported"
        order = 2
    elif args.Ki != 0:
        assert args.Kdd == 0, \
            "IIR filters with I and D^2 gain are unsupported"
        order = 1
    else:
        order = 0

    kernels = [
        [1, 0, 0],
        [1, -1, 0],
        [1, -2, 1]
    ]

    gains = [args.Kii, args.Ki, args.Kp, args.Kd, args.Kdd]
    limits = [args.Kii/args.Kii_limit, args.Ki/args.Ki_limit,
              1, args.Kd / args.Kd_limit, args.Kdd / args.Kdd_limit]
    w = 2*pi*args.sample_period
    b = [sum(gains[2 - order + i] * w**(order - i) * kernels[i][j]
             for i in range(3)) for j in range(3)]

    a = [sum(limits[2 - order + i] * w**(order - i) * kernels[i][j]
             for i in range(3)) for j in range(3)]
    b = [i/a[0] for i in b]
    a = [i/a[0] for i in a]
    assert a[0] == 1
    return b + [-ai for ai in a[1:]]


def _main():
    parser = argparse.ArgumentParser(
        description="Configure Thermostat filter parameters.")
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='Increase logging verbosity')
    parser.add_argument("--broker", "-b", type=str, default="10.42.0.1",
                        help="The MQTT broker to use to communicate with "
                        "Thermostat-EEM (%(default)s)")
    parser.add_argument("--prefix", "-p", type=str,
                        default="dt/sinara/thermostat-eem/+",
                        help="The Thermostat device prefix in MQTT, "
                        "wildcards allowed as long as the match is unique "
                        "(%(default)s)")
    parser.add_argument("--no-discover", "-d", action="store_true",
                        help="Do not discover device prefix.")

    parser.add_argument("--channel", "-c", type=int, choices=[0, 1, 2, 3],
                        required=True, help="The output channel to configure.")
    parser.add_argument("--sample-period", type=float,
                        default=SAMPLE_PERIOD,
                        help="Sample period in seconds (%(default)s s)")

    parser.add_argument("--x-offset", type=float, default=0,
                        help="The channel input offset (%(default)s V)")
    parser.add_argument("--y-min", type=float,
                        default=-1,
                        help="The channel minimum output (%(default)s V)")
    parser.add_argument("--y-max", type=float,
                        default=1,
                        help="The channel maximum output (%(default)s V)")
    parser.add_argument("--y-offset", type=float, default=0,
                        help="The channel output offset (%(default)s V)")
    parser.add_argument("--input-weights", type=float, default=[1, 0, 0, 0, 0, 0, 0, 0],
                        help="Input channel weights (%(default)s V)")
    parser.add_argument("--shutdown", type=bool, default=False,
                        help="Channel TEC shutdown (%(default)s V)")
    parser.add_argument("--hold", type=bool, default=False,
                        help="Channel IIR hold (%(default)s V)")
    parser.add_argument("--voltage-limit", type=float, default=1,
                        help="Channel voltage limit (%(default)s V)")

    # Next, add subparsers and their arguments.
    subparsers = parser.add_subparsers(
        help="Filter-specific design parameters", dest="filter_type",
        required=True)

    filters = get_filters()

    for (filter_name, filt) in filters.items():
        subparser = subparsers.add_parser(filter_name, help=filt.help)
        for arg in filt.arguments:
            subparser.add_argument(*arg.positionals, **arg.keywords)

    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        level=logging.WARN - 10*args.verbose)

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
            raise ValueError(f"Multiple prefixes discovered ({devices})."
                             "Please specify a more specific --prefix")
        prefix = devices[0]
        logger.info("Automatically using detected device prefix: %s", prefix)

    async def configure():
        logger.info("Connecting to broker")
        interface = await miniconf.Miniconf.create(prefix, args.broker)

        # Set the filter coefficients.
        # Note: In the future, we will need to Handle higher-order cascades.
        await interface.command(f"output_channel/{args.channel}", {
            "shutdown": args.shutdown,
            "hold": args.hold,
            "voltage_limit": args.voltage_limit,
            "iir": {"ba": coefficients,
                    "y_offset": args.y_offset + forward_gain * args.x_offset,
                    "y_min": args.y_min,
                    "y_max": args.y_max},
            "weights": args.input_weights
        })

    asyncio.run(configure())


if __name__ == "__main__":
    _main()

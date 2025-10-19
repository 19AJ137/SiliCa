# Adapted from https://github.com/zaq2989/FelicaTools
# Refactored by GitHub Copilot

from typing import Callable, Optional
from nfc.clf import ContactlessFrontend, RemoteTarget, TimeoutError
import argparse
import sys

HELP_DEFAULT = "(default: %(default)s)"
POLLING_TARGET = "212F"


def add_base_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-t",
        "--timeout",
        metavar="",
        type=float,
        default=1.0,
        help=f"exchange timeout [s] {HELP_DEFAULT}",
    )
    parser.add_argument(
        "--device",
        metavar="PATH",
        default="usb",
        help=f"local device search path {HELP_DEFAULT}",
    )


def make_exchange(clf: ContactlessFrontend, timeout_s: float) -> Callable[[Optional[bytes]], bytes]:
    """
    Return an exchange function bound to the given ContactlessFrontend and timeout.
    The exchange function accepts None or bytes. If bytes is provided, a 1-byte
    length prefix (len+1) is added as required by the device protocol.
    """
    def exchange(data: Optional[bytes]) -> bytes:
        if data is None:
            payload = None
        else:
            payload = (len(data) + 1).to_bytes(1, "big") + data
        return clf.exchange(payload, timeout_s)[1:]
    return exchange


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Communicate directly with FeliCa")
    add_base_arguments(parser)
    parser.add_argument(
        "-s",
        "--system-code",
        metavar="",
        default="FFFF",
        help=f"polling system code {HELP_DEFAULT}",
    )
    return parser.parse_args(argv)


def run_command(exchange: Callable[[Optional[bytes]], bytes], system_code: str) -> None:
    """
    Interactive loop to send raw commands (hex) to the card via `exchange`.
    Typing an empty line repeats prompt; typing a hex string is sent. Use '[idm]'
    placeholder to substitute the last seen IDm value.
    """
    assert len(system_code) == 4, "system_code must be 4 hex characters"
    idm: str = ""
    try:
        while True:
            try:
                # show default polling command on first prompt
                print("<< # ", end="")
                if idm == "":
                    default_cmd = f"00 {system_code} 00 00"
                    print(default_cmd)
                    raw = default_cmd
                else:
                    raw = input().strip()
                if raw == "":
                    continue

                raw = raw.replace(" ", "").lower().replace("[idm]", idm)
                payload = bytes.fromhex(raw)
                r = exchange(payload)

                if not r:
                    print(">> # <no response>", file=sys.stderr)
                    continue

                # Polling response: first byte 0x01, next 8 bytes contain IDm
                if r[0] == 0x01 and len(r) >= 9:
                    idm = r[1:9].hex()
                    print(f"\t[IDm] set to {idm}", file=sys.stderr)

                h = r.hex()
                h = "# " + h
                if idm:
                    h = h.replace(idm, " [IDm] ")
                print(">>", h)
            except (ValueError,) as e:
                print(f"Invalid input: {e}", file=sys.stderr)
            except TimeoutError:
                print("TIMEOUT", file=sys.stderr)
    except (KeyboardInterrupt, EOFError):
        # graceful exit on user interrupt
        print("", file=sys.stderr)


def main(argv=None) -> int:
    args = parse_args(argv)
    system_code = args.system_code
    timeout_s = args.timeout
    device = args.device

    try:
        clf = ContactlessFrontend(device)
    except OSError:
        print("No device", file=sys.stderr)
        return 3

    try:
        target = clf.sense(RemoteTarget(POLLING_TARGET))
        if target is None:
            print("No card", file=sys.stderr)
            return 1

        run_command(make_exchange(clf, timeout_s), system_code)
        return 0
    finally:
        try:
            clf.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

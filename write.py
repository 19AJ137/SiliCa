#!/usr/bin/env python3

from typing import Optional
import sys
import argparse
import nfc

COMMAND_WRITE = 0x08
DEFAULT_PMM = bytes.fromhex("0001FFFFFFFFFFFF")  # 8 bytes


def write_system_block(tag: nfc.tag.Tag, block_num: int, data: bytes, timeout: float = 1.0) -> None:
    """
    Write a 16-byte system block to a FeliCa tag.
    Raises nfc.tag.tt3.Type3TagCommandError on write failure.
    """
    if not (0 <= block_num <= 0xFF):
        raise ValueError("block_num must fit in one byte (0-255)")
    if len(data) != 16:
        raise ValueError("data must be exactly 16 bytes")

    # Construct command data: (number of services, service code list (2 bytes),
    # number of blocks, block list element (2 bytes service, 1 byte block))
    cmd_data = bytearray([1, 0x09, 0x00, 1, 0x80, block_num]) + data
    tag.send_cmd_recv_rsp(COMMAND_WRITE, bytes(cmd_data), timeout)


def parse_hex_parameter(s: str) -> Optional[bytes]:
    try:
        b = bytes.fromhex(s)
        return b
    except ValueError:
        return None


def build_data_for_command(command: str, param: bytes) -> Optional[tuple[int, bytes]]:
    """
    Returns (block_num, data) or None for unknown command / validation failure.
    """
    if command.isdigit():
        block = int(command)
        if not (0 <= block < 14):
            print("Block number must be between 0 and 13")
            return None
        if len(param) != 16:
            print("Data must be exactly 16 bytes for raw write")
            return None
        return block, param

    if command.startswith("idm"):
        block = 0x83
        if len(param) not in (8, 16):
            print("IDm must be 8 bytes, PMm optional 8 bytes (total 16 bytes)")
            return None
        data = param if len(param) == 16 else param + DEFAULT_PMM
        return block, data

    if command.startswith("ser"):
        block = 0x84
        if len(param) != 2:
            print("Service code must be 2 bytes")
            return None
        # original code reversed service code then padded to 16 bytes
        return block, param[::-1] + bytes(14)

    if command.startswith("sys"):
        block = 0x85
        if len(param) != 2:
            print("System code must be 2 bytes")
            return None
        return block, param + bytes(14)

    print(f"Unknown command: {command}")
    return None


def main(argv):
    parser = argparse.ArgumentParser(
        prog=argv[0],
        description="Write SiliCa system blocks (IDm/PMm, service code, system code or raw block).",
        epilog="Examples:\n  idm 0123456789ABCDEF\n  3 00112233445566778899AABBCCDDEEFF",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "command", help="command (block number or idm[_pmm], sys[tem], ser[vice])")
    parser.add_argument("parameter", help="hex parameter")
    args = parser.parse_args(argv[1:])

    param_bytes = parse_hex_parameter(args.parameter)
    if param_bytes is None:
        print("Parameter must be in hex format")
        return 1

    built = build_data_for_command(args.command.lower(), param_bytes)
    if built is None:
        return 1
    block_num, data = built

    try:
        with nfc.ContactlessFrontend("usb") as clf:
            print("Waiting for a FeliCa...")
            tag = clf.connect(
                rdwr={"targets": ["212F"], 'on-connect': lambda tag: False})
            if tag is None:
                print("No tag found")
                return 1
            print("Tag found:", tag)

            try:
                write_system_block(tag, block_num, data)
            except nfc.tag.tt3.Type3TagCommandError:
                print(
                    f"Unable to write to block {block_num:02X}h. The tag might not be a SiliCa.")
                return 1

    except Exception as exc:
        print("Error:", exc)
        return 1

    print("Write completed")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

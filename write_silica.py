import nfc

COMMAND_WRITE = 0x08
DEFAULT_PMm = bytearray.fromhex('0001FFFFFFFFFFFF')


def write_system_block(tag, block_num, data, timeout=1.0):
    assert len(data) == 16, "Data must be 16 bytes"

    cmd_data = bytearray([1, 0x09, 0x00, 1, 0x80, block_num]) + data

    tag.send_cmd_recv_rsp(COMMAND_WRITE, cmd_data, timeout)


def main(args):
    if len(args) < 3:
        print(
            f"Usage: {args[0]} <command> <parameter in hex>\n"
            "Commands:\n"
            "  <num>       Write raw 16-byte data to specified block number\n"
            "  idm[_pmm]   Set IDm (and optionally PMm) of the SiliCa\n"
            "  ser[vice]   Set service code of the SiliCa\n"
            "  sys[tem]    Set system code of the SiliCa\n"
            "Example:\n"
            f"  {args[0]} idm 0123456789ABCDEF"
        )
        return

    with nfc.ContactlessFrontend('usb') as clf:
        print("Waiting for a FeliCa...")
        tag = clf.connect(
            rdwr={'targets': ['212F'], 'on-connect': lambda tag: False})

        print("Tag found:", tag)

        command = args[1].lower()
        try:
            parameter = bytearray.fromhex(args[2])
        except ValueError:
            print("Parameter must be in hex format")
            return

        block_num = None

        if command.isdigit():
            block_num = int(command)
            if not (0 <= block_num < 14):
                print("Block number must be between 0 and 13")
                return
            if len(parameter) != 16:
                print("Data must be exactly 16 bytes for raw write")
                return
            data = parameter

        elif command.startswith("idm"):
            block_num = 0x83
            if len(parameter) not in (8, 16):
                print("IDm must be 8 bytes, PMm optional 8 bytes (total 16 bytes)")
                return
            data = parameter
            if len(data) == 8:
                data += DEFAULT_PMm

        elif command.startswith("ser"):
            block_num = 0x84
            if len(parameter) != 2:
                print("Service code must be 2 bytes")
                return
            data = parameter[::-1] + bytearray(14)

        elif command.startswith("sys"):
            block_num = 0x85
            if len(parameter) != 2:
                print("System code must be 2 bytes")
                return
            data = parameter + bytearray(14)

        else:
            print(f"Unknown command: {command}")

        try:
            write_system_block(tag, block_num, data)
        except nfc.tag.tt3.Type3TagCommandError:
            print(f"Unable to write to block {block_num:02X}h.")
            print("The tag might not be a SiliCa.")
            return

        print('Write completed')


if __name__ == "__main__":
    import sys
    main(sys.argv)

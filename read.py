import nfc

COMMAND_READ = 0x06


def read_system_block(tag, timeout=1.0) -> bytes:
    cmd_data = bytearray([1, 0xFF, 0xFF, 2, 0x80, 0xE0, 0x80, 0xE1])
    return tag.send_cmd_recv_rsp(COMMAND_READ, bytes(cmd_data), timeout)[1:]


def main(argv):
    if len(argv) != 2 or argv[1] != 'err':
        print(f'Usage: {argv[0]} err')
        return 1

    with nfc.ContactlessFrontend("usb") as clf:
        print("Waiting for a FeliCa...")
        tag = clf.connect(
            rdwr={"targets": ["212F"], 'on-connect': lambda tag: False})
        print("Tag found:", tag)

        try:
            data = read_system_block(tag)
        except nfc.tag.tt3.Type3TagCommandError:
            print("Unable to read system block. The tag might not be a SiliCa.")
            return 1

        length = data[0]

        print("Last Error Command:", data[1:length].hex(' ').upper())


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv))

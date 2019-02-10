#!/usr/bin/env python
from intelhex import IntelHex
import argparse
import random

def build_image(stack, app, passkey, out):
  passkey_index = 0
  signature = 0xdeadbeef

  ih = IntelHex(app)

  for i in range(0, len(ih)):
    if ih[i] == (signature & 0xff) and ih[i + 1] == ((signature >> 8) & 0xff) and ih[i + 2] == ((signature >> 16) & 0xff) and ih[i + 3] == ((signature >> 24) & 0xff):
      if passkey_index == 0:
        passkey_index = i
      else:
        raise RuntimeError('Found passkey signature twice?!')

  if passkey_index == 0:
    raise RuntimeError('Passkey signature was not found!')

  print('Setting new passkey %06d' % passkey)
  ih[passkey_index] = passkey & 0xff
  ih[passkey_index + 1] = (passkey >> 8) & 0xff
  ih[passkey_index + 2] = (passkey >> 16) & 0xff
  ih[passkey_index + 3] = (passkey >> 24) & 0xff

  stack = IntelHex(stack)
  ih.merge(stack)
  ih.write_hex_file(out, byte_count=32)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-s', '--stack',
    default='stack/FlashROM/home_automation_stack.hex',
    help='Path to BLE stack HEX (default: %(default)s)')
  parser.add_argument('-a', '--app',
    default='app/FlashROM/home_automation_app.hex',
    help='Path to application HEX (default: %(default)s)')
  parser.add_argument('-o', '--out',
    default='home_automation.hex',
    help='Path to output combined HEX (default: %(default)s)')
  parser.add_argument('-p', '--passkey', type=int,
    default=random.randint(0, 999999),
    help='Passkey to set for pairing (default: random value)')
  args = parser.parse_args()

  build_image(args.stack, args.app, args.passkey, args.out)

if __name__ == "__main__":
  main()

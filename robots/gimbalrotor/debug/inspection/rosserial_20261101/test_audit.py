"""Synthetic framing and FTDI packet-boundary checks; no hardware."""
import struct
import tempfile
import unittest
from pathlib import Path
from audit import frames, usb


def frame(topic, payload):
    length=struct.pack('<H',len(payload))
    body=struct.pack('<H',topic)+payload
    return b'\xff\xfe'+length+bytes([(255-sum(length))&255])+body+bytes([(255-sum(body))&255])


class DecoderTests(unittest.TestCase):
    def test_boundary_fragment_and_valid_frame(self):
        blob=b'partial'+frame(157,bytes(range(68)))
        r=frames(blob,[len(blob)],[1.0],{})
        self.assertEqual(r['valid_frames'],1)
        self.assertEqual(r['leading_fragment_bytes'],7)
        self.assertEqual(r['interior_skipped_bytes'],0)
        self.assertEqual(r['trailing_bytes'],0)

    def test_bad_body_checksum(self):
        blob=bytearray(frame(157,bytes(range(68))));blob[20]^=1
        r=frames(bytes(blob),[len(blob)],[1.0],{})
        self.assertEqual(r['valid_frames'],0)
        self.assertEqual(len(r['bad_checksum']),1)

    def test_bad_header_and_recovery(self):
        bad=bytearray(frame(157,b'abc'));bad[4]^=1
        blob=bytes(bad)+frame(153,b'xyz')
        r=frames(blob,[len(blob)],[1.0],{})
        self.assertEqual(len(r['bad_header']),1)
        self.assertEqual(r['valid_frames'],1)

    def test_ftdi_multiple_usb_packets_and_fragmented_frame(self):
        serial=frame(157,bytes(range(68)))
        # First URB: one full packet plus a short packet; second URB: remainder.
        parts=[b'\x01\x60'+serial[:62]+b'\x01\x60'+serial[62:70],
               b'\x01\x60'+serial[70:]]
        content=struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,262144,220)
        for index,part in enumerate(parts):
            h=bytearray(64);h[8:12]=bytes([ord('C'),3,129,6])
            struct.pack_into('<iII',h,28,0,len(part),len(part))
            record=bytes(h)+part
            content+=struct.pack('<IIII',1,index,len(record),len(record))+record
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'test.pcap';p.write_bytes(content)
            r=usb(p,{})
        self.assertEqual(r['streams']['in']['valid_frames'],1)
        self.assertEqual(r['streams']['in']['bad_checksum'],[])
        self.assertEqual(r['serial_bytes']['in'],len(serial))
        self.assertEqual(r['line_error_packets'],0)


if __name__=='__main__':unittest.main()

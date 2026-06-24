# This script continuously monitors specific registers of the MAX96717 device (CARKIT 88C0). Useful for debugging and verifying configurations.
#!/bin/bash

watch -n 0.5 '
echo "Register 0x390 (phy_clk_cnt):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x90 r1; echo "";
echo "Register 0x38d (phy1_pkt_cnt):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x8d r1; echo "";
echo "Register 0x38e (csi1_pkt_cnt):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x8e r1; echo "";
echo "Register 0x112 (VIDEO_TX2):"; i2ctransfer -y -f 2 w2@0x42 0x01 0x12 r1; echo "";
echo "Register 0x33c (phy1_hs_err):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x3c r1; echo ""
echo "Register 0x33e (phy2_hs_err):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x3e r1; echo "";
echo "";
echo "Register 0x330 (bit6=cont. clk):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x30 r1; echo "";
echo "Register 0x331 (bit6=enable deskew):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x31 r1; echo "";
echo "Register 0x1F  (INTR7):"; i2ctransfer -y -f 2 w2@0x42 0x00 0x1F r1; echo "";
echo "Register 0x343 (MIPI_RX19):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x43 r1; echo "";
echo "Register 0x344 (MIPI_RX20):"; i2ctransfer -y -f 2 w2@0x42 0x03 0x44 r1; echo "";
echo "Register 0x24F (VTX1):"; i2ctransfer -y -f 2 w2@0x42 0x02 0x4F r1; echo "";
'

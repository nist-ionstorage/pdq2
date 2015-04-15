#!/bin/sh

wget https://s3.amazonaws.com/nist-artiq/xilinx_ise_14.7_s3_s6.tar.gz.gpg
echo "$secret" | gpg --passphrase-fd 0 xilinx_ise_14.7_s3_s6.tar.gz.gpg
sudo tar -C / -xzf xilinx_ise_14.7_s3_s6.tar.gz
wget https://s3.amazonaws.com/nist-artiq/xilinx_webpack.lic.gpg
echo "$secret" | gpg --passphrase-fd 0 xilinx_webpack.lic.gpg
mkdir ~/.Xilinx
mv xilinx_webpack.lic ~/.Xilinx/Xilinx.lic

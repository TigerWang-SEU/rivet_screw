#!/bin/bash

LIB_DIR='/usr/local/lib/'
INCLUDE_DIR='/usr/local/include/'

MESCAN='/include/libmescan/'
MESCAN_H0='LLTDataTypes.h'
MESCAN_H1='mescan.h'
MESCAN_H2='mescan_adv.h'
MESCAN_H3='mescan_basic.h'
LLT='/include/libllt/'
LLT_H='llt.h'
SO_dir='/lib/x86_64/'
MESCAN_SO='libmescan.so.0.2.0'
LLT_SO='libllt.so.0.2.0'
DEVICE='/misc/device_properties.dat'
DIR_ERROR="You have to run ./install.sh \"/path/to/C++ SDK (Linux)/\""

if ! [ -d "$1$MESCAN" ]
then
    echo "Could not find $MESCAN folder in $1."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -d "$1$LLT" ]
then
    echo "Could not find $LLT folder in $1."
    echo $DIR_ERROR
    exit 1
fi


if ! [ -s "$1$MESCAN$MESCAN_H0" ]
then
    echo "Could not find $MESCAN_H0 in $1$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$MESCAN$MESCAN_H1" ]
then
    echo "Could not find $MESCAN_H1 in $1$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$MESCAN$MESCAN_H2" ]
then
    echo "Could not find $MESCAN_H3 in $1$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$MESCAN$MESCAN_H3" ]
then
    echo "Could not find $MESCAN_H3 in $1$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$SO_dir$MESCAN_SO" ]
then
    echo "Could not find $MESCAN_SO in $1$MESCAN."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$LLT$LLT_H" ]
then
    echo "Could not find $LLT_H in $1$LLT."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$SO_dir$LLT_SO" ]
then
    echo "Could not find $LLT_SO in $1."
    echo $DIR_ERROR
    exit 1
fi

if ! [ -s "$1$DEVICE" ]
then
    echo "Could not find $DEVICE in $1."
    echo $DIR_ERROR
    exit 1
fi

if [ -d "${INCLUDE_DIR}/libmescan/" ]
then
    echo "remove Dir ${INCLUDE_DIR}/libmescan/."
    sudo rm -rf "${INCLUDE_DIR}/libmescan/"
fi
sudo mkdir -p -- "${INCLUDE_DIR}/libmescan/"
if [ -d "${INCLUDE_DIR}/libllt/" ]
then
    echo "remove Dir ${INCLUDE_DIR}/libllt/."
    sudo rm -rf "${INCLUDE_DIR}/libllt/"
fi
sudo mkdir -p -- "${INCLUDE_DIR}/libllt/"

echo "Copying libs to $LIB_DIR."
sudo cp -- "$1$SO_dir$MESCAN_SO" "$LIB_DIR"
sudo cp -- "$1$SO_dir$LLT_SO" "$LIB_DIR"
sudo chmod 755 "$LIB_DIR$LLT_SO"
sudo chmod 755 "$LIB_DIR$MESCAN_SO"

echo "running ldconfig to create symlinks and cache."
sudo ldconfig
if [ -e "${LIB_DIR}libllt.so" ]
then
    echo "remove link ${LIB_DIR}libllt.so."
    sudo rm -f "${LIB_DIR}libllt.so"
fi
sudo ln -s -- "$LIB_DIR$LLT_SO" "${LIB_DIR}libllt.so"
if [ -e "${LIB_DIR}libmescan.so" ]
then
    echo "remove link ${LIB_DIR}libmescan.so."
    sudo rm -f "${LIB_DIR}libmescan.so"
fi
sudo ln -s -- "$LIB_DIR$MESCAN_SO" "${LIB_DIR}libmescan.so"

echo "Copying headers to $INCLUDE_DIR."
sudo cp -- "$1$MESCAN$MESCAN_H0" "${INCLUDE_DIR}/libmescan/"
sudo cp -- "$1$MESCAN$MESCAN_H1" "${INCLUDE_DIR}/libmescan/"
sudo cp -- "$1$MESCAN$MESCAN_H2" "${INCLUDE_DIR}/libmescan/"
sudo cp -- "$1$MESCAN$MESCAN_H3" "${INCLUDE_DIR}/libmescan/"
sudo cp -- "$1$LLT$LLT_H" "${INCLUDE_DIR}/libllt/"
sudo chmod -R u+rwX,g+rX,o+rX "${INCLUDE_DIR}/libmescan/"
sudo chmod -R u+rwX,g+rX,o+rX "${INCLUDE_DIR}/libllt/"

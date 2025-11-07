#!/bin/bash
#
# Builds release versions of the Crazyflie firmware and the drone show app
# by re-packaging an official Crazyflie firmware release

set -e

SCRIPT_ROOT="$(dirname $0)"

###############################################################################

# Our firmware version number
VERSION=$(date +%Y%m%d)

# Upstream Crazyflie version on top of which this firmware is built
UPSTREAM_CF_VERSION=2025.09

# Whether to use an alternative nRF firmware, compiled locally
USE_ALTERNATIVE_NRF_FIRMWARE=

###############################################################################

CURRENT_DIR="$(pwd)"

cd "$(dirname $0)"/../../..
REPO_ROOT="$(pwd)"

APP_ROOT="${REPO_ROOT}/app"

# TMP_DIR=$(mktemp -d -t cf-release-$(date +%Y%m%dT%H%M%S)-XXXXXXXXXX)
# trap "rm -rf ${TMP_DIR}" EXIT
TMP_DIR="/tmp/cf-build"
rm -rf "${TMP_DIR}"
mkdir -p "${TMP_DIR}"

PLATFORMS="cf2 cf21bl bolt flapper"

echo "Downloading official Crazyflie release, version ${UPSTREAM_CF_VERSION}..."
for PLATFORM in ${PLATFORMS}; do
  curl -sLo "${TMP_DIR}/firmware-${PLATFORM}.zip" https://github.com/bitcraze/crazyflie-release/releases/download/${UPSTREAM_CF_VERSION}/firmware-${PLATFORM}-${UPSTREAM_CF_VERSION}.zip
done

# Variants of the firmware to build
if [ "x$VARIANTS" = x ]; then
  VARIANTS=$(find "${APP_ROOT}/conf" -name '*.cfg' -maxdepth 1 -exec basename {} .cfg \; | grep -v common | sort)
fi

# Compile radio firmware if needed
if [ "x$USE_ALTERNATIVE_NRF_FIRMWARE" = x1 ]; then
  for PLATFORM in ${PLATFORMS}; do
    echo "Compiling nRF firmware for ${PLATFORM}..."
    cd ../crazyflie2-nrf-firmware
    PLATFORM=${PLATFORM} make clean
    PLATFORM=${PLATFORM} make
    cp ${PLATFORM}_nrf.bin "${TMP_DIR}/nrf-firmware-${PLATFORM}.bin"
    PLATFORM=${PLATFORM} make clean
  done
fi

cd "${APP_ROOT}"
make clean
for VARIANT in ${VARIANTS}; do
  echo "Compiling custom firmware for ${VARIANT}..."
  rm -rf build *.bin
  ./compile ${VARIANT}

  BASE_BOARD=$(cat app-config | grep "^CONFIG_PLATFORM_" | tail -1 | cut -d '_' -f 3 | cut -d '=' -f 1 | tr -d ' ' | tr 'A-Z' 'a-z')

  ZIP_NAME="skybrush-${BASE_BOARD}_${VARIANT}_${VERSION}"
  rm -f "${TMP_DIR}"/${BASE_BOARD}-skybrush-*.bin
  cp ${BASE_BOARD}-skybrush-*.bin "${TMP_DIR}"
  make clean
  rm -rf build *.bin

  cp "${TMP_DIR}/firmware-${BASE_BOARD}.zip" "${TMP_DIR}/${ZIP_NAME}.zip"
  mv ${TMP_DIR}/${BASE_BOARD}-*.bin "${TMP_DIR}/${BASE_BOARD}-${UPSTREAM_CF_VERSION}.bin"

  (cd "${TMP_DIR}" && zip "${ZIP_NAME}.zip" "${BASE_BOARD}-${UPSTREAM_CF_VERSION}.bin")

  if [ "x$USE_ALTERNATIVE_NRF_FIRMWARE" = x1 ]; then
    cp "${TMP_DIR}/nrf-firmware-${BASE_BOARD}.bin" "${TMP_DIR}/${BASE_BOARD}_nrf-${UPSTREAM_CF_VERSION}.bin"
    (cd "${TMP_DIR}" && zip "${ZIP_NAME}.zip" "${BASE_BOARD}_nrf-${UPSTREAM_CF_VERSION}.bin")
  fi

  rm -f "${TMP_DIR}"/${BASE_BOARD}-*.bin
done

cd "${CURRENT_DIR}"
rm -rf dist/${VERSION}
for VARIANT in ${VARIANTS}; do
  mkdir -p dist/${VERSION}/${VARIANT}
  mv "${TMP_DIR}"/skybrush-*_${VARIANT}_*.zip dist/${VERSION}/${VARIANT}
done

# zip -0 -r dist/skybrush-cf2-${VERSION}.zip dist/${VERSION}

echo ""
echo "========================================================================"
echo ""

echo "Firmware built in:"
find dist/${VERSION} -name 'skybrush-*.zip'

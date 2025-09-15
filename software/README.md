Software Design
---------------

This folder provides the Zephyr-based firmware and scheduler code for Leaf-Link, including:

- Firmware for STM32WL-based sensing nodes (RAK3172 module)
- Implementation of the AsTAR++ scheduler
- Configuration examples for LoRaWAN communication

Building and Running
--------------------

Before building the sample, make sure to select the correct region in the
``prj.conf`` file.

The following commands build and flash the sample.

Building for rak3172
--------------------

:zephyr:board:`rak3172` as follows:

.. zephyr-app-commands::
   :zephyr-app: app/lorawan_otaa
   :board: rak3172
   :goals: build flash
   :west-args: --no-sysbuild
   :gen-args: -DEXTRA_CONF_FILE="overlay_sleep_rak3172.conf"

Building for rak4631
--------------------

:zephyr:board:`rak4631` as follows:

.. zephyr-app-commands::
   :zephyr-app: app/lorawan_otaa
   :board: rak4631
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DEXTRA_CONF_FILE="overlay_ble.conf;overlay_mcuboot.conf:overlay_sleep_rak4631.conf"

Building for rak11720
---------------------

:zephyr:board:`rak11720` as follows:

.. zephyr-app-commands::
   :zephyr-app: app/lorawan_otaa
   :board: rak11720
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DEXTRA_CONF_FILE="overlay_ble.conf;overlay_mcuboot.conf:overlay_sleep_rak11720.conf"



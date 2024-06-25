# STTS751

The STTS751 is a digital temperature sensor which communicates over a 2-wire
SMBus 2.0
compatible bus. The temperature is measured with a user-configurable resolution
between
9 and 12 bits. At 9 bits, the smallest step size is 0.5 °C, and at 12 bits, it
is 0.0625 °C. At the
default resolution (10 bits, 0.25 °C/LSB), the conversion time is nominally 21
milliseconds.

The open-drain EVENT output is used to indicate an alarm condition in which the
measured temperature has exceeded the user-programmed high limit or fallen below the low
limit.

When the EVENT pin is asserted, the host can respond using the SMBus Alert
Response Address (ARA) protocol to which the STTS751 will respond by sending its slave
address.

The STTS751 is a 6-pin device that supports user-configurable slave addresses.
Via the pull-up resistor on the Addr/Therm pin, one of four different slave addresses
can be specified. Two order numbers (STTS751-0 and STTS751-1) provide two different
sets of slave addresses bringing the total available to eight. Thus, up to eight
devices can share the
same 2-wire SMBus without ambiguity, thereby allowing monitoring of multiple
temperature zones in an application.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/stts751.html](https://www.st.com/en/mems-and-sensors/stts751.html)


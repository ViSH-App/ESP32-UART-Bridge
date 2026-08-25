#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak>=0.22", "pyserial>=3.5"]
# ///
"""RFC2217 server backed by the ESP32-S3-VCP-BT-Bridge BLE serial protocol.

Exposes the bridge as a network serial port that esptool and PlatformIO can
use natively, including DTR/RTS resets and baud rate changes:

    ./tools/nus2217.py --name ESP32_BRIDGE --port 4000
    esptool --port rfc2217://localhost:4000 read-flash-status
    pio ... monitor_port = rfc2217://localhost:4000

Protocol (see docs/ipad-integration.md):
    6E400002  data host -> bridge (write without response)
    6E400003  data bridge -> host (notify)
    6E400004  control lines, 1 byte: bit0 DTR, bit1 RTS (write)
    6E400005  line coding, 7 bytes: baud LE32, stop, parity, data (write)
    6E400006  serial state, 2 bytes LE, USB CDC SerialState bitmap (notify)
"""
import argparse
import asyncio
import logging
import os
import struct
import sys

import serial
from serial.rfc2217 import PortManager
from bleak import BleakClient, BleakScanner

UUID = "6E40000{}-B5A3-F393-E0A9-E50E24DCCA9E".format
CHAR_RX, CHAR_TX, CHAR_CTRL, CHAR_LINE, CHAR_STATE = (
    UUID(2), UUID(3), UUID(4), UUID(5), UUID(6))

PARITY_MAP = {
    serial.PARITY_NONE: 0, serial.PARITY_ODD: 1, serial.PARITY_EVEN: 2,
    serial.PARITY_MARK: 3, serial.PARITY_SPACE: 4}
STOPBITS_MAP = {
    serial.STOPBITS_ONE: 0, serial.STOPBITS_ONE_POINT_FIVE: 1,
    serial.STOPBITS_TWO: 2}

log = logging.getLogger("nus2217")


class VirtualSerial:
    """Duck-typed stand-in for serial.Serial, as consumed by PortManager.

    Property writes don't touch BLE directly; they snapshot the requested
    state into `pending`, which the connection handler flushes to the bridge
    in arrival order.
    """

    def __init__(self):
        self._baudrate = 115200
        self._bytesize = serial.EIGHTBITS
        self._parity = serial.PARITY_NONE
        self._stopbits = serial.STOPBITS_ONE
        self._dtr = False
        self._rts = False
        self.xonxoff = False
        self.rtscts = False
        # modem lines, updated from the bridge's serial state notifications
        self.cts = False
        self.dsr = False
        self.ri = False
        self.cd = False
        self.pending: list[tuple[str, bytes]] = []

    def _queue_ctrl(self):
        mask = (0x01 if self._dtr else 0) | (0x02 if self._rts else 0)
        self.pending.append(("ctrl", bytes([mask])))

    def _queue_line(self):
        self.pending.append(("line", struct.pack(
            "<IBBB", self._baudrate, STOPBITS_MAP[self._stopbits],
            PARITY_MAP[self._parity], self._bytesize)))

    baudrate = property(lambda s: s._baudrate)
    bytesize = property(lambda s: s._bytesize)
    parity = property(lambda s: s._parity)
    stopbits = property(lambda s: s._stopbits)
    dtr = property(lambda s: s._dtr)
    rts = property(lambda s: s._rts)

    @baudrate.setter
    def baudrate(self, v):
        self._baudrate = v
        self._queue_line()

    @bytesize.setter
    def bytesize(self, v):
        self._bytesize = v
        self._queue_line()

    @parity.setter
    def parity(self, v):
        self._parity = v
        self._queue_line()

    @stopbits.setter
    def stopbits(self, v):
        self._stopbits = v
        self._queue_line()

    @dtr.setter
    def dtr(self, v):
        self._dtr = bool(v)
        self._queue_ctrl()

    @rts.setter
    def rts(self, v):
        self._rts = bool(v)
        self._queue_ctrl()

    @property
    def break_condition(self):
        return False

    @break_condition.setter
    def break_condition(self, v):
        if v:
            log.warning("break requested, not supported by the bridge")

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass


class TelnetConnection:
    """Sync write facade over an asyncio StreamWriter, for PortManager."""

    def __init__(self, writer):
        self._writer = writer

    def write(self, data):
        self._writer.write(data)


class Bridge:
    def __init__(self, args):
        self.args = args
        self.ble: BleakClient | None = None
        self.vserial = VirtualSerial()
        self.manager: PortManager | None = None
        self.writer: asyncio.StreamWriter | None = None
        self.max_chunk = 20
        self.disconnected = asyncio.Event()
        self.write_fd: int | None = None

    # --- BLE side ---

    async def ble_connect(self):
        if self.args.device:
            dev = await BleakScanner.find_device_by_address(
                self.args.device, timeout=self.args.scan_timeout)
        else:
            dev = await BleakScanner.find_device_by_name(
                self.args.name, timeout=self.args.scan_timeout)
        if dev is None:
            raise RuntimeError("bridge not found (is it advertising?)")
        log.info("connecting to %s (%s)", dev.name, dev.address)
        self.ble = BleakClient(
            dev, disconnected_callback=lambda _: self.disconnected.set())
        await self.ble.connect()
        mtu = self._bluez_mtu() or 0
        if not mtu:
            try:  # fallback: BlueZ backend learns the MTU on request
                await self.ble._backend._acquire_mtu()
            except Exception:
                pass
            mtu = self.ble.mtu_size
        # data characteristic accepts at most 512 bytes per write
        self.max_chunk = max(20, min(mtu - 3, 512))
        await self.ble.start_notify(CHAR_TX, self.on_data)
        await self.ble.start_notify(CHAR_STATE, self.on_serial_state)
        await self.acquire_write()
        log.info("connected, write chunk %d bytes, fd path %s",
                 self.max_chunk, "yes" if self.write_fd else "no")

    async def acquire_write(self):
        """Bypass bluetoothd for data writes.

        BlueZ AcquireWrite hands over the ATT socket: each os.write()
        becomes one write-without-response with no D-Bus round trip
        (the D-Bus WriteValue detour costs 10-30ms per chunk).
        """
        try:
            from dbus_fast import BusType, Message, MessageType
            from dbus_fast.aio import MessageBus
            ch = self.ble.services.get_characteristic(CHAR_RX)
            path = ch.obj[0] if isinstance(ch.obj, tuple) else ch.path
            bus = await MessageBus(
                bus_type=BusType.SYSTEM, negotiate_unix_fd=True).connect()
            reply = await bus.call(Message(
                destination="org.bluez", path=path,
                interface="org.bluez.GattCharacteristic1",
                member="AcquireWrite", signature="a{sv}", body=[{}]))
            if reply.message_type != MessageType.METHOD_RETURN:
                raise RuntimeError(reply.body[0] if reply.body else "denied")
            fd_index, mtu = reply.body
            self.write_fd = reply.unix_fds[fd_index]
            self.max_chunk = max(20, min(mtu - 3, 512))
            log.info("AcquireWrite: mtu=%d", mtu)
        except Exception as e:
            log.info("AcquireWrite unavailable (%s), using D-Bus writes", e)

    def _bluez_mtu(self):
        """BlueZ exposes the negotiated ATT MTU as a characteristic prop."""
        try:
            ch = self.ble.services.get_characteristic(CHAR_RX)
            props = ch.obj
            if isinstance(props, tuple):
                props = props[1]
            return int(props.get("MTU", 0))
        except Exception:
            return 0

    def on_data(self, _, data: bytearray):
        payload = bytes(data)
        log.debug("ble -> net: %s", payload.hex())
        if self.manager and self.writer:
            self.writer.write(b"".join(self.manager.escape(payload)))

    def on_serial_state(self, _, data: bytearray):
        if len(data) < 2:
            return
        state = data[0] | (data[1] << 8)
        # USB CDC SerialState: bit0 bRxCarrier(DCD), bit1 bTxCarrier(DSR),
        # bit2 break, bit3 ring
        self.vserial.cd = bool(state & 0x01)
        self.vserial.dsr = bool(state & 0x02)
        self.vserial.ri = bool(state & 0x08)
        log.info("serial state: 0x%04x", state)
        if self.manager:
            self.manager.check_modem_lines()

    async def send_data(self, payload: bytes):
        if self.write_fd is not None:
            # Direct ATT socket writes; a full socket buffer blocks the
            # thread, which is exactly the flow control we want
            loop = asyncio.get_running_loop()
            for i in range(0, len(payload), self.max_chunk):
                chunk = payload[i:i + self.max_chunk]
                await loop.run_in_executor(None, os.write, self.write_fd,
                                           chunk)
            return
        # Sequential on purpose: concurrent D-Bus writes contend inside
        # BlueZ and end up slower than the kernel's own flow-control pacing
        for i in range(0, len(payload), self.max_chunk):
            await self.ble.write_gatt_char(
                CHAR_RX, payload[i:i + self.max_chunk], response=False)

    async def flush_control(self):
        pending, self.vserial.pending = self.vserial.pending, []
        for kind, value in pending:
            char = CHAR_CTRL if kind == "ctrl" else CHAR_LINE
            log.info("%s <- %s", kind, value.hex())
            await self.ble.write_gatt_char(char, value, response=True)

    # --- RFC2217/TCP side ---

    async def handle_client(self, reader, writer):
        peer = writer.get_extra_info("peername")
        if self.manager is not None:
            log.warning("rejecting second client %s", peer)
            writer.close()
            return
        log.info("client connected: %s", peer)
        self.writer = writer
        self.manager = PortManager(
            self.vserial, TelnetConnection(writer),
            logger=log.getChild("rfc2217") if self.args.verbose > 1 else None)
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                payload = b"".join(self.manager.filter(data))
                # control before data: within one packet esptool orders
                # resets ahead of the bytes that rely on them
                await self.flush_control()
                if payload:
                    log.debug("net -> ble: %s", payload.hex())
                    await self.send_data(payload)
                await writer.drain()
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            log.info("client disconnected: %s", peer)
            self.manager = None
            self.writer = None
            writer.close()

    async def run(self):
        await self.ble_connect()
        server = await asyncio.start_server(
            self.handle_client, self.args.bind, self.args.port)
        log.info("rfc2217 server on %s:%d — use e.g. "
                 "esptool --port rfc2217://localhost:%d read-flash-status",
                 self.args.bind, self.args.port, self.args.port)
        async with server:
            await self.disconnected.wait()
        raise RuntimeError("BLE connection lost")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-d", "--device", help="bridge BLE address")
    ap.add_argument("-n", "--name", default="ESP32_BRIDGE",
                    help="bridge advertised name (default: %(default)s)")
    ap.add_argument("-b", "--bind", default="127.0.0.1")
    ap.add_argument("-p", "--port", type=int, default=4000)
    ap.add_argument("--scan-timeout", type=float, default=15.0)
    ap.add_argument("-v", "--verbose", action="count", default=0)
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    try:
        asyncio.run(Bridge(args).run())
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        log.error("%s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()

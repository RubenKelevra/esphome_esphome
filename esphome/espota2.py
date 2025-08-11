from __future__ import annotations

import contextlib
import gzip
import hashlib
import io
import logging
import random
import selectors
import socket
import socket as _s
import sys
import time

from esphome.core import EsphomeError
from esphome.helpers import resolve_ip_address

RESPONSE_OK = 0x00
RESPONSE_REQUEST_AUTH = 0x01

RESPONSE_HEADER_OK = 0x40
RESPONSE_AUTH_OK = 0x41
RESPONSE_UPDATE_PREPARE_OK = 0x42
RESPONSE_BIN_MD5_OK = 0x43
RESPONSE_RECEIVE_OK = 0x44
RESPONSE_UPDATE_END_OK = 0x45
RESPONSE_SUPPORTS_COMPRESSION = 0x46
RESPONSE_CHUNK_OK = 0x47

RESPONSE_ERROR_MAGIC = 0x80
RESPONSE_ERROR_UPDATE_PREPARE = 0x81
RESPONSE_ERROR_AUTH_INVALID = 0x82
RESPONSE_ERROR_WRITING_FLASH = 0x83
RESPONSE_ERROR_UPDATE_END = 0x84
RESPONSE_ERROR_INVALID_BOOTSTRAPPING = 0x85
RESPONSE_ERROR_WRONG_CURRENT_FLASH_CONFIG = 0x86
RESPONSE_ERROR_WRONG_NEW_FLASH_CONFIG = 0x87
RESPONSE_ERROR_ESP8266_NOT_ENOUGH_SPACE = 0x88
RESPONSE_ERROR_ESP32_NOT_ENOUGH_SPACE = 0x89
RESPONSE_ERROR_NO_UPDATE_PARTITION = 0x8A
RESPONSE_ERROR_MD5_MISMATCH = 0x8B
RESPONSE_ERROR_UNKNOWN = 0xFF

OTA_VERSION_1_0 = 1
OTA_VERSION_2_0 = 2

MAGIC_BYTES = [0x6C, 0x26, 0xF7, 0x5C, 0x45]

FEATURE_SUPPORTS_COMPRESSION = 0x01

OTA_IMAGE_ACK_SIZE_OTA_V2 = 8192
OTA_IMAGE_TRANSFER_TIMEOUT = 600.0
OTA_CONNECT_TIMEOUT = 60.0

_LOGGER = logging.getLogger(__name__)


class ProgressBar:
    def __init__(self):
        self.last_progress = None

    def update(self, progress: float):
        bar_length = 60
        status = ""
        if progress >= 1:
            progress = 1
            status = "Done...\r\n"
        new_progress = int(progress * 100)
        if new_progress == self.last_progress:
            return
        self.last_progress = new_progress

        # Compute blocks from the integer percent so 0% => 0 blocks
        block = int(round(bar_length * (new_progress / 100.0)))
        text = f"\rUploading: [{'=' * block + ' ' * (bar_length - block)}] {round(progress * 100, 2):6.2f}% {status}"
        sys.stderr.write(text)
        sys.stderr.flush()

    def done(self):
        sys.stderr.write("\n")
        sys.stderr.flush()


class OTAError(EsphomeError):
    pass


def recv_decode(sock, amount, decode=True):
    data = sock.recv(amount)
    if not decode:
        return data
    return list(data)


def receive_exactly(sock, amount, msg, expect, decode=True):
    data = [] if decode else b""

    try:
        data += recv_decode(sock, 1, decode=decode)
    except OSError as err:
        raise OTAError(f"Error receiving acknowledge {msg}: {err}") from err

    try:
        check_error(data, expect)
    except OTAError as err:
        sock.close()
        raise OTAError(f"Error {msg}: {err}") from err

    while len(data) < amount:
        try:
            data += recv_decode(sock, amount - len(data), decode=decode)
        except OSError as err:
            raise OTAError(f"Error receiving {msg}: {err}") from err
    return data


def check_error(data, expect):
    if not expect:
        return
    dat = data[0]
    if dat == RESPONSE_ERROR_MAGIC:
        raise OTAError("Error: Invalid magic byte")
    if dat == RESPONSE_ERROR_UPDATE_PREPARE:
        raise OTAError(
            "Error: Couldn't prepare flash memory for update. Is the binary too big? "
            "Please try restarting the ESP."
        )
    if dat == RESPONSE_ERROR_AUTH_INVALID:
        raise OTAError("Error: Authentication invalid. Is the password correct?")
    if dat == RESPONSE_ERROR_WRITING_FLASH:
        raise OTAError(
            "Error: Wring OTA data to flash memory failed. See USB logs for more "
            "information."
        )
    if dat == RESPONSE_ERROR_UPDATE_END:
        raise OTAError(
            "Error: Finishing update failed. See the MQTT/USB logs for more "
            "information."
        )
    if dat == RESPONSE_ERROR_INVALID_BOOTSTRAPPING:
        raise OTAError(
            "Error: Please press the reset button on the ESP. A manual reset is "
            "required on the first OTA-Update after flashing via USB."
        )
    if dat == RESPONSE_ERROR_WRONG_CURRENT_FLASH_CONFIG:
        raise OTAError(
            "Error: ESP has been flashed with wrong flash size. Please choose the "
            "correct 'board' option (esp01_1m always works) and then flash over USB."
        )
    if dat == RESPONSE_ERROR_WRONG_NEW_FLASH_CONFIG:
        raise OTAError(
            "Error: ESP does not have the requested flash size (wrong board). Please "
            "choose the correct 'board' option (esp01_1m always works) and try "
            "uploading again."
        )
    if dat == RESPONSE_ERROR_ESP8266_NOT_ENOUGH_SPACE:
        raise OTAError(
            "Error: ESP does not have enough space to store OTA file. Please try "
            "flashing a minimal firmware (remove everything except ota)"
        )
    if dat == RESPONSE_ERROR_ESP32_NOT_ENOUGH_SPACE:
        raise OTAError(
            "Error: The OTA partition on the ESP is too small. ESPHome needs to resize "
            "this partition, please flash over USB."
        )
    if dat == RESPONSE_ERROR_NO_UPDATE_PARTITION:
        raise OTAError(
            "Error: The OTA partition on the ESP couldn't be found. ESPHome needs to create "
            "this partition, please flash over USB."
        )
    if dat == RESPONSE_ERROR_MD5_MISMATCH:
        raise OTAError(
            "Error: Application MD5 code mismatch. Please try again "
            "or flash over USB with a good quality cable."
        )
    if dat == RESPONSE_ERROR_UNKNOWN:
        raise OTAError("Unknown error from ESP")
    if not isinstance(expect, (list, tuple)):
        expect = [expect]
    if dat not in expect:
        raise OTAError(f"Unexpected response from ESP: 0x{data[0]:02X}")


def send_check(sock, data, msg):
    try:
        if isinstance(data, (list, tuple)):
            data = bytes(data)
        elif isinstance(data, int):
            data = bytes([data])
        elif isinstance(data, str):
            data = data.encode("utf8")

        sock.sendall(data)
    except OSError as err:
        raise OTAError(f"Error sending {msg}: {err}") from err


def perform_ota(
    sock: socket.socket, password: str, file_handle: io.IOBase, filename: str
) -> None:
    file_contents = file_handle.read()
    file_size = len(file_contents)
    _LOGGER.info("Uploading %s (%s bytes)", filename, file_size)

    log_info_inline("Asking node for OTA version...", logger=_LOGGER)
    # Enable nodelay, we need it for phase 1
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    send_check(sock, MAGIC_BYTES, "magic bytes")
    _, version = receive_exactly(sock, 2, "version", RESPONSE_OK)
    log_inline_append(_LOGGER, " OK")
    log_newline(_LOGGER)

    _LOGGER.debug("Device support OTA version: %s", version)
    supported_versions = (OTA_VERSION_1_0, OTA_VERSION_2_0)
    if version not in supported_versions:
        raise OTAError(
            f"Device uses unsupported OTA version {version}, this ESPHome supports {supported_versions}"
        )

    # Features
    log_info_inline("Asking node for features...", logger=_LOGGER)
    send_check(sock, FEATURE_SUPPORTS_COMPRESSION, "features")
    features = receive_exactly(
        sock, 1, "features", [RESPONSE_HEADER_OK, RESPONSE_SUPPORTS_COMPRESSION]
    )[0]
    log_inline_append(_LOGGER, " OK")
    log_newline(_LOGGER)

    if features == RESPONSE_SUPPORTS_COMPRESSION:
        log_info_inline("Compressing image file...", logger=_LOGGER)
        upload_contents = gzip.compress(file_contents, compresslevel=9)
        log_inline_append(_LOGGER, f" OK: Compressed to {len(upload_contents)} bytes")
        log_newline(_LOGGER)
    else:
        upload_contents = file_contents

    (auth,) = receive_exactly(
        sock, 1, "auth", [RESPONSE_REQUEST_AUTH, RESPONSE_AUTH_OK]
    )
    if auth == RESPONSE_REQUEST_AUTH:
        if not password:
            raise OTAError("ESP requests password, but no password given!")
        nonce = receive_exactly(
            sock, 32, "authentication nonce", [], decode=False
        ).decode()
        _LOGGER.debug("Auth: Nonce is %s", nonce)
        cnonce = hashlib.md5(str(random.random()).encode()).hexdigest()
        _LOGGER.debug("Auth: CNonce is %s", cnonce)

        log_info_inline("Authorize with node to do the update...", logger=_LOGGER)
        send_check(sock, cnonce, "auth cnonce")

        result_md5 = hashlib.md5()
        result_md5.update(password.encode("utf-8"))
        result_md5.update(nonce.encode())
        result_md5.update(cnonce.encode())
        result = result_md5.hexdigest()
        _LOGGER.debug("Auth: Result is %s", result)

        send_check(sock, result, "auth result")
        receive_exactly(sock, 1, "auth result", RESPONSE_AUTH_OK)
        log_inline_append(_LOGGER, " OK")
        log_newline(_LOGGER)

    # Set higher timeout during upload
    sock.settimeout(OTA_IMAGE_TRANSFER_TIMEOUT)

    upload_size = len(upload_contents)
    upload_size_encoded = [
        (upload_size >> 24) & 0xFF,
        (upload_size >> 16) & 0xFF,
        (upload_size >> 8) & 0xFF,
        (upload_size >> 0) & 0xFF,
    ]
    log_info_inline(
        "Asking node if firmware image size can be accepted...", logger=_LOGGER
    )
    send_check(sock, upload_size_encoded, "binary size")
    receive_exactly(sock, 1, "binary size", RESPONSE_UPDATE_PREPARE_OK)
    log_inline_append(_LOGGER, " OK")
    log_newline(_LOGGER)

    upload_md5 = hashlib.md5(upload_contents).hexdigest()
    _LOGGER.debug("MD5 of upload is %s", upload_md5)

    log_info_inline("Providing node with md5 checksum for firmware...", logger=_LOGGER)
    send_check(sock, upload_md5, "file checksum")
    receive_exactly(sock, 1, "file checksum", RESPONSE_BIN_MD5_OK)
    duration = 1
    log_inline_append(_LOGGER, " OK")
    log_newline(_LOGGER)

    # Disable Nagle for the image transfer.
    # Avoids delay on the last part of the image.
    # Otherwise Nagle is a no-op anyway on large transfers
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    _LOGGER.info("Starting to send firmware image to node...")

    progress = ProgressBar()

    start_time = time.perf_counter()

    try:
        if version >= OTA_VERSION_2_0:
            image_transfer_with_device_acks_nonblocking(
                sock,
                upload_contents,
                progress=progress,
            )
        else:
            # Send blocking everything in one go.
            sock.sendall(upload_contents)
    except Exception as err:
        sys.stderr.write("\n\n")
        raise OTAError(f"Error sending data: {err}") from err
    else:
        if version < OTA_VERSION_2_0:
            progress.update(1.0)
        progress.done()

    # Enable nodelay for last checks
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    duration = time.perf_counter() - start_time

    _LOGGER.info("Upload took %.2f seconds, waiting for result...", duration)

    receive_exactly(sock, 1, "receive OK", RESPONSE_RECEIVE_OK)
    receive_exactly(sock, 1, "Update end", RESPONSE_UPDATE_END_OK)
    send_check(sock, RESPONSE_OK, "end acknowledgement")

    _LOGGER.info("OTA successful")

    # Do not connect logs until it is fully on
    time.sleep(1)


def run_ota_impl_(remote_host, remote_port, password, filename):
    try:
        res = resolve_ip_address(remote_host, remote_port)
    except EsphomeError as err:
        _LOGGER.error(
            "Error resolving IP address of %s. Is it connected to WiFi?",
            remote_host,
        )
        _LOGGER.error(
            "(If this error persists, please set a static IP address: "
            "https://esphome.io/components/wifi.html#manual-ips)"
        )
        raise OTAError(err) from err

    for r in res:
        af, socktype, _, _, sa = r
        _LOGGER.info("Connecting to %s port %s...", sa[0], sa[1])
        sock = socket.socket(af, socktype)
        sock.settimeout(OTA_CONNECT_TIMEOUT)
        try:
            # Clamp TCP MSS so more packets fit into the small
            # receive window of the embedded devices.
            # This allows to keep more than ≥4 segments in flight,
            # making fast retransmit more likely and reducing
            # the amount of RTO backoffs on loss.
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_MAXSEG, 536)
        except (AttributeError, OSError) as err:
            _LOGGER.warning("Could not set TCP MSS: %s", err)

        try:
            sock.connect(sa)
        except OSError as err:
            sock.close()
            _LOGGER.error("Connecting to %s port %s failed: %s", sa[0], sa[1], err)
            continue

        _LOGGER.info("Connected to %s", sa[0])
        with open(filename, "rb") as file_handle:
            try:
                perform_ota(sock, password, file_handle, filename)
            except OTAError as err:
                _LOGGER.error(str(err))
                return 1
            finally:
                sock.close()

        return 0

    _LOGGER.error("Connection failed.")
    return 1


def run_ota(remote_host, remote_port, password, filename):
    try:
        return run_ota_impl_(remote_host, remote_port, password, filename)
    except OTAError as err:
        _LOGGER.error(err)
        return 1


def image_transfer_with_device_acks_nonblocking(
    sock,
    data: bytes,
    *,
    progress=None,
) -> None:
    """Single thread transfer function with feedback processing

    Reads the 8 kByte feedback responses by the microcontroller
    to update the progress bar asynchronously, and sending
    as much data to the buffer as it will accept.

    Globals required:
      - OTA_IMAGE_ACK_SIZE_OTA_V2 (bytes per ACK)
      - OTA_IMAGE_TRANSFER_TIMEOUT (stall timeout in seconds)
      - RESPONSE_CHUNK_OK (ack byte; int 0..255 or single-byte bytes)
    """

    # Fetch global variables and check for existence
    try:
        ack_block_size = OTA_IMAGE_ACK_SIZE_OTA_V2
    except NameError as e:
        raise ValueError(
            "Global OTA_IMAGE_ACK_SIZE_OTA_V2 is not defined; cannot compute progress"
        ) from e

    try:
        stall_timeout = float(OTA_IMAGE_TRANSFER_TIMEOUT)
    except NameError as e:
        raise ValueError(
            "Global OTA_IMAGE_TRANSFER_TIMEOUT is not defined; cannot set timeout"
        ) from e

    try:
        _ack = RESPONSE_CHUNK_OK
    except NameError as e:
        raise ValueError(
            "Global RESPONSE_CHUNK_OK is not defined; cannot recognize device ACKs"
        ) from e

    # Normalize ack byte from either int or bytes
    if isinstance(_ack, int):
        if not (0 <= _ack <= 255):
            raise ValueError("RESPONSE_CHUNK_OK int must be in range 0..255")
        ack_byte = bytes([_ack])
    elif isinstance(_ack, (bytes, bytearray)):
        if len(_ack) != 1:
            raise ValueError("RESPONSE_CHUNK_OK bytes must be exactly 1 byte long")
        ack_byte = bytes(_ack)
    else:
        raise ValueError(
            "RESPONSE_CHUNK_OK must be an int or a single-byte bytes object"
        )

    total = len(data)
    view = memoryview(data)
    sent = 0
    acked = 0

    prev_blocking = sock.getblocking()
    sock.setblocking(False)

    sel = selectors.DefaultSelector()
    sel.register(sock, selectors.EVENT_READ | selectors.EVENT_WRITE)

    last_progress = time.perf_counter()
    ack_val = ack_byte[0]

    try:
        while acked < total:
            events = sel.select(timeout=stall_timeout)
            if not events:
                raise OTAError(
                    f"Stall: no send/ack activity within {stall_timeout} seconds"
                )

            for _, mask in events:
                # Write as much as the kernel will accept right now
                if (mask & selectors.EVENT_WRITE) and sent < total:
                    try:
                        n = sock.send(view[sent:])
                    except (BlockingIOError, InterruptedError):
                        n = 0
                    if n > 0:
                        sent += n
                        last_progress = time.perf_counter()
                        if sent >= total:
                            # No more writes; only need to read acks
                            sel.modify(sock, selectors.EVENT_READ)

                # Drain only leading ACK bytes using MSG_PEEK so we don't
                # consume control bytes (0x44/0x45) that follow the last ACK.
                if mask & selectors.EVENT_READ:
                    try:
                        peek = sock.recv(4096, _s.MSG_PEEK)
                    except (BlockingIOError, InterruptedError):
                        peek = b""

                    if peek == b"":
                        if acked < total:
                            raise OTAError(
                                f"Peer closed early (acked {acked} / {total})"
                            )
                        break

                    # Count leading ACK bytes (0x47) only.
                    i = 0
                    for b in peek:
                        if b == ack_val:
                            i += 1
                        else:
                            break

                    if i > 0:
                        # Pop only the the leading ACK bytes we just counted.
                        _ = sock.recv(i)
                        acked = min(acked + i * ack_block_size, total)
                        if progress is not None:
                            with contextlib.suppress(Exception):
                                progress.update(acked / total)
                        last_progress = time.perf_counter()
                    else:
                        # First unread byte is *not* an ACK and we
                        # haven't got all ACKs for the upload yet.
                        # That's a protocol error: the device sent
                        # an control/error byte before final ACK.

                        # If acked == total we'll exit the outer
                        # loop; leavings the following control
                        # bytes in the kernel buffer for the caller.

                        if acked < total:
                            first = peek[0]
                            raise OTAError(
                                f"Unexpected byte 0x{first:02X} from device before upload completed"
                            )

            if time.perf_counter() - last_progress > stall_timeout:
                raise OTAError(
                    f"Stall: no send progress/protocol ack received within {stall_timeout} seconds"
                )
    finally:
        with contextlib.suppress(Exception):
            sel.unregister(sock)
        sock.setblocking(prev_blocking)


def _pick_stream_handlers(logger: logging.Logger):
    # Prefer this logger's handlers; fall back up the chain if it propagates.
    hs = [h for h in logger.handlers if isinstance(h, logging.StreamHandler)]
    if not hs and logger.propagate:
        cur = logger.parent
        while cur:
            hs += [h for h in cur.handlers if isinstance(h, logging.StreamHandler)]
            if hs or not cur.propagate:
                break
            cur = cur.parent
    return hs


def log_info_inline(msg: str, logger: logging.Logger) -> None:
    """Emit one INFO-style prefix + message, but **no newline**."""
    hs = _pick_stream_handlers(logger)
    if not hs:
        print(f"INFO {msg}", end="", flush=True)  # last-ditch
        return
    h = hs[0]
    # Format once using the handler's formatter so prefix matches your setup
    rec = logger.makeRecord(
        logger.name, logging.INFO, fn="", lno=0, msg=msg, args=(), exc_info=None
    )
    s = h.format(rec)
    h.acquire()
    try:
        h.stream.write(s)  # no terminator
        h.flush()
    finally:
        h.release()


def log_inline_append(logger: logging.Logger, msg: str) -> None:
    """Append raw text to the same line (no prefix, no newline)."""
    hs = _pick_stream_handlers(logger)
    if not hs:
        print(msg, end="", flush=True)
        return
    h = hs[0]
    h.acquire()
    try:
        h.stream.write(msg)
        h.flush()
    finally:
        h.release()


def log_newline(logger: logging.Logger) -> None:
    """Finish the line."""
    hs = _pick_stream_handlers(logger)
    if not hs:
        print()
        return
    h = hs[0]
    h.acquire()
    try:
        h.stream.write("\n")
        h.flush()
    finally:
        h.release()

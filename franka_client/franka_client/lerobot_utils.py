import io
import json
import logging
import pickle
import os
import time
from dataclasses import dataclass, field
from pathlib import Path

# Important: We import from our local copy of services_pb2 now
import services_pb2

TransferState = services_pb2.TransferState

CHUNK_SIZE = 2 * 1024 * 1024  # 2 MB
MAX_MESSAGE_SIZE = 4 * 1024 * 1024  # 4 MB

def bytes_buffer_size(buffer: io.BytesIO) -> int:
    buffer.seek(0, io.SEEK_END)
    result = buffer.tell()
    buffer.seek(0)
    return result

def send_bytes_in_chunks(buffer: bytes, message_class: type, log_prefix: str = "", silent: bool = True):
    bytes_buffer: io.BytesIO = io.BytesIO(buffer)
    size_in_bytes = bytes_buffer_size(bytes_buffer)

    sent_bytes = 0

    logging_method = logging.info if not silent else logging.debug

    logging_method(f"{log_prefix} Buffer size {size_in_bytes / 1024 / 1024} MB with")

    while sent_bytes < size_in_bytes:
        transfer_state = TransferState.TRANSFER_MIDDLE

        if sent_bytes + CHUNK_SIZE >= size_in_bytes:
            transfer_state = TransferState.TRANSFER_END
        elif sent_bytes == 0:
            transfer_state = TransferState.TRANSFER_BEGIN

        size_to_read = min(CHUNK_SIZE, size_in_bytes - sent_bytes)
        chunk = bytes_buffer.read(size_to_read)

        yield message_class(transfer_state=transfer_state, data=chunk)
        sent_bytes += size_to_read
        logging_method(f"{log_prefix} Sent {sent_bytes}/{size_in_bytes} bytes with state {transfer_state}")

    logging_method(f"{log_prefix} Published {sent_bytes / 1024 / 1024} MB")

def grpc_channel_options(
    max_receive_message_length: int = MAX_MESSAGE_SIZE,
    max_send_message_length: int = MAX_MESSAGE_SIZE,
    enable_retries: bool = True,
    initial_backoff: str = "0.1s",
    max_attempts: int = 5,
    backoff_multiplier: float = 2,
    max_backoff: str = "2s",
):
    service_config = {
        "methodConfig": [
            {
                "name": [{}],  # Applies to ALL methods in ALL services
                "retryPolicy": {
                    "maxAttempts": max_attempts,  # Max retries (total attempts = 5)
                    "initialBackoff": initial_backoff,  # First retry after 0.1s
                    "maxBackoff": max_backoff,  # Max wait time between retries
                    "backoffMultiplier": backoff_multiplier,  # Exponential backoff factor
                    "retryableStatusCodes": [
                        "UNAVAILABLE",
                        "DEADLINE_EXCEEDED",
                    ],  # Retries on network failures
                },
            }
        ]
    }

    service_config_json = json.dumps(service_config)

    retries_option = 1 if enable_retries else 0

    return [
        ("grpc.max_receive_message_length", max_receive_message_length),
        ("grpc.max_send_message_length", max_send_message_length),
        ("grpc.enable_retries", retries_option),
        ("grpc.service_config", service_config_json),
    ]

def init_logging(log_file=None, display_pid=False):
    logging.basicConfig(level=logging.INFO)

def get_logger(name: str, log_to_file: bool = True) -> logging.Logger:
    if log_to_file:
        os.makedirs("logs", exist_ok=True)
        log_file = Path(f"logs/{name}_{int(time.time())}.log")
    else:
        log_file = None

    init_logging(log_file=log_file, display_pid=False)
    return logging.getLogger(name)

@dataclass
class FPSTracker:
    target_fps: float
    first_timestamp: float = None
    total_obs_count: int = 0

    def calculate_fps_metrics(self, current_timestamp: float) -> dict[str, float]:
        self.total_obs_count += 1

        if self.first_timestamp is None:
            self.first_timestamp = current_timestamp

        total_duration = current_timestamp - self.first_timestamp
        avg_fps = (self.total_obs_count - 1) / total_duration if total_duration > 1e-6 else 0.0

        return {"avg_fps": avg_fps, "target_fps": self.target_fps}

    def reset(self):
        self.first_timestamp = None
        self.total_obs_count = 0

@dataclass
class RemotePolicyConfig:
    policy_type: str
    pretrained_name_or_path: str
    lerobot_features: dict
    actions_per_chunk: int
    device: str = "cpu"
    fps: float = 30.0  # robot control-loop frequency; used by server for action timestamping
    rename_map: dict[str, str] = field(default_factory=dict)

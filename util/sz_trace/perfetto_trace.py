# Copyright 2025 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

from collections import deque, defaultdict
from perfetto.protos.perfetto.trace import perfetto_trace_pb2
import re

# Aliases
TYPE_INSTANT = perfetto_trace_pb2.TrackEvent.TYPE_INSTANT
TYPE_SLICE_BEGIN = perfetto_trace_pb2.TrackEvent.TYPE_SLICE_BEGIN
TYPE_SLICE_END = perfetto_trace_pb2.TrackEvent.TYPE_SLICE_END
TYPE_COUNTER = perfetto_trace_pb2.TrackEvent.TYPE_COUNTER
LEXICOGRAPHIC = perfetto_trace_pb2.TrackDescriptor.ChildTracksOrdering.LEXICOGRAPHIC

# A random number which must be used for all events in the trace
TRUSTED_PKG_SEQ_ID = 22222


def extract_fu_details(fu_string):
    """
    Extract functional unit details from a FU string.
    
    Patterns supported:
    - Just letters: e.g., "ALU", "LSU", "FPU", "SPATZ", "CSR", "ACC", "NONE"
    - Letters + number: e.g., "ALU0", "LSU2", "SPATZ0"
    - Letters + number + dot + number: e.g., "ALU0.0", "LSU2.1", "SPATZ0.1"
    - Special internal patterns: e.g., "SPATZ_INT_0", "SPATZ_INT_1"
    
    Returns:
        tuple: (track_name, slot_id) where track_name is the base track name
               and slot_id is either None or an integer
    """
    # Define regex patterns
    # Pattern for SPATZ internal events (e.g., SPATZ_INT_0)
    spatz_int_pattern = r"^(SPATZ_INT)_(\d+)$"
    # Pattern for just letters (no numbers)
    fu_type_pattern = r"^([A-Za-z]+)$"
    # Pattern for letters + number (e.g., ALU0, SPATZ0)
    fu_id_pattern = r"^([A-Za-z]+)(\d+)$"
    # Pattern for letters + number + dot + number (e.g., ALU0.0, SPATZ0.1)
    fu_slot_pattern = r"^([A-Za-z]+)(\d+)\.(\d+)$"

    # Match SPATZ internal pattern first (most specific)
    if match := re.match(spatz_int_pattern, fu_string):
        prefix, spatz_id = match.groups()
        track_name = f"{prefix}_{spatz_id}"
        return track_name, None
    
    # Match slot pattern (letters + number + dot + number)
    elif match := re.match(fu_slot_pattern, fu_string):
        fu_type, fu_id, slot_id = match.groups()
        fu_id = int(fu_id)
        slot_id = int(slot_id)
        track_name = f'{fu_type}{fu_id}'
        return track_name, slot_id
    
    # Match ID pattern (letters + number)
    elif match := re.match(fu_id_pattern, fu_string):
        fu_type, fu_id = match.groups()
        fu_id = int(fu_id)
        track_name = f'{fu_type}{fu_id}'
        return track_name, None
    
    # Match type pattern (just letters)
    elif match := re.match(fu_type_pattern, fu_string):
        fu_type = match.group(1)
        return fu_type, None
    
    else:
        raise ValueError(f"Invalid FU string format to extract details: {fu_string}")


class PerfettoTrace():

    def __init__(self):
        self.tracks = {}
        self.events = []
        # Start from non-zero value
        self.free_uuid = 1

    def get_uuid(self, track):
        """Get UUID of a track by its name.

        Args:
            name: The name of the track.
        """
        return self.tracks[track].track_descriptor.uuid

    def add_track(self, name, parent=None, unique_name=True):
        """Add a new track to the trace.

        Args:
            name: The name of the track to add.
            parent: Optional parent track name. If None, the track is a root track.
        """
        uuid = self.free_uuid
        self.free_uuid += 1
        track = perfetto_trace_pb2.TracePacket()
        track.track_descriptor.uuid = uuid
        if parent is not None:
            track.track_descriptor.parent_uuid = self.get_uuid(parent)
        track.track_descriptor.name = name
        track.track_descriptor.child_ordering = LEXICOGRAPHIC
        if not unique_name:
            name = f'{name}-{uuid}'
        self.tracks[name] = track
        return uuid

    def add_counter_track(self, name, parent=None, unit_name=''):
        """Add a new counter track to the trace.

        Args:
            name: The name of the track to add.
            parent: Optional parent track name. If None, the track is a root track.
        """
        uuid = self.add_track(name, parent)
        self.tracks[name].track_descriptor.counter.unit_name = unit_name
        return uuid

    def add_event(self, track, type, timestamp, name=None, annotations=None):
        """Record an event in the trace.

        Args:
            track: UUID or name of the track where the event occurs.
            type: The type of event (e.g., TYPE_SLICE_BEGIN, TYPE_SLICE_END).
            timestamp: The timestamp of the event in nanoseconds.
            name: The name/description of the event.
        """
        if isinstance(track, str):
            track = self.get_uuid(track)
        event = perfetto_trace_pb2.TracePacket()
        event.timestamp = timestamp
        event.track_event.type = type
        event.track_event.track_uuid = track
        if name is not None:
            event.track_event.name = name
        if annotations is not None:
            for key, val in annotations.items():
                if val is not None:
                    annotation = event.track_event.debug_annotations.add()
                    annotation.name = key
                    if isinstance(val, int):
                        annotation.int_value = val
                    elif isinstance(val, str):
                        annotation.string_value = val
                    else:
                        raise TypeError(f"Unsupported annotation type for key '{key}': "
                                        f"{type(val).__name__}")
        event.trusted_packet_sequence_id = TRUSTED_PKG_SEQ_ID
        self.events.append(event)

    def add_counter_event(self, track, timestamp, value):
        self.add_event(track, TYPE_COUNTER, timestamp)
        self.events[-1].track_event.counter_value = value

    def to_file(self, path):
        """Write the trace to a file.

        Args:
            path: The output file path where the trace will be written.
        """
        trace = perfetto_trace_pb2.Trace()
        trace.packet.extend(list(self.tracks.values()))
        trace.packet.extend(self.events)
        with open(path, 'wb') as f:
            f.write(trace.SerializeToString())


class PerfettoInstructionTrace(PerfettoTrace):
    """Specialized class for recording instruction-level traces.

    This class extends the generic Perfetto trace class with methods
    specific to instruction tracing.
    """

    def __init__(self):
        """Initialize the instruction trace.

        Sets up the root 'Instructions' track and adds a start offset event
        for cycle count synchronization.
        """
        super().__init__()
        self.outstanding_insns = defaultdict(deque)
        self.ipc = 0
        self.ipc_time = None

        self.add_track('Instructions')
        self.add_track('NONE', 'Instructions')
        self.add_event('NONE', TYPE_INSTANT, 0, "Start offset")
        self.add_track('Metrics')
        self.add_counter_track('IPC', 'Metrics', 'insns/cycle')
        
        # Add a track for SPATZ internal events
        self.add_track('SPATZ_Internal', 'Instructions')

    def update_ipc(self, timestamp):
        if self.ipc_time is None:
            self.ipc_time = timestamp
            self.ipc = 1
        elif self.ipc_time == timestamp:
            self.ipc += 1
        else:
            # timestamp advanced: emit previous sample, start new bucket
            self.add_counter_event('IPC', self.ipc_time, self.ipc)
            self.ipc_time = timestamp
            self.ipc = 1

    def start_insn(self, fu, name, timestamp, annotations={}):
        """Record the start of an instruction execution.

        Creates a slice begin event to mark the start of instruction execution
        on the appropriate functional unit track.

        Args:
            fu: The functional unit executing the instruction.
            name: The name/mnemonic of the instruction.
            timestamp: The timestamp when the instruction starts in nanoseconds.
        """
        # Create a new track for an FU when first encountered
        fu_string, slot_id = extract_fu_details(fu)
        
        # For SPATZ internal events, use the SPATZ_Internal parent track
        if fu_string.startswith('SPATZ_INT'):
            parent = 'SPATZ_Internal'
        else:
            parent = 'Instructions'
        
        if fu_string not in self.tracks:
            self.add_track(fu_string, parent=parent)

        # Create a new track for each instruction to allow non perfectly nested events.
        # This new track has the parent set to the hierarchical track we want to use but a different
        # uuid. The name must also match that Perfetto UI merges the tracks.
        # The uuid of the "instruction track" must be unique in the whole trace.
        # See https://perfetto.dev/docs/reference/synthetic-track-event#process-scoped-async-slices
        # This link points to process-scoped async slices but the nesting work the same way for
        # custom scoped slices.
        insn_uuid = self.add_track(fu_string, parent=parent, unique_name=False)

        # We must keep track of the uuid of the event we started to end it later. We assign it to a
        # dict with a deque indexed by the hierarchical track uuid.
        self.outstanding_insns[fu_string].appendleft(insn_uuid)

        # Create slice begin event
        if slot_id is not None:
            annotations['slot_id'] = slot_id
        self.add_event(insn_uuid, TYPE_SLICE_BEGIN, timestamp, name, annotations)

        # Update IPC
        self.update_ipc(timestamp)

    def end_insn(self, fu, timestamp):
        """Record the end of an instruction execution.

        Creates a slice end event to mark the completion of instruction execution
        on the appropriate functional unit track.

        Args:
            fu: The functional unit executing the instruction.
            timestamp: The timestamp when the instruction ends in nanoseconds.
        """
        fu_string, _ = extract_fu_details(fu)
        if fu_string not in self.outstanding_insns or len(self.outstanding_insns[fu_string]) == 0:
            # This can happen for SPATZ internal events or other special cases
            # Just create a temporary track for this end event
            if fu_string.startswith('SPATZ_INT'):
                parent = 'SPATZ_Internal'
            else:
                parent = 'Instructions'
            insn_uuid = self.add_track(fu_string, parent=parent, unique_name=False)
        else:
            insn_uuid = self.outstanding_insns[fu_string].pop()
        self.add_event(insn_uuid, TYPE_SLICE_END, timestamp, None)

//! Actor capture is a bounded copy only; SPI drains at its existing 1 Hz cadence.

use core::cell::RefCell;
use critical_section::Mutex;
use nrf_embassy_firmware::sync_trace::{
    SendRecord, TraceBatch, TraceBuffer, TraceCounters, TrackerCounters,
};
use nrf_embassy_firmware::tx_failure_trace::{self, Buffer, Record};

struct Recorder {
    buffer: TraceBuffer,
    tracker: TrackerCounters,
    failures: Buffer,
}

static RECORDER: Mutex<RefCell<Recorder>> = Mutex::new(RefCell::new(Recorder {
    buffer: TraceBuffer::new(),
    failures: Buffer::new(),
    tracker: TrackerCounters {
        id_exhausted: 0,
        send_boundary_parse_errors: 0,
        counter_saturated: false,
    },
}));

#[derive(Clone, Copy)]
pub(crate) struct CapturedBatch {
    pub(crate) batch: TraceBatch,
    pub(crate) tracker: TrackerCounters,
    pub(crate) failures: tx_failure_trace::Publication,
}

#[derive(Clone, Copy)]
// Retain bounded owned storage without allocation on the firmware target.
#[allow(clippy::large_enum_variant)]
pub(crate) enum Publication {
    Batch(CapturedBatch),
    Health {
        trace: TraceCounters,
        tracker: TrackerCounters,
        failures: tx_failure_trace::Publication,
    },
}

impl Publication {
    pub(crate) fn failures(&self) -> tx_failure_trace::Publication {
        match self {
            Self::Batch(captured) => captured.failures,
            Self::Health { failures, .. } => *failures,
        }
    }
}

pub(crate) fn observe_failure(record: Record) {
    critical_section::with(|cs| RECORDER.borrow(cs).borrow_mut().failures.capture(record));
}

pub(crate) fn observe(record: Option<SendRecord>, tracker: TrackerCounters) {
    critical_section::with(|cs| {
        let mut recorder = RECORDER.borrow(cs).borrow_mut();
        recorder.tracker = tracker;
        if let Some(record) = record {
            recorder.buffer.capture(record);
        }
    });
}

pub(crate) fn drain() -> Publication {
    critical_section::with(|cs| {
        let mut recorder = RECORDER.borrow(cs).borrow_mut();
        let failures = recorder.failures.drain();
        match recorder.buffer.drain() {
            Ok(batch) => Publication::Batch(CapturedBatch {
                batch,
                tracker: recorder.tracker,
                failures,
            }),
            Err(_) => Publication::Health {
                trace: recorder.buffer.counters(),
                tracker: recorder.tracker,
                failures,
            },
        }
    })
}

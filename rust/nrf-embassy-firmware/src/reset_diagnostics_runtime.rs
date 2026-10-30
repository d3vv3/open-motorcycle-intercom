//! Binary-only retained RAM and fail-stop exception/panic handlers.

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{ExceptionFrame, exception};
use nrf_embassy_firmware::reset_diagnostics::{
    Breadcrumb, COUNTER_MAGIC, RECORD_WORDS, Snapshot, file_hash, next_boot,
};

// cortex-m-rt 0.7.5 keeps .uninit.* outside startup's .bss clearing, in NOLOAD RAM.
// Only integer words are stored: every possible retained RAM bit pattern is valid.
#[repr(C)]
struct Retained {
    breadcrumb: [u32; RECORD_WORDS],
    counter_magic: u32,
    counter: u32,
}

#[unsafe(link_section = ".uninit.reset_diagnostics")]
static mut RETAINED: Retained = Retained {
    breadcrumb: [0; RECORD_WORDS],
    counter_magic: 0,
    counter: 0,
};
static BOOT: AtomicU32 = AtomicU32::new(0);
static SNAPSHOT: critical_section::Mutex<Cell<Snapshot>> =
    critical_section::Mutex::new(Cell::new(Snapshot {
        resetreas: 0,
        boots: 0,
        breadcrumb: None,
    }));

pub fn capture_boot() {
    // embassy-nrf 0.10.0 exports reset::{read_reasons, clear_reasons} only for
    // nRF5340. nRF52840 has POWER.RESETREAS instead; the PAC accessors are safe.
    // RESETREAS is write-one-to-clear: clear precisely the captured reasons.
    let register = embassy_nrf::pac::POWER.resetreas();
    let resetreas = register.read().0;
    register.write(|value| value.0 = resetreas);
    // SAFETY: called once before tasks start. No RAM references are created;
    // aligned volatile u32 accesses accept arbitrary retained integer contents.
    // Exception handlers only write this storage and never resume execution.
    let (breadcrumb, boots) = unsafe {
        let ptr = core::ptr::addr_of_mut!(RETAINED).cast::<u32>();
        let words = core::array::from_fn(|i| ptr.add(i).read_volatile());
        let boots = next_boot(
            ptr.add(RECORD_WORDS).read_volatile(),
            ptr.add(RECORD_WORDS + 1).read_volatile(),
        );
        // Consume the previous breadcrumb before publishing this boot's counter.
        for i in 0..RECORD_WORDS {
            ptr.add(i).write_volatile(0);
        }
        ptr.add(RECORD_WORDS).write_volatile(0);
        ptr.add(RECORD_WORDS + 1).write_volatile(boots);
        ptr.add(RECORD_WORDS).write_volatile(COUNTER_MAGIC);
        (Breadcrumb::decode(words), boots)
    };
    BOOT.store(boots, Ordering::Relaxed);
    critical_section::with(|cs| {
        SNAPSHOT.borrow(cs).set(Snapshot {
            resetreas,
            boots,
            breadcrumb,
        })
    });
}

pub fn snapshot() -> Snapshot {
    critical_section::with(|cs| SNAPSHOT.borrow(cs).get())
}

fn halt(mut breadcrumb: Breadcrumb) -> ! {
    cortex_m::interrupt::disable();
    breadcrumb.boot = BOOT.load(Ordering::Relaxed);
    let words = breadcrumb.encode();
    // SAFETY: interrupts are masked, this handler never returns, and no references
    // to RETAINED exist. Commit magic last so an interrupted write fails validation.
    // A preempting HardFault replaces a panic record and also never returns.
    unsafe {
        let ptr = core::ptr::addr_of_mut!(RETAINED).cast::<u32>();
        ptr.write_volatile(0);
        for (i, word) in words.iter().enumerate().skip(1) {
            ptr.add(i).write_volatile(*word);
        }
        cortex_m::asm::dmb();
        ptr.write_volatile(words[0]);
        cortex_m::asm::dsb();
    }
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    let mut breadcrumb = Breadcrumb {
        kind: 1,
        ..Breadcrumb::default()
    };
    if let Some(location) = info.location() {
        breadcrumb.file_hash = file_hash(location.file());
        breadcrumb.line = location.line();
        breadcrumb.column = location.column();
    }
    halt(breadcrumb)
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    halt(Breadcrumb {
        kind: 2,
        pc: ef.pc(),
        lr: ef.lr(),
        ..Breadcrumb::default()
    })
}

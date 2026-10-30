use std::fs;
use std::io;
#[cfg(unix)]
use std::os::unix::process::CommandExt;
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub struct TempDir(pub PathBuf);

impl TempDir {
    pub fn new() -> Self {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path =
            std::env::temp_dir().join(format!("openhelmet-mesh-{}-{nonce}", std::process::id()));
        fs::create_dir(&path).unwrap();
        Self(path)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

pub struct ReapChild {
    pub child: Child,
    reaped: bool,
}

impl Drop for ReapChild {
    fn drop(&mut self) {
        if self.reaped {
            return;
        }
        // Kill the whole isolated group before waiting for its direct child.
        // Errors are best-effort; direct-child cleanup remains the fallback.
        #[cfg(unix)]
        {
            let _ = Command::new("/bin/kill")
                .args(["-KILL", "--", &format!("-{}", self.child.id())])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

pub fn spawn_group(command: &mut Command) -> io::Result<ReapChild> {
    #[cfg(unix)]
    command.process_group(0);
    Ok(ReapChild {
        child: command.spawn()?,
        reaped: false,
    })
}

pub fn wait_bounded(mut child: ReapChild, deadline: Instant) -> io::Result<ExitStatus> {
    loop {
        if let Some(status) = child.child.try_wait()? {
            child.reaped = true;
            return Ok(status);
        }
        if Instant::now() >= deadline {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "subprocess deadline exceeded",
            ));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[allow(dead_code)]
pub fn bounded(command: &mut Command, deadline: Instant) -> ExitStatus {
    wait_bounded(
        spawn_group(command).expect("could not spawn test subprocess"),
        deadline,
    )
    .expect("test subprocess wait failed")
}

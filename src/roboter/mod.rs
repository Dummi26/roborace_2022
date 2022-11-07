use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::{motors::{LargeMotor, MediumMotor}, sensors::{ColorSensor, TouchSensor, UltrasonicSensor}};

pub mod linienfolger;
pub mod erkennung;

pub struct Robot {
    // Settings
    /// How many retries will occur when any device (sensor/motor/...) fails to execute a given action, as well as how long to wait between two attempts.
    pub max_number_of_retries_on_communication_failure: (u32, Duration),

    // Threads
    /// Takes control of all motors required for driving and attempts to follow a black line on the floor based on the color sensor's readings.
    thread_linienfolger: Thread<linienfolger::Thread, linienfolger::Task, linienfolger::StopReason>,
    thread_erkennung: Thread<erkennung::Thread, erkennung::Task, erkennung::StopReason>,

    // Components
    pub motor_l1: Option<LargeMotor>,
    pub motor_l2: Option<LargeMotor>,
    pub motor_med: Option<MediumMotor>,
    pub sensor_color: Option<ColorSensor>,
    pub sensor_ultraschall: Option<UltrasonicSensor>,
    pub sensor_touch: Option<TouchSensor>,

    // PC test mode
    #[cfg(feature="pc_test")]
    pub pc_test_info: crate::test::virt_info::VirtInfo,
    #[cfg(feature="pc_test")]
    pub pc_test_thread: (std::thread::JoinHandle<()>, std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>),
}

pub enum Thread<T, S, R> {
    None,
    Initialized(T),
    Running(mpsc::Sender<S>, std::thread::JoinHandle<(T, R)>),
} impl<T, S, R> Thread<T, S, R> {
    pub fn take_thread(&mut self) -> Option<(mpsc::Sender<S>, std::thread::JoinHandle<(T, R)>)> {
        match self {
            Self::None => None,
            Self::Initialized(..) => None,
            Self::Running(..) => {
                let me = std::mem::replace(self, Self::None).assume_thread();
                Some(me)
            },
        }
    }
    pub fn assume_thread(self) -> (mpsc::Sender<S>, std::thread::JoinHandle<(T, R)>) {
        if let Self::Running(a, b) = self { (a, b) } else { panic!("Was not Self::Running!") }
    }
    pub fn sender(&self) -> Option<mpsc::Sender<S>> {
        match self {
            Self::Running(sender, _) => Some(sender.clone()),
            _ => None,
        }
    }
}

impl Robot {
    pub fn new(#[cfg(feature="pc_test")] pc_test_robot: crate::test::virt_info::VirtInfo) -> Self {
        let mut large_motors = match LargeMotor::list() { Ok(v) => v.into_iter(), Err(_) => vec![].into_iter() };
        Self {
            max_number_of_retries_on_communication_failure: (0, Duration::ZERO),
            thread_linienfolger: Thread::None,
            thread_erkennung: Thread::None,
            motor_l1: large_motors.next(),
            motor_l2: large_motors.next(),
            motor_med: MediumMotor::find().ok(),
            sensor_color: ColorSensor::find().ok(),
            sensor_ultraschall: UltrasonicSensor::find().ok(),
            sensor_touch: TouchSensor::find().ok(),
            #[cfg(feature="pc_test")]
            pc_test_thread: crate::test::virtual_robot::VirtualRobot::new(pc_test_robot.clone()),
            #[cfg(feature="pc_test")]
            pc_test_info: pc_test_robot,
        }
    }

    pub fn thread_linienfolger(&mut self) -> Result<mpsc::Sender<linienfolger::Task>, linienfolger::InitError> {
        let thread = linienfolger::Thread::init(self)?;
        let run: (_, _) = thread.run();
        let r = run.0.clone();
        self.thread_linienfolger = Thread::Running(run.0, run.1);
        Ok(r)
    }
    pub fn check_linienfolger(&mut self) -> ThreadState {
        let r = match &self.thread_linienfolger {
            Thread::None => Some(ThreadState::None),
            Thread::Initialized(..) => Some(ThreadState::Initialized),
            Thread::Running(_, t) => if t.is_finished() { None } else { Some(ThreadState::Running) },
        };
        if let Some(r) = r {
            r
        } else {
            let j = self.thread_linienfolger.take_thread().unwrap().1.join().expect("[check_linienfolger] The other thread panicked...");
            j.0.clean(self);
            println!("[check_linienfolger] The other thread stopped: {:?}", j.1);
            ThreadState::None
        }
    }

    pub fn thread_erkennung(&mut self) -> Result<mpsc::Sender<erkennung::Task>, erkennung::InitError> {
        let thread = erkennung::Thread::init(self)?;
        let run: (_, _) = thread.run();
        let r = run.0.clone();
        self.thread_erkennung = Thread::Running(run.0, run.1);
        Ok(r)
    }
    pub fn check_erkennung(&mut self) -> ThreadState {
        let r = match &self.thread_erkennung {
            Thread::None => Some(ThreadState::None),
            Thread::Initialized(..) => Some(ThreadState::Initialized),
            Thread::Running(_, t) => if t.is_finished() { None } else { Some(ThreadState::Running) },
        };
        if let Some(r) = r {
            r
        } else {
            let j = self.thread_erkennung.take_thread().unwrap().1.join().expect("[check_erkennung] The other thread panicked...");
            j.0.clean(self);
            println!("[check_erkennung] The other thread stopped: {:?}", j.1);
            ThreadState::None
        }
    }

    /// Returns (none, initialized, running) threads counts. Simply calls all self.check_[some_thread]() methods.
    pub fn check_threads(&mut self) -> (u32, u32, u32) {
        let mut none = 0;
        let mut init = 0;
        let mut runn = 0;
        for state in [
            self.check_linienfolger(),
            self.check_erkennung()
        ] {
            match state {
                ThreadState::None => none += 1,
                ThreadState::Initialized => init += 1,
                ThreadState::Running => runn += 1,
            }
        }
        (none, init, runn)
    }
}

pub enum Device {
    LargeMotor1,
    LargeMotor2,
    MediumMotor,
    ColorSensor,
    UltrasonicSensor,
}

pub enum ThreadState {
    None,
    Initialized,
    Running,
}

pub trait ThreadedFeature where Self: Sized {
    type InitError;
    fn init(robot: &mut Robot) -> Result<Self, Self::InitError>;
    fn clean(self, robot: &mut Robot);
}

pub trait ThreadedRun<T> where Self: Sized {
    fn run(self) -> T;
}

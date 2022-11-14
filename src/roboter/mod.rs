use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::{motors::{LargeMotor, MediumMotor}, sensors::{ColorSensor, GyroSensor, TouchSensor, UltrasonicSensor}};

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
    pub sensor_gyro: Option<GyroSensor>,

    pub config: (linienfolger::Config, erkennung::Config),

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
    pub fn new(config: String, #[cfg(feature="pc_test")] pc_test_robot: crate::test::virt_info::VirtInfo) -> Self {
        let config = { // Parse config file
            let chars = config.chars().into_iter();
            let mut identifier = String::new();
            let mut value = String::new();
            let mut state = State::SkipSpacesThenReadIdentifier;
            let mut identifier_state = std::collections::HashMap::<String, String>::new();
            enum State {
                SkipSpacesThenReadIdentifier,
                ReadingIdentifier,
                SkipSpacesThenReadValue,
                ReadingValue,
            }
            for ch in chars {
                match state {
                    State::SkipSpacesThenReadIdentifier => match ch {
                        ' ' | '\t' | '\n' => (), _ => { identifier.push(ch); state = State::ReadingIdentifier; },
                    },
                    State::ReadingIdentifier => match ch {
                        ':' => state = State::SkipSpacesThenReadValue,
                        _ => identifier.push(ch),
                    },
                    State::SkipSpacesThenReadValue => match ch {
                        ' ' | '\t' | '\n' => (), _ => { value.push(ch); state = State::ReadingValue; },
                    },
                    State::ReadingValue => match ch {
                        '\n' => {
                            // println!("'{}' = '{}'", identifier, value);
                            identifier_state.insert(identifier, value);
                            (identifier, value) = (String::new(), String::new());
                            state = State::SkipSpacesThenReadIdentifier;
                        },
                        _ => value.push(ch),
                    },
                }
            }
            match state {
                State::SkipSpacesThenReadIdentifier => (),
                State::ReadingIdentifier => println!("Unexpected end of robot config: EOF found after identifier '{}'!", identifier),
                State::SkipSpacesThenReadValue | State::ReadingValue => {
                    // println!("\"{}\" = \"{}\"", identifier, value);
                    identifier_state.insert(identifier, value);
                },
            }
            // Set configs
            (
                linienfolger::Config {
                    default_speed_in_percent: if let Some(v) = identifier_state.get("linienfolger default speed") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger default speed: failed to parse ({e}); [60]"); 60 } }
                    } else { println!("linienfolger default speed: [60]"); 60 },
                    importance_of_brightness_change: if let Some(v) = identifier_state.get("linienfolger importance of brightness change") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger importance of brightness change: failed to parse ({e}); [2.5]"); 2.5 } }
                    } else { println!("linienfolger importance of brightness change: [2.5]"); 2.5 },
                    angle_lane_change_start: if let Some(v) = identifier_state.get("linienfolger angle lange change start") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger angle lange change start: failed to parse ({e}); [30.0]"); 30.0 } }
                    } else { println!("linienfolger angle lange change start: [30.0]"); 30.0 },
                    seconds_to_straighten_out: if let Some(v) = identifier_state.get("linienfolger seconds to straighten out") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger seconds to straighten out: failed to parse ({e}); [0.7]"); 0.7 } }
                    } else { println!("linienfolger seconds to straighten out: [0.7]"); 0.7 },
                    angle_lane_change_final: if let Some(v) = identifier_state.get("linienfolger angle lange change final") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger angle lange change final: failed to parse ({e}); [30.0]"); 6.0 } }
                    } else { println!("linienfolger angle lange change final: [30.0]"); 6.0 },
                    brightness_entering_black_line: if let Some(v) = identifier_state.get("linienfolger brightness entering black line") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger brightness entering black line: failed to parse ({e}); [60]"); 60 } }
                    } else { println!("linienfolger brightness entering black line: [60]"); 60 },
                    angle_lane_change_encounter: if let Some(v) = identifier_state.get("linienfolger angle lange change encounter") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger angle lange change encounter: failed to parse ({e}); [2.0]"); 2.0 } }
                    } else { println!("linienfolger angle lange change encounter: [2.0]"); 2.0 },
                    brightness_on_black_line: if let Some(v) = identifier_state.get("linienfolger brightness on black line") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger brightness on black line: failed to parse ({e}); [40]"); 40 } }
                    } else { println!("linienfolger brightness on black line: [40]"); 40 },
                    brightness_after_black_line: if let Some(v) = identifier_state.get("linienfolger brightness after black line") {
                        match v.parse() { Ok(v) => v, Err(e) => { println!("linienfolger brightness after black line: failed to parse ({e}); [50]"); 50 } }
                    } else { println!("linienfolger brightness after black line: [50]"); 50 },
                },
                erkennung::Config {
                },
            )
        };
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
            sensor_gyro: GyroSensor::find().ok(),
            config,
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
    GyroSensor,
    TouchSensor,
    UltrasonicSensor,
}
impl From<&linienfolger::Device> for Device {
    fn from(dev: &linienfolger::Device) -> Self { match dev {
        linienfolger::Device::LargeMotor1 => Self::LargeMotor1,
        linienfolger::Device::LargeMotor2 => Self::LargeMotor2,
        linienfolger::Device::MediumMotor => Self::MediumMotor,
        linienfolger::Device::ColorSensor => Self::ColorSensor,
        linienfolger::Device::GyroSensor => Self::GyroSensor,
    } }
}
impl From<&erkennung::Device> for Device  {
    fn from(dev: &erkennung::Device) -> Self { match dev {
        erkennung::Device::TouchSensor => Self::TouchSensor,
        erkennung::Device::UltrasonicSensor => Self::UltrasonicSensor,
    } }
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

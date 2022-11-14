use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::sensors::{TouchSensor, UltrasonicSensor};

use super::Robot;
use super::linienfolger::Lane;
use super::linienfolger::Task as LfTask;

#[cfg(feature="pc_test")]
use crate::test::virtual_robot::VirtualRequest;

pub struct Thread {
    pub sender: mpsc::Sender<Task>,
    pub receiver: mpsc::Receiver<Task>,
    pub config: Config,
    pub send_to_linienfolger: mpsc::Sender<super::linienfolger::Task>,
    pub max_number_of_retries_on_communication_failure: (u32, Duration),
    #[cfg(not(feature="pc_test"))]
    pub sensor_distance: UltrasonicSensor,
    // #[cfg(not(feature="pc_test"))]
    // pub sensor_touch: TouchSensor,
    sleep_duration: Duration,
    #[cfg(feature="pc_test")]
    pub virtual_robot: std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>,
}
#[derive(Clone)]
pub struct Config {
    /// [50.0] The distance (in cm) where, if we detect an obstacle, we start to switch to another lane
    pub distance_obstacle: f32,
}

impl super::ThreadedFeature for Thread {
    type InitError = InitError;
    fn init(robot: &mut Robot) -> Result<Self, InitError> {
        let send_to_linienfolger = match robot.thread_linienfolger.sender() { Some(v) => v,
            None => return Err(InitError::LinienfolgerUnavailable), };
        #[cfg(not(feature="pc_test"))]
        let sensor_distance = match robot.sensor_ultraschall.take() { Some(v) => v, None => {
            return Err(InitError::MissingDevice(Device::UltrasonicSensor)); } };
        #[cfg(not(feature="pc_test"))]
        // let sensor_touch = match robot.sensor_touch.take() { Some(v) => v, None => {
        //     robot.sensor_ultraschall = Some(sensor_distance);
        //     return Err(InitError::MissingDevice(Device::TouchSensor)); } };
        #[cfg(not(feature="pc_test"))] {
            match sensor_distance.set_mode_us_dist_cm() { Ok(()) => {}, Err(_e) => {
                robot.sensor_ultraschall = Some(sensor_distance);
                // robot.sensor_touch = Some(sensor_touch);
                return Err(InitError::MissingDevice(Device::UltrasonicSensor)); } };
        }

        let (sender, receiver) = mpsc::channel();

        Ok(Self {
            sender,
            receiver,
            config: robot.config.1.clone(),
            send_to_linienfolger,
            max_number_of_retries_on_communication_failure: robot.max_number_of_retries_on_communication_failure.clone(),
            #[cfg(not(feature="pc_test"))]
            sensor_distance,
            // #[cfg(not(feature="pc_test"))]
            // sensor_touch,
            sleep_duration: Duration::from_millis(20), // max: 50Hz
            #[cfg(feature="pc_test")]
            virtual_robot: robot.pc_test_thread.1.clone(),
        })
    }
    fn clean(self, robot: &mut Robot) {
        #[cfg(not(feature="pc_test"))] {
            robot.sensor_ultraschall = Some(self.sensor_distance);
        }
    }
}

impl Thread {
    /// Attempts to read a color from the color sensor, breaking if the sensor fails to respond multiple times.
    #[cfg(not(feature="pc_test"))]
    fn read_distance_sensor(&self) -> Result<f32, StopReason> {
        let mut retries = 0;
        Ok(loop {
            match self.sensor_distance.get_distance_centimeters() { Ok(v) => break v, Err(_) => if retries < self.max_number_of_retries_on_communication_failure.0 {
                retries += 1; thread::sleep(self.max_number_of_retries_on_communication_failure.1);
            } else { return Err(StopReason::DeviceFailedToRespond(Device::UltrasonicSensor)) }, };
        })
    }
    #[cfg(feature="pc_test")]
    fn read_distance_sensor(&self) -> Result<f32, StopReason> {
        let (s, r) = std::sync::mpsc::channel();
        self.virtual_robot.send(VirtualRequest::GetDistance(s)).expect("Robot channel broke.");
        Ok(r.recv().expect("Answer channel broke."))
    }

    fn run_sync(&mut self) -> Result<(), StopReason> {
        let mut lane = { // idk which lane we start on, but linienfolger does!
            let (s, r) = std::sync::mpsc::channel();
            self.send_to_linienfolger.send(LfTask::GetLane(s)).unwrap();
            r.recv().unwrap()
        };
        loop {
            while let Ok(recv) = self.receiver.try_recv() {
                match recv {
                    Task::SetLaneTo(new_lane) => lane = new_lane,
                }
            }
            let dist = self.read_distance_sensor()?;
            // println!("{:.1}cm", dist);
            if dist < self.config.distance_obstacle {
                _ = self.send_to_linienfolger.send(LfTask::SwitchToLane(match lane {
                    Lane::Left => Lane::Center,
                    Lane::Center => Lane::Left,
                    Lane::Right => {
                        println!("HOW DID WE END UP ON THE RIGHT LANE????");
                        Lane::Right
                    },
                }));
                let delay = Duration::from_secs_f32(0.1);
                let new_lane = loop {
                    thread::sleep(delay);
                    let (s, r) = std::sync::mpsc::channel();
                    _ = self.send_to_linienfolger.send(LfTask::GetState(s));
                    match r.recv().unwrap() {
                        crate::roboter::linienfolger::LinienfolgerState::Stopped => break None,
                        crate::roboter::linienfolger::LinienfolgerState::Following(lane) => break Some(lane),
                        crate::roboter::linienfolger::LinienfolgerState::Switching(..) => continue,
                    }
                };
                if let Some(new_lane) = new_lane {
                    lane = new_lane;
                    println!("Passed the obstacle.");
                } else {
                    println!("Stopped...? [!!]");
                }
            }
            thread::sleep(self.sleep_duration);
        };
    }
}

impl super::ThreadedRun<(mpsc::Sender<Task>, std::thread::JoinHandle<(Self, StopReason)>)> for Thread {
    fn run(self) -> (mpsc::Sender<Task>, std::thread::JoinHandle<(Self, StopReason)>) {
        (self.sender.clone(), self.run())
    }
}
impl super::ThreadedRun<std::thread::JoinHandle<(Self, StopReason)>> for Thread {
    fn run(mut self) -> std::thread::JoinHandle<(Self, StopReason)> {
        std::thread::spawn(move || {
            let run = self.run_sync().unwrap_err();
            match &run {
                StopReason::DeviceFailedToRespond(..) |
                StopReason::DeviceFailedToRespondWithError(..) |
                StopReason::DeviceFailedToRespondWithErrors(..) => {},
            }
            (self, run)
        })
    }
}

pub enum Task {
    SetLaneTo(super::linienfolger::Lane),
}

#[derive(std::fmt::Debug)]
pub enum StopReason {
    DeviceFailedToRespond(Device),
    DeviceFailedToRespondWithError(Device, ev3dev_lang_rust::Ev3Error),
    DeviceFailedToRespondWithErrors(Device, Vec<ev3dev_lang_rust::Ev3Error>),
}

#[derive(std::fmt::Debug)]
pub enum InitError {
    /// Threads "reserve" devices by .take()ing them from the Option<_>s that store them. If .take() is called on the None variant, this error is thrown. This can happen when starting a thread which wants to take() a certain device after another thread has already taken it.
    MissingDevice(Device),
    /// The 'linienfolger' thread was not yet created! (Because detected obstacles are sent to the 'linienfolger' thread so we can drive around said obstacles, we need a mpsc::Sender to that thread, which only exists if the other thread is started before this one.)
    LinienfolgerUnavailable,
}

#[derive(std::fmt::Debug)]
pub enum Device {
    TouchSensor,
    UltrasonicSensor,
}

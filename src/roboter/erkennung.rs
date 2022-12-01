use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::{sensors::{TouchSensor, UltrasonicSensor}, motors::LargeMotor};

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
    pub motor_sensor: LargeMotor,
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
    /// You can specify the path here. (right instead of left | fast mode)
    pub path: Vec<(bool, bool)>,
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
        let motor_l2 = match robot.motor_l1.take() { Some(v) => v, None => {
            return Err(InitError::MissingDevice(Device::LargeMotorSensor)); } };
        #[cfg(not(feature="pc_test"))]
        // #[cfg(not(feature="pc_test"))]
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
            #[cfg(not(feature="pc_test"))]
            motor_sensor: motor_l2,
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
            robot.motor_l1 = Some(self.motor_sensor);
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


    #[cfg(not(feature="pc_test"))]
    fn set_sensor_angle(&self, angle: f32) -> Result<(), StopReason> {
        // const GEARS: f32 = 12.0 / 20.0;
        const GEARS: f32 = 20.0 / 12.0;
        let angle = (GEARS * angle).round() as i32;
        // match self.motor_sensor.run_direct() {
        //     Ok(()) => {},
        //     Err(_e) => return Err(StopReason::DeviceFailedToRespond(Device::LargeMotorSensor)),
        // }
        let max = match self.motor_sensor.get_max_speed() {
            Ok(v) => v,
            Err(_e) => return Err(StopReason::DeviceFailedToRespond(Device::LargeMotorSensor)),
        };
        match self.motor_sensor.set_speed_sp(max) {
            Ok(()) => {},
            Err(_e) => return Err(StopReason::DeviceFailedToRespond(Device::LargeMotorSensor)),
        }
        _ = self.motor_sensor.set_ramp_up_sp(0);
        self.motor_sensor.set_stop_action("hold").unwrap(); // or "brake"?
        let mut attempts = 0;
        println!("Turn by {} / {}.", angle, self.motor_sensor.get_count_per_rot().unwrap());
        loop {
            match self.motor_sensor.run_to_abs_pos(Some(angle)) {
                Ok(()) => return Ok(()),
                Err(_) => {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotorSensor));
                    }
                },
            }
        }
    }

    #[cfg(not(feature="pc_test"))]
    fn get_sensor_pos(&self) -> i32 {
        self.motor_sensor.get_position().unwrap()
    }
    

    fn run_sync(&mut self) -> Result<(), StopReason> {
        // loop { println!("Distance: {:.1}", self.read_distance_sensor()?); }
        _ = self.send_to_linienfolger.send(LfTask::SetErkennungSender(Some(self.sender.clone())));
        let mut lane = { // idk which lane we start on, but linienfolger does!
            let (s, r) = std::sync::mpsc::channel();
            self.send_to_linienfolger.send(LfTask::GetLane(s)).unwrap();
            r.recv().unwrap()
        };
        _ = self.motor_sensor.set_position(0);
        let mut predetermined_path = self.config.path.iter();
        loop {
            while let Ok(recv) = self.receiver.try_recv() {
                match recv {
                    Task::SetLaneTo(new_lane) => {
                        println!("Forced lane change! {:?}", new_lane);
                        lane = new_lane;
                    }
                }
            }
            let dist = self.read_distance_sensor()?;
            // println!("{:.1}cm", dist);
            if dist < self.config.distance_obstacle {
                let predetermined_path = predetermined_path.next();
                let should_go_right = if let Some(path) = // only Some(...) if fast mode is enabled
                    if let Some(p) = predetermined_path {
                        if p.1 { Some(p)
                        } else { None }
                    } else { None }
                {
                    path.0
                } else {
                    _ = self.send_to_linienfolger.send(LfTask::Pause(None));
                    let check_right = match lane {
                        Lane::Left => true,
                        Lane::Center => false,
                        Lane::Right => false,
                    };
                    const ANGLE: f32 = 30.0;
                    const ANGLE_CHANGE: f32 = 4.0;
                    self.set_sensor_angle(if check_right { ANGLE } else { -ANGLE })?;
                    std::thread::sleep(Duration::from_secs_f64(1.0));
                    let mut far = 0;
                    let limit = 72.5;
                    for i in 0..4 {
                        let angle = ANGLE + i as f32 * ANGLE_CHANGE;
                        self.set_sensor_angle(if check_right { angle } else { -angle })?;
                        std::thread::sleep(Duration::from_secs_f64(1.0));
                        let dist = self.read_distance_sensor()?;
                        println!("Dist: {:.1}", dist);
                        if dist > limit { far += 1; }
                    }
                    println!("Far: {}", far);
                    let should_go_right = if far >= 2 {
                        println!("No obstacle.");
                        check_right
                    } else {
                        println!("OBSTACLE!");
                        !check_right
                    };
                    self.set_sensor_angle(0.0)?;
                    // self.set_sensor_angle(if should_go_right { -90.0 } else { 90.0 })?; // look towards the obstacle that was in front of us before (so we know when it has passed)
                    println!("We should probably go {}.", if should_go_right { "right" } else { "left" });
                    if let Some(path) = predetermined_path {
                        path.0
                    } else {
                        should_go_right
                    }
                };
                let should_steer_right = if match lane {
                    Lane::Left => !should_go_right,
                    Lane::Right => should_go_right, // right + should go right => go very left instead
                    Lane::Center => false, // this is always fine
                } {
                    Some(!should_go_right) // invert if right and want to go even more right
                } else {
                    Some(should_go_right)
                };
                _ = self.send_to_linienfolger.send(LfTask::Pause(should_steer_right));
                std::thread::sleep(Duration::from_secs_f64(0.5));
                _ = self.send_to_linienfolger.send(LfTask::SwitchToLane(match lane {
                    Lane::Left => if should_go_right { Lane::Center } else { Lane::Right },
                    Lane::Center => if should_go_right { Lane::Right } else { Lane::Left },
                    Lane::Right => if should_go_right { Lane::Left } else { Lane::Center },
                }));
                let delay = Duration::from_secs_f32(0.1);
                let new_lane = loop {
                    thread::sleep(delay);
                    let (s, r) = std::sync::mpsc::channel();
                    _ = self.send_to_linienfolger.send(LfTask::GetState(s));
                    match r.recv().unwrap() {
                        crate::roboter::linienfolger::LinienfolgerState::Stopped => break None,
                        super::linienfolger::LinienfolgerState::Paused => break None,
                        crate::roboter::linienfolger::LinienfolgerState::Following(lane) => break Some(lane),
                        crate::roboter::linienfolger::LinienfolgerState::Switching(..) |
                        crate::roboter::linienfolger::LinienfolgerState::SwitchingFar(..) => {
                            if self.read_distance_sensor()? < 5.0 {
                                _ = self.send_to_linienfolger.send(LfTask::Stop);
                            }
                            continue;
                        },
                    }
                };
                thread::sleep(Duration::from_secs_f64(0.5)); // wait for the sensor to detect the obstacle
                // while self.read_distance_sensor()? < 17.5 { thread::sleep(delay); } // wait for the obstacle to end
                self.set_sensor_angle(0.0)?;
                if let Some(new_lane) = new_lane {
                    println!("Passed the obstacle. New lane: {:?}", new_lane);
                    lane = new_lane;
                } else {
                    println!("Stopped...? [!!]");
                }
                thread::sleep(Duration::from_secs_f64(2.5)); // wait (so we dont detect an obstacle twice)
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
    LargeMotorSensor,
}

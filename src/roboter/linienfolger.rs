use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::{motors::{LargeMotor, MediumMotor}, sensors::{ColorSensor, UltrasonicSensor}};

use super::Robot;

#[cfg(feature="pc_test")]
use crate::test::virtual_robot::VirtualRequest;

pub struct Thread {
    pub sender: mpsc::Sender<Task>,
    pub receiver: mpsc::Receiver<Task>,
    pub max_number_of_retries_on_communication_failure: (u32, Duration),
    #[cfg(not(feature="pc_test"))]
    pub motor_l1: LargeMotor,
    #[cfg(feature="pc_test")]
    pub motor_l1: (),
    #[cfg(not(feature="pc_test"))]
    pub motor_l2: LargeMotor,
    #[cfg(feature="pc_test")]
    pub motor_l2: (),
    #[cfg(not(feature="pc_test"))]
    pub motor_med: MediumMotor,
    #[cfg(feature="pc_test")]
    pub motor_med: (),
    #[cfg(not(feature="pc_test"))]
    pub sensor_color: ColorSensor,
    #[cfg(feature="pc_test")]
    pub sensor_color: (),
    sleep_duration: Duration,
    old_state: Option<LinienfolgerState>,
    current_state: LinienfolgerState,
    target_lane: Lane,
    target_lane_updated: bool,
    #[cfg(feature="pc_test")]
    pub virtual_robot: std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>,
}

impl super::ThreadedFeature for Thread {
    type InitError = InitError;
    fn init(robot: &mut Robot) -> Result<Self, InitError> {
        #[cfg(feature="pc_test")]
        let motor_l1 = ();
        #[cfg(not(feature="pc_test"))]
        let motor_l1 = match robot.motor_l1.take() { Some(v) => v, None => {
            return Err(InitError::MissingDevice(Device::LargeMotor1)); } };
        #[cfg(feature="pc_test")]
        let motor_l2 = ();
        #[cfg(not(feature="pc_test"))]
        let motor_l2 = match robot.motor_l2.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            return Err(InitError::MissingDevice(Device::LargeMotor2)); } };
        #[cfg(not(feature="pc_test"))]
        let motor_med = match robot.motor_med.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_l2 = Some(motor_l2);
            return Err(InitError::MissingDevice(Device::MediumMotor)); } };
        #[cfg(feature="pc_test")]
        let motor_med = ();
        #[cfg(not(feature="pc_test"))]
        let sensor_color = match robot.sensor_color.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_l2 = Some(motor_l2);
            robot.motor_med = Some(motor_med);
            return Err(InitError::MissingDevice(Device::ColorSensor)); } };
        #[cfg(feature="pc_test")]
        let sensor_color = ();
        #[cfg(not(feature="pc_test"))] {
            match sensor_color.set_mode_col_reflect() {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::ColorSensor)),
            }
            match motor_med.set_speed_sp(motor_med.get_max_speed().unwrap()) {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::MediumMotor)),
            }
            match motor_l1.run_direct() {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::LargeMotor1)),
            }
            match motor_l2.run_direct() {
                Ok(()) => {},
                Err(_e) => return Err(InitError::MissingDevice(Device::LargeMotor2)),
            }
            match motor_med.set_stop_action("hold") {
                Ok(()) => {},
                Err(e) => { println!("Could not set motor stop action: {e}"); return Err(InitError::MissingDevice(Device::MediumMotor)); }
            }
        }

        let (sender, receiver) = mpsc::channel();

        Ok(Self {
            sender,
            receiver,
            max_number_of_retries_on_communication_failure: robot.max_number_of_retries_on_communication_failure.clone(),
            motor_l1,
            motor_l2,
            motor_med,
            sensor_color,
            sleep_duration: Duration::from_millis(4), // max: 250Hz
            old_state: None,
            current_state: LinienfolgerState::Following(Lane::Center),
            target_lane: Lane::Center,
            target_lane_updated: false,
            #[cfg(feature="pc_test")]
            virtual_robot: robot.pc_test_thread.1.clone(),
        })
    }
    fn clean(self, robot: &mut Robot) {
        #[cfg(not(feature="pc_test"))] {
            robot.motor_l1 = Some(self.motor_l1);
            robot.motor_l2 = Some(self.motor_l2);
            robot.motor_med = Some(self.motor_med);
            robot.sensor_color = Some(self.sensor_color);
        }
    }
}

impl Thread {
    /// Attempts to read a color from the color sensor, breaking if the sensor fails to respond multiple times.
    #[cfg(not(feature="pc_test"))]
    fn read_color_sensor(&self) -> Result<i32, StopReason> {
        let mut retries = 0;
        Ok(loop {
            match self.sensor_color.get_color() { Ok(v) => break v, Err(_) => if retries < self.max_number_of_retries_on_communication_failure.0 {
                retries += 1; thread::sleep(self.max_number_of_retries_on_communication_failure.1);
            } else { return Err(StopReason::DeviceFailedToRespond(Device::ColorSensor)) }, };
        })
    }
    #[cfg(feature="pc_test")]
    fn read_color_sensor(&self) -> Result<i32, StopReason> {
        let (s, r) = std::sync::mpsc::channel();
        self.virtual_robot.send(VirtualRequest::ColorSensorBrightness(s)).expect("Robot channel broke.");
        Ok(r.recv().expect("Answer channel broke."))
    }

    #[cfg(not(feature="pc_test"))]
    fn set_steering_angle(&self, angle_deg: f32) -> Result<(), StopReason> {
        // println!("Steering {}°", angle_deg);
        match self.motor_med.run_to_abs_pos(Some((angle_deg * 12.0).round() as i32)) {
            Ok(_) => Ok(()),
            Err(_) => Err(StopReason::DeviceFailedToRespond(Device::MediumMotor)),
        }
    }
    #[cfg(feature="pc_test")]
    fn set_steering_angle(&self, angle_deg: f32) -> Result<(), StopReason> {
        // println!("Steering {}°", angle_deg);
        self.virtual_robot.send(VirtualRequest::SetTurningAngleDeg(angle_deg)).expect("Robot channel broke.");
        Ok(())
    }

    #[cfg(not(feature="pc_test"))]
    fn set_speed(&self, speed: i32) -> Result<(), StopReason> {
        self.motor_l1.run_direct().ok();
        self.motor_l2.run_direct().ok();
        let (speed1, speed2) = (speed, speed);
        let mut attempts = 0;
        loop {
            match (self.motor_l1.set_duty_cycle_sp(speed1), self.motor_l2.set_duty_cycle_sp(speed2)) {
                (Ok(()), Ok(())) => return Ok(()),
                (Err(_), Ok(())) => loop {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                        if self.motor_l1.set_duty_cycle_sp(speed1).is_ok() { return Ok(()); }
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotor1));
                    }
                },
                (Ok(()), Err(_)) => loop {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                        if self.motor_l2.set_duty_cycle_sp(speed2).is_ok() { return Ok(()); }
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotor2));
                    }
                },
                (Err(_), Err(_)) => {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotor1)); // TODO: Two-Device-Failure!
                    }
                },
            }
        }
    }
    #[cfg(feature="pc_test")]
    fn set_speed(&self, speed: i32) -> Result<(), StopReason> {
        self.virtual_robot.send(crate::test::virtual_robot::VirtualRequest::SetSpeed(speed));
        Ok(())
    }

    #[cfg(not(feature="pc_test"))]
    fn stop_motors(&self) {
        self.motor_l1.stop().ok();
        self.motor_l2.stop().ok();
        self.motor_med.stop().ok();
    }
    #[cfg(feature="pc_test")]
    fn stop_motors(&self) {
        println!("Stopping motors.");
    }

    fn set_state(&mut self, state: LinienfolgerState) {
        println!("Switching state: {:?}", state);
        self.old_state = Some(std::mem::replace(&mut self.current_state, state));
    }

    fn run_sync(&mut self) -> Result<(), StopReason> {
        let right_side_of_line_mode = match {
            self.stop_motors();
            let max_angle_deg = 42.5;
            thread::sleep(Duration::from_secs_f64(0.3));
            self.set_steering_angle(-max_angle_deg)?;
            thread::sleep(Duration::from_secs_f64(1.0));
            self.set_steering_angle(max_angle_deg)?;
            thread::sleep(Duration::from_secs_f64(1.0));
            self.set_steering_angle(0.0)?;
            thread::sleep(Duration::from_secs_f64(1.0));
            let mut detected_dirs = [None, None]; // false -> this direction = darker, true -> brighter
            for (i, angles) in [&[-10.0, -20.0, -30.0, -40.0], &[10.0, 20.0, 30.0, 40.0]].into_iter().enumerate() {
                let center_brightness = self.read_color_sensor()?;
                let mut prev_brightness = center_brightness;
                for angle in angles {
                    self.set_steering_angle(*angle)?;
                    thread::sleep(Duration::from_secs_f64(0.2));
                    let brightness = self.read_color_sensor()?;
                    if (brightness > prev_brightness && prev_brightness < center_brightness)
                    || (brightness < prev_brightness && prev_brightness > center_brightness) { // direction reversed => we can exit the loop early
                        break;
                    }
                    prev_brightness = brightness;
                }
                if detected_dirs[i].is_none() {
                    if prev_brightness < center_brightness {
                        detected_dirs[i] = Some(false);
                    } else if prev_brightness > center_brightness {
                        detected_dirs[i] = Some(true);
                    }
                }
                self.set_steering_angle(0.0)?;
                thread::sleep(Duration::from_secs_f64(0.5));
            }
            match detected_dirs { // false = left, true = right of line
                [None, None] => None,
                [None, Some(false)] => Some(false),
                [None, Some(true)] => Some(true),
                [Some(false), None] => Some(true),
                [Some(true), None] => Some(false),
                [Some(true), Some(false)] => Some(false),
                [Some(false), Some(true)] => Some(true),
                [Some(true), Some(true)] => None,
                [Some(false), Some(false)] => None,
            }
        } { None => { println!("Couldn't determine which side of the line I am on, using default value."); false }, Some(v) => {println!("It seems I am on the {} side of the line.", if v { "right" } else { "left" }); v } };

        // VARS

        // Following
        let mut previous_brightness = 50;
        let mut avg_change = 0.0;

        // Switching
        let mut switching_start_time = std::time::Instant::now();
        let mut switching_start_angle = 0.0;
        let mut switching_state = 0; // TODO: Enum!

        // START

        self.set_speed(30)?;

        loop {
            while let Ok(recv) = self.receiver.try_recv() {
                match recv {
                    Task::Stop => {
                        self.stop_motors();
                        self.set_steering_angle(0.0)?;
                        return Err(StopReason::RequestedStop);
                    },
                    Task::StopNoChange => {
                        return Err(StopReason::RequestedStopNoChange);
                    },
                    Task::SwitchToLane(lane) => {
                        let old_lane = std::mem::replace(&mut self.target_lane, lane.clone());
                        self.target_lane_updated = true; // TODO: is self.target_lane even necessary? It's part of Following(..) and Switching(..)!
                        self.set_state(LinienfolgerState::Switching(old_lane, lane, 0.0));
                    },
                    Task::GetLane(sender) => _ = sender.send(self.target_lane.clone()),
                }
            }

            match &self.current_state {
                LinienfolgerState::Stopped => return Err(StopReason::GracefullyStoppedForInternalReasons()),
                LinienfolgerState::Following(lane) => {
                    // Linienverfolgung
                    if true {
                        let detected_color = self.read_color_sensor()?;
                        avg_change = avg_change * 0.7 + 0.3 * ((detected_color - previous_brightness) as f32);
                        let expected_brightness_in_25ms = (detected_color + previous_brightness) as f32 / 2.0 + (2.5 /* = 25ms */ + 0.5 /* because we average with previous_brightness */) * avg_change;
                        let brightness_diff = expected_brightness_in_25ms - 50.0;
                        self.set_steering_angle(brightness_diff.min(30.0).max(-30.0) as f32)?;
                        previous_brightness = detected_color;
                    } else {
                        let detected_color = self.read_color_sensor()?;
                        // println!("[Color Sensor] read {}.", detected_color);
                        // println!("Change: {}", avg_change);

                        // Bright -> Away
                        // Dark -> Line
                        avg_change = avg_change * 0.8 + 0.2 * ((detected_color - previous_brightness) as f32 / 10.0).max(-1.0).min(1.0);
                        let brightness_diff = detected_color - 50;
                        let turn_factor = (brightness_diff as f32 / 40.0).max(-1.0).min(1.0);
                        // POSITIVE -> Steer normal, NEGATIVE -> Steer away from line
                        let change_factor = if avg_change >= 0.0 && turn_factor >= 0.0 {
                            if avg_change > 0.4 {
                                println!("going the wrong way... (line :: {:.2})", avg_change);
                            }
                            1.0 + 12.0 * avg_change * avg_change
                        } else if avg_change >= 0.0 {
                            0.5 - 1.5 * avg_change.abs()
                        } else if turn_factor >= 0.0 {
                            0.5 - 1.5 * avg_change.abs()
                        } else {
                            if -avg_change > 0.4 {
                                println!("going the wrong way... (line :: {:.2})", -avg_change);
                            }
                            1.0 + 12.0 * avg_change * avg_change
                        };
                        // println!("Factor: {:.2} | Turn: {:.2}", change_factor, turn_factor);
                        let angle = (change_factor * turn_factor * 20.0).max(-40.0).min(40.0);
                        self.set_steering_angle(if right_side_of_line_mode { -angle } else { angle })?;
                        previous_brightness = detected_color;
                    }
                },
                LinienfolgerState::Switching(prev_lane, target_lane, max_distance) => {
                    if let Some(_old_state) = self.old_state.take() {
                        switching_start_time = std::time::Instant::now();
                        switching_state = 0;
                        // begin
                        switching_start_angle =
                            match (prev_lane, target_lane) {
                                (Lane::Left, Lane::Left) => 0.0,
                                (Lane::Left, Lane::Center) => 30.0,
                                (Lane::Left, Lane::Right) => 30.0,
                                (Lane::Center, Lane::Left) => -30.0,
                                (Lane::Center, Lane::Center) => 0.0,
                                (Lane::Center, Lane::Right) => 30.0,
                                (Lane::Right, Lane::Left) => -30.0,
                                (Lane::Right, Lane::Center) => -30.0,
                                (Lane::Right, Lane::Right) => 0.0,
                            }
                        ;
                        self.set_steering_angle(switching_start_angle)?;
                    }
                    let elapsed_seconds = switching_start_time.elapsed().as_secs_f64();
                    println!("Switching State: {}", switching_state);
                    match &switching_state {
                        0 => {
                            if elapsed_seconds >= 0.5 {
                                self.set_steering_angle(0.0)?;
                                switching_state += 1;
                            }
                        },
                        1 => {
                            if elapsed_seconds >= 1.0 && self.read_color_sensor()? < 60 {
                                self.set_steering_angle(-switching_start_angle)?;
                                switching_start_time = std::time::Instant::now();
                                switching_state += 1;
                            }
                        },
                        2 => {
                            if elapsed_seconds >= 0.5 {
                                self.set_steering_angle(0.0)?;
                                switching_state += 1;
                            }
                        },
                        _ => {
                            self.set_state(LinienfolgerState::Following(target_lane.clone()));
                        },
                    }
                },
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
                StopReason::RequestedStop |
                StopReason::RequestedStopNoChange |
                StopReason::GracefullyStoppedForInternalReasons() => {},
                StopReason::DeviceFailedToRespond(..) |
                StopReason::DeviceFailedToRespondWithError(..) |
                StopReason::DeviceFailedToRespondWithErrors(..) => self.stop_motors(),
            }
            (self, run)
        })
    }
}

pub enum Task {
    /// Stops the robot, then returns.
    Stop,
    /// The thread will return almost immedeately, but the motors will not be stopped. This is not recommended.
    StopNoChange,
    SwitchToLane(Lane),
    GetLane(std::sync::mpsc::Sender<Lane>),
}

#[derive(std::fmt::Debug)]
pub enum StopReason {
    RequestedStop,
    RequestedStopNoChange,
    GracefullyStoppedForInternalReasons(),
    DeviceFailedToRespond(Device),
    DeviceFailedToRespondWithError(Device, ev3dev_lang_rust::Ev3Error),
    DeviceFailedToRespondWithErrors(Device, Vec<ev3dev_lang_rust::Ev3Error>),
}

#[derive(std::fmt::Debug)]
pub enum InitError {
    /// Threads "reserve" devices by .take()ing them from the Option<_>s that store them. If .take() is called on the None variant, this error is thrown. This can happen when starting a thread which wants to take() a certain device after another thread has already taken it.
    MissingDevice(Device),
}

#[derive(std::fmt::Debug)]
pub enum Device {
    LargeMotor1,
    LargeMotor2,
    MediumMotor,
    ColorSensor,
} impl Into<super::Device> for &Device {
    fn into(self) -> super::Device {
        match self {
            Device::LargeMotor1 => super::Device::LargeMotor1,
            Device::LargeMotor2 => super::Device::LargeMotor2,
            Device::MediumMotor => super::Device::MediumMotor,
            Device::ColorSensor => super::Device::ColorSensor,
        }
    }
}

#[derive(std::fmt::Debug)]
#[derive(Clone)]
pub enum Lane {
    Left,
    Center,
    Right,
}

#[derive(std::fmt::Debug)]
pub enum LinienfolgerState {
    Stopped,
    Following(Lane),
    /// Switching from [one lane |1] to [another |2], moving at most [3]m forward (if possible).
    Switching(Lane, Lane, f64),
}

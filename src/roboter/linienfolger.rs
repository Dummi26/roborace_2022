use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::{motors::{LargeMotor, MediumMotor}, sensors::{ColorSensor, GyroSensor}};

use super::screen::Task as ScTask;

use super::Robot;

#[cfg(feature="pc_test")]
use crate::test::virtual_robot::VirtualRequest;

pub struct Thread {
    pub sender: mpsc::Sender<Task>,
    pub receiver: mpsc::Receiver<Task>,
    screen: Option<mpsc::Sender<super::screen::Task>>,
    pub erkennung_sender: Option<mpsc::Sender<super::erkennung::Task>>,
    pub config: Config,
    pub max_number_of_retries_on_communication_failure: (u32, Duration),
    #[cfg(not(feature="pc_test"))]
    pub motor_drive: LargeMotor,
    #[cfg(not(feature="pc_test"))]
    pub motor_steer: MediumMotor,
    #[cfg(not(feature="pc_test"))]
    pub sensor_color: ColorSensor,
    #[cfg(not(feature="pc_test"))]
    pub sensor_gyro: GyroSensor,
    sleep_duration: Duration,
    old_state: Option<LinienfolgerState>,
    current_state: LinienfolgerState,
    target_lane: Lane,
    target_lane_updated: bool,
    max_speed: i32,
    #[cfg(feature="pc_test")]
    pub virtual_robot: std::sync::mpsc::Sender<crate::test::virtual_robot::VirtualRequest>,
}
#[derive(Clone)]
pub struct Config {
    /// [60] The highest speed at which the robot will move.
    pub max_speed_in_percent: i32,
    /// [0.6] How much the robot will decelerate when the color sensor deviates too far from 50% brightness. 0.0 means to always go full speed, 1.0 allows the robot to stop completely (which will cause it to swing the color sensor left and right until it finds a line. If the line is out of reach, the robot will be stuck). 0.6 Means the robots motor speed will never drop below 40% max speed in line following mode
    pub decelerate_max_factor: f64,
    /// [0.009] How fast the robot will accelerate. This is a factor that weights how far the robot needs to be on/off the line to cause significant speed changes
    pub acceleration_factor: f64,
    /// [0.012] How fast the robot will decelerate - see acceleration_factor
    pub deceleration_factor: f64,
    /// [50] The speed the robot will use when switching lanes.
    pub lane_switch_speed: i32,
    /// [30.0] Steering angle when changing from one lane to another, in degrees
    pub angle_lane_change_start: f32,
    /// [3.0] Like seconds_to_straighten_out, but this is preferred.
    pub rotations_to_straighten_out: f32,
    /// [0.7] The steering angle will be changed from the start angle to a lesser angle during this time. If this is too high, the robot will probably overshoot the line. It is more acceptable to set this to a value that is a little bit lower than the optimal one, just to be safe. This should probably be dependant on the robot's speed in the future! [TODO]
    pub seconds_to_straighten_out: f32,
    /// [6.0] After enough time has elapsed, the angle between movement direction and line will be this. Set this too high and we might miss the line, set it too low and lane changes take longer.
    pub angle_lane_change_final: f32,
    /// [60] This brightness will trigger the switch from angle_lane_change_final to angle_lane_change_encounter.
    pub brightness_entering_black_line: i32,
    /// [2.0] After detecting the black line, this is the new target angle
    pub angle_lane_change_encounter: f32,
    /// [40] Read brightness_after_black_line!
    pub brightness_on_black_line: i32,
    /// [50] If brightness was <= brightness_on_black_line and later >= this, assume we are on the outer side of the line (we have moved past the line's center) and switch back to Line-Following mode.
    pub brightness_after_black_line: i32,
}

impl super::ThreadedFeature for Thread {
    type InitError = InitError;
    fn init(robot: &mut Robot) -> Result<Self, InitError> {
        #[cfg(not(feature="pc_test"))]
        let motor_l1 = match robot.motor_l2.take() { Some(v) => v, None => {
            return Err(InitError::MissingDevice(Device::LargeMotorDrive)); } };
        #[cfg(not(feature="pc_test"))]
        let motor_med = match robot.motor_med.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            return Err(InitError::MissingDevice(Device::MediumMotor)); } };
        #[cfg(not(feature="pc_test"))]
        let sensor_color = match robot.sensor_color.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_med = Some(motor_med);
            return Err(InitError::MissingDevice(Device::ColorSensor)); } };
        #[cfg(not(feature="pc_test"))]
        let sensor_gyro = match robot.sensor_gyro.take() { Some(v) => v, None => {
            robot.motor_l1 = Some(motor_l1);
            robot.motor_med = Some(motor_med);
            robot.sensor_color = Some(sensor_color);
            return Err(InitError::MissingDevice(Device::GyroSensor)); } };
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
                Err(_e) => return Err(InitError::MissingDevice(Device::LargeMotorDrive)),
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
            screen: robot.screen.clone(),
            erkennung_sender: None,
            config: robot.config.0.clone(),
            max_number_of_retries_on_communication_failure: robot.max_number_of_retries_on_communication_failure.clone(),
            #[cfg(not(feature="pc_test"))]
            motor_drive: motor_l1,
            #[cfg(not(feature="pc_test"))]
            motor_steer: motor_med,
            #[cfg(not(feature="pc_test"))]
            sensor_color,
            #[cfg(not(feature="pc_test"))]
            sensor_gyro,
            sleep_duration: Duration::from_millis(2), // max: 500Hz (prev. 250)
            old_state: None,
            current_state: LinienfolgerState::Following(Lane::Center), // NOTE: Start lane defined here
            target_lane: Lane::Center,
            target_lane_updated: false,
            max_speed: 100,
            #[cfg(feature="pc_test")]
            virtual_robot: robot.pc_test_thread.1.clone(),
        })
    }
    fn clean(self, robot: &mut Robot) {
        #[cfg(not(feature="pc_test"))] {
            robot.motor_l2 = Some(self.motor_drive);
            robot.motor_med = Some(self.motor_steer);
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

    /// (in deg) https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html#lego-ev3-gyro
    #[cfg(not(feature="pc_test"))]
    fn read_gyro_sensor(&self) -> Result<i32, StopReason> {
        let mut retries = 0;
        Ok(loop {
            match self.sensor_gyro.get_angle() { Ok(v) => break -v, Err(_) => if retries < self.max_number_of_retries_on_communication_failure.0 {
                retries += 1; thread::sleep(self.max_number_of_retries_on_communication_failure.1);
            } else { return Err(StopReason::DeviceFailedToRespond(Device::GyroSensor)) }, };
        })
    }
    #[cfg(feature="pc_test")]
    fn read_gyro_sensor(&self) -> Result<i32, StopReason> {
        let (s, r) = std::sync::mpsc::channel();
        self.virtual_robot.send(VirtualRequest::GetGyroSensorDirection(s)).expect("Robot channel broke.");
        Ok(r.recv().expect("Answer channel broke."))
    }

    #[cfg(not(feature="pc_test"))]
    fn set_steering_angle(&self, angle_deg: f32) -> Result<(), StopReason> {
        // println!("Steering {}°", angle_deg);
        match self.motor_steer.run_to_abs_pos(Some((angle_deg * 12.0).round() as i32)) {
            Ok(_) => Ok(()),
            Err(_) => Err(StopReason::DeviceFailedToRespond(Device::MediumMotor)),
        }
    }
    // NOTE: Factor between motor and steering angle is used in 2 locations!
    fn get_steering_angle(&self) -> Result<f32, StopReason> {
        match self.motor_steer.get_position() {
            Ok(v) => Ok(v as f32 / 12.0),
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
    fn set_speed(&self, mut speed: i32) -> Result<(), StopReason> {
        if speed.abs() > self.max_speed { speed = self.max_speed * speed.signum(); }
        self.motor_drive.run_direct().ok();
        let mut attempts = 0;
        loop {
            match self.motor_drive.set_duty_cycle_sp(-speed) {
                Ok(()) => return Ok(()),
                Err(_) => {
                    if attempts < self.max_number_of_retries_on_communication_failure.0 {
                        attempts += 1;
                        std::thread::sleep(self.max_number_of_retries_on_communication_failure.1);
                    } else {
                        return Err(StopReason::DeviceFailedToRespond(Device::LargeMotorDrive));
                    }
                },
            }
        }
    }
    #[cfg(feature="pc_test")]
    fn set_speed(&self, speed: i32) -> Result<(), StopReason> {
        if speed.abs() > self.max_speed { speed = self.max_speed * speed.signum(); }
        _ = self.virtual_robot.send(crate::test::virtual_robot::VirtualRequest::SetSpeed(speed));
        Ok(())
    }

    fn set_max_speed(&mut self, factor: f32) {
        self.max_speed = (self.config.max_speed_in_percent as f32 * factor).round() as _;
    }

    #[cfg(not(feature="pc_test"))]
    fn stop_motors(&self) {
        _ = self.motor_drive.set_stop_action("brake");
        _ = self.motor_drive.stop();
        // _ = self.motor_sensor.stop();
        _ = self.motor_steer.stop();
    }
    #[cfg(feature="pc_test")]
    fn stop_motors(&self) {
        println!("Stopping motors.");
    }

    fn get_motor_rotations(&self) -> f32 { // TODO: remove unwraps
        self.motor_drive.get_position().unwrap() as f32 / self.motor_drive.get_count_per_rot().unwrap() as f32
    }

    pub fn wait_for_time(&self, duration: Duration) -> WaitFor {
        WaitFor::Time(std::time::Instant::now(), duration)
    }
    pub fn wait_for_rotations(&self, rotations: f32) -> WaitFor {
        WaitFor::Rotations(self.get_motor_rotations(), rotations)
    }
    pub fn progress(&self, wait_for: &mut WaitFor) -> Option<f32> {
        match wait_for {
            WaitFor::None => None,
            WaitFor::Time(time, duration) => {
                let elapsed = time.elapsed().as_secs_f32();
                let dur = duration.as_secs_f32();
                if elapsed >= dur {
                    *wait_for = WaitFor::None;
                }
                Some(elapsed / dur)
            },
            WaitFor::Rotations(start, goal) => {
                let rotations = (self.get_motor_rotations() - *start).abs();
                let goal = *goal;
                if rotations >= goal {
                    *wait_for = WaitFor::None;
                }
                Some(rotations / goal)
            },
        }
    }

    fn set_state(&mut self, state: LinienfolgerState) {
        println!("Switching state: {:?}", state);
        self.old_state = Some(std::mem::replace(&mut self.current_state, state));
    }

    fn run_sync(&mut self) -> Result<(), StopReason> {
        // return Err(StopReason::RequestedStop);
        // loop { thread::sleep(Duration::from_secs_f64(0.1)); println!("Angle: {}", self.read_gyro_sensor()?); }
        self.stop_motors();
        std::thread::sleep(Duration::from_secs_f64(0.5));
        #[cfg(not(feature="pc_test"))]
        match self.sensor_gyro.set_mode_gyro_cal() { Ok(_) => (), Err(_) => return Err(StopReason::DeviceFailedToRespond(Device::GyroSensor)), };
        #[cfg(not(feature="pc_test"))]
        std::thread::sleep(Duration::from_secs_f64(2.5));
        #[cfg(not(feature="pc_test"))]
        match self.sensor_gyro.set_mode_gyro_ang() { Ok(_) => (), Err(_) => return Err(StopReason::DeviceFailedToRespond(Device::GyroSensor)), };
        std::thread::sleep(Duration::from_secs_f64(0.25));
        let mut right_side_of_line_mode = match {
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
        } { None => { println!("Couldn't determine which side of the line I am on, using default value."); true }, Some(v) => {println!("It seems I am on the {} side of the line.", if v { "right" } else { "left" }); v } };

        self.set_max_speed(1.0);

        // VARS

        // Following
        let mut speed_factor = 0.0;
        // let mut previous_brightness = 50;
        // let mut avg_change = 0.0;
        let mut near_edge = 0;
        let mut near_edge_wait = WaitFor::None;

        // Switching
        let mut switching_start_time: Option<std::time::Instant> = None;
        // If this is true, switching_start_time (or motor rotations) will be used by the line follower to limit how far the steering will go.
        let mut limit_angle = false;
        let mut switching_start_angle = 0.0;
        let mut switching_state = 0; // TODO?: Enum

        // START

        // {
        //     let sleep = Duration::from_secs_f64(0.05);
        //     for prog in 0..10 {
        //         self.set_speed(prog * self.config.default_speed_in_percent / 10)?;
        //         thread::sleep(sleep);
        //     }
        //     self.set_speed(self.config.default_speed_in_percent)?;
        // }

        if let Some(screen) = &self.screen {
            _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: Some((1.0, 0.0, 0.0)), obstacles: Vec::with_capacity(0) })
        }
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
                    Task::Pause(steer) => {
                        self.stop_motors();
                        self.set_steering_angle(if let Some(s) = steer { if s { 30.0 } else { -30.0 } } else { 0.0 })?;
                        self.set_state(LinienfolgerState::Paused);
                    },
                    Task::SwitchToLane(lane) => {
                        let old_lane = std::mem::replace(&mut self.target_lane, lane.clone());
                        self.target_lane_updated = true; // TODO: is self.target_lane even necessary? It's part of Following(..) and Switching(..)!
                        self.set_state(LinienfolgerState::Switching(old_lane, lane, 0.0));
                    },
                    Task::GetLane(sender) => _ = sender.send(self.target_lane.clone()),
                    Task::GetState(sender) => _ = sender.send(self.current_state.clone()),
                    Task::SetErkennungSender(s) => self.erkennung_sender = s,
                    Task::SetMaxSpeedRel(factor) => self.set_max_speed(factor),
                    Task::SetMaxSpeedAbs(speed) => self.max_speed = speed,
                }
            }
            let mut new_state = None;
            match &self.current_state { // ~NOTE~ This is where stuff happens ~NOTE~
                LinienfolgerState::Stopped => return Err(StopReason::GracefullyStoppedForInternalReasons()),
                LinienfolgerState::Paused => {},
                LinienfolgerState::Following(lane) => {
                    // Linienverfolgung
                    {
                        let brightness = self.read_color_sensor()?;
                        // speed factor
                        let br_diff = (brightness - 50).abs() as f64 / 50.0;
                        let desired_speed_factor = 1.0 - self.config.decelerate_max_factor * br_diff;
                        speed_factor +=
                            if speed_factor < desired_speed_factor { // increasing speed [0.004 | 0.01]
                                0.009 * (desired_speed_factor - speed_factor)
                            } else { // slowing down
                                0.6 * (desired_speed_factor - speed_factor) * (desired_speed_factor - speed_factor).abs()
                            }
                        ;
                        // println!("Speed factor: {:.2}", speed_factor);
                        self.set_speed((speed_factor * self.config.max_speed_in_percent as f64).round() as _)?;
                        // steering
                        let steering_angle = self.get_steering_angle()?;
                        let max_angle = 30.0;

                        // If steering very far in one direction for a while still doesnt let me find the line, assume right_side_of_line_mode is wrong and invert it.
                        if let Some(_) = self.progress(&mut near_edge_wait) {
                            if near_edge_wait.is_none() {
                                if let Some(screen) = &self.screen { _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: Some((match lane { Lane::Left => 0.0, Lane::Center => 1.0, Lane::Right => 2.0 }, 0.2, 0.0)), obstacles: Vec::with_capacity(0) }) }
                            }
                        } else if steering_angle.abs() > 0.9 * max_angle { // near the edge
                            if near_edge < 100 {
                                near_edge += 1;
                            }
                        } else {
                            if near_edge > 0 {
                                near_edge -= 1;
                            }
                        }
                        if near_edge > 97 {
                            if let Some(screen) = &self.screen { _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: None, obstacles: Vec::with_capacity(0) }) }
                            right_side_of_line_mode = !right_side_of_line_mode;
                            println!("Switching RSOLM: {}", right_side_of_line_mode);
                            near_edge = 0;
                            near_edge_wait = self.wait_for_time(Duration::from_secs_f32(0.4)); // prevent near_edge from approaching 100 for a while because the steering mechanism takes some time to go from one extreme position to the other
                        }

                        // [ -1.0 .. +1.0 ] How much we want to steer to the right
                        let right = (if right_side_of_line_mode { 50 - brightness } else { brightness - 50 } as f32 / 30.0).min(1.0).max(-1.0);
                        if right.abs() > 0.01 { // more than 0.5% of brightness difference
                            // apply new angle [NOTE: The factor on *right* is the steering intensity
                            self.set_steering_angle((steering_angle + 0.2 * right * right.abs()).min(max_angle).max(-max_angle))?;
                        }
                    }
                    // {
                    //     let detected_brightness = self.read_color_sensor()?;
                    //     avg_change = avg_change * 0.7 + 0.3 * ((detected_brightness - previous_brightness) as f32);
                    //     let expected_brightness_soon = (detected_brightness + previous_brightness) as f32 / 2.0 + (self.config.importance_of_brightness_change + 0.5 /* because we average with previous_brightness */) * avg_change;
                    //     let brightness_diff = expected_brightness_soon - 50.0;
                    //     let max_angle = if limit_angle {
                    //         if let Some(time) = switching_start_time {
                    //             let el = time.elapsed().as_secs_f32();
                    //             if el > 0.5 { limit_angle = false; println!("[following] angle limit removed - time"); }
                    //             10.0
                    //         } else {
                    //             // TODO!
                    //             if self.motor_drive.get_position().unwrap() as f32 / self.motor_drive.get_count_per_rot().unwrap() as f32 > 0.8 {
                    //                 limit_angle = false;
                    //                 println!("[following] angle limit removed - motor rotations");
                    //             }
                    //             10.0
                    //         }
                    //     } else {
                    //         30.0
                    //     };
                    //     let steer = brightness_diff.min(max_angle).max(-max_angle) as f32;
                    //     self.set_steering_angle(if right_side_of_line_mode { -steer } else { steer })?;
                    //     previous_brightness = detected_brightness;
                    // }
                },
                LinienfolgerState::Switching(prev_lane, target_lane, m) => {
                    if let Some(_old_state) = self.old_state.take() {
                        if let Ok(_) = self.motor_drive.set_position(0) {
                            switching_start_time = None;
                        } else {
                            switching_start_time = Some(std::time::Instant::now());
                        }
                        switching_state = 0;
                        // begin
                        switching_start_angle = self.config.angle_lane_change_start *
                            match (prev_lane, target_lane) {
                                (Lane::Left, Lane::Left) => 0.0,
                                (Lane::Left, Lane::Center) => 1.0,
                                (Lane::Left, Lane::Right) => { new_state = Some(LinienfolgerState::SwitchingFar(true)); 0.0 },
                                (Lane::Center, Lane::Left) => -1.0,
                                (Lane::Center, Lane::Center) => 0.0,
                                (Lane::Center, Lane::Right) => 1.0,
                                (Lane::Right, Lane::Left) => { new_state = Some(LinienfolgerState::SwitchingFar(false)); 0.0 },
                                (Lane::Right, Lane::Center) => -1.0,
                                (Lane::Right, Lane::Right) => 0.0,
                            }
                        ;
                        println!("Switching: {:.1}", switching_start_angle);
                        if switching_start_angle == 0.0 {
                            if let None = new_state {
                                new_state = Some(LinienfolgerState::Following(target_lane.clone()));
                            } // else, a new new_state was already set
                        } else {
                            self.set_speed(self.config.lane_switch_speed)?;
                            self.set_steering_angle(switching_start_angle.min(30.0).max(-30.0))?;
                            // NOTE: This only makes sense for one-line switching (not right to left)
                            if let Some(screen) = &self.screen { _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: Some((match prev_lane { Lane::Left => 0.0, Lane::Center => 1.0, Lane::Right => 2.0 } + 0.3f32.copysign(switching_start_angle), 0.5, 0.5f32.copysign(switching_start_angle))), obstacles: Vec::with_capacity(0) }) }
                        }
                    }
                    // println!("switching_state: {}", switching_state);
                    let steer = match &switching_state {
                        0 => {
                            //                    TODO/NOTE: This value depends on the robot's speed!
                            let elapsed = if let Some(time) = switching_start_time {
                                let elapsed_seconds = time.elapsed().as_secs_f32();
                                elapsed_seconds / self.config.seconds_to_straighten_out
                            } else {
                                // TODO: Change unwrap()s!
                                let rotations = self.get_motor_rotations();
                                rotations.abs() / self.config.rotations_to_straighten_out
                            }.min(1.0);
                            if elapsed >= 1.0 {
                                self.set_steering_angle(0.0)?;
                                if let Some(_) = switching_start_time {
                                    println!("[lane switch] switch init ended: time");
                                } else {
                                    println!("[lane switch] switch init ended: motor rotations");
                                }
                                switching_state += 1;
                                if let Some(screen) = &self.screen { _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: Some((match target_lane { Lane::Left => 0.0, Lane::Center => 1.0, Lane::Right => 2.0 } - 0.4f32.copysign(switching_start_angle), 0.5, 0.25f32.copysign(switching_start_angle))), obstacles: Vec::with_capacity(0) }) }
                                Some(self.config.angle_lane_change_final * switching_start_angle.signum())
                            } else {
                                Some(
                                    (switching_start_angle * (1.0 - elapsed)
                                    + elapsed * self.config.angle_lane_change_final * switching_start_angle.signum())
                                    // .min(30.0).max(-30.0) /* since this isn't directly influencing the steering angle, we don't necessarily need this (although it might be useful) */
                                    // NOTE: I REMOVED THIS WITHOUT TESTING (but it seems to work)
                                )
                            }
                        },
                        1 => {
                            let brightness = self.read_color_sensor()?;
                            if brightness <= self.config.brightness_entering_black_line { // encountered a black line
                                if brightness <= self.config.brightness_on_black_line {
                                    println!("[lane switch] found line");
                                    if let Some(screen) = &self.screen { _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: Some((match target_lane { Lane::Left => 0.0, Lane::Center => 1.0, Lane::Right => 2.0 } - 0.2f32.copysign(switching_start_angle), 0.5, 0.2f32.copysign(switching_start_angle))), obstacles: Vec::with_capacity(0) }) }
                                    switching_state += 1;
                                }
                                Some(self.config.angle_lane_change_encounter * switching_start_angle.signum())
                            } else {
                                Some(self.config.angle_lane_change_final * switching_start_angle.signum())
                            }
                        },
                        2 => {
                            if self.read_color_sensor()? >= self.config.brightness_after_black_line { // slowly leaving black line again
                                right_side_of_line_mode = switching_start_angle.is_sign_positive();
                                println!("Setting RSOLM (switching lanes): {}", right_side_of_line_mode);
                                println!("[lane switch] leaving line");
                                if let Some(screen) = &self.screen { _ = screen.send(ScTask::ShowLines { lines: 3, robot_pos: Some((match target_lane { Lane::Left => 0.0, Lane::Center => 1.0, Lane::Right => 2.0 } + 0.2f32.copysign(switching_start_angle), 0.5, 0.0)), obstacles: Vec::with_capacity(0) }) }
                                switching_state += 1;
                                Some(0.0)
                            } else {
                                Some(self.config.angle_lane_change_encounter * switching_start_angle.signum())
                            }
                        },
                        _ => {
                            if true /* self.read_color_sensor()? <= self.config.brightness_entering_black_line */ {
                                new_state = Some(LinienfolgerState::Following(target_lane.clone()));
                                near_edge = 0;
                                near_edge_wait = self.wait_for_rotations(2.0); // TODO/NOTE: Change this value to one that works well
                                speed_factor = 0.0;
                                limit_angle = true;
                            }
                            None
                        },
                    };
                    match steer {
                        Some(dir) => {
                            let rotation_deg = 0.0; // self.read_gyro_sensor()? as f32;
                            // let rotation_off = dir - rotation_deg;
                            // println!("{} / {:.1}", rotation_deg, dir); // println!("-> {:.1}", rotation_off);
                            // let ang = 2.0 * rotation_off;
                            let ang = dir;
                            self.set_steering_angle(ang.max(-30.0).min(30.0))?;
                        },
                        None => {},
                    }
                },
                LinienfolgerState::SwitchingFar(right) => {
                    if let Some(_) = self.old_state.take() {
                        switching_state = 0;
                        self.set_steering_angle(if *right { 30.0 } else { -30.0 })?;
                        std::thread::sleep(Duration::from_secs_f64(0.75));
                        self.set_speed(self.config.lane_switch_speed)?;
                    }
                    match switching_state {
                        0 => {
                            if self.read_color_sensor()? <= self.config.brightness_entering_black_line {
                                println!("center line");
                                switching_state += 1;
                                self.set_steering_angle(if *right { -30.0 } else { 30.0 })?;
                                self.set_steering_angle(0.0)?;
                                std::thread::sleep(Duration::from_secs_f64(0.2));
                            }
                        },
                        1 => {
                            if self.read_color_sensor()? <= self.config.brightness_entering_black_line {
                                println!("final line");
                                switching_state += 1;
                            }
                        },
                        _ => {
                            new_state = Some(LinienfolgerState::Following(if *right { Lane::Right } else { Lane::Left }));
                            near_edge = 0;
                            near_edge_wait = self.wait_for_rotations(1.0); // TODO/NOTE: Change this value to one that works well
                            speed_factor = 0.0;
                            limit_angle = true;
                            right_side_of_line_mode = *right;
                            thread::sleep(Duration::from_secs_f64(1.0));
                        }
                    }
                },
            }
            if let Some(new_state) = new_state { self.set_state(new_state); }

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
    Pause(Option<bool>),
    SwitchToLane(Lane),
    GetLane(std::sync::mpsc::Sender<Lane>),
    GetState(std::sync::mpsc::Sender<LinienfolgerState>),
    SetErkennungSender(Option<mpsc::Sender<super::erkennung::Task>>),
    SetMaxSpeedRel(f32),
    SetMaxSpeedAbs(i32),
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
    LargeMotorDrive,
    LargeMotorSensor,
    MediumMotor,
    ColorSensor,
    GyroSensor,
}

#[derive(std::fmt::Debug)]
#[derive(Clone)]
pub enum Lane {
    Left,
    Center,
    Right,
}

#[derive(Clone, std::fmt::Debug)]
pub enum LinienfolgerState {
    Stopped,
    Paused,
    Following(Lane),
    /// Switching from [one lane |1] to [another |2], moving at most [3]m forward (if possible).
    Switching(Lane, Lane, f64),
    SwitchingFar(bool),
}

enum WaitFor {
    None,
    Time(std::time::Instant, Duration),
    Rotations(f32, f32),
} impl WaitFor { pub fn is_none(&self) -> bool { match self { Self::None => true, _ => false } } }

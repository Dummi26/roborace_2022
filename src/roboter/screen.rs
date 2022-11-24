use std::{sync::mpsc, thread, time::Duration};

use ev3dev_lang_rust::Screen;
use imageproc::drawing::Canvas;
use imageproc::drawing::*;

use super::Robot;

pub struct Thread {
    pub sender: mpsc::Sender<Task>,
    pub receiver: mpsc::Receiver<Task>,
    screen: Screen,
}

impl super::ThreadedFeature for Thread {
    type InitError = ();
    fn init(robot: &mut Robot) -> Result<Self, ()> {

        let (sender, receiver) = mpsc::channel();

        let screen = ev3dev_lang_rust::Screen::new().unwrap();

        Ok(Self {
            sender,
            receiver,
            screen,
        })
    }
    fn clean(self, _robot: &mut Robot) {}
}

impl Thread {
    pub fn new() -> (mpsc::Sender<Task>, thread::JoinHandle<(Thread, ())>) {
        let (sender, receiver) = mpsc::channel();
        let mut s = Self {
            sender: sender.clone(),
            receiver,
            screen: ev3dev_lang_rust::Screen::new().unwrap(),
        };
        (sender, std::thread::spawn(move || {
            _ = s.run_sync();
            (s, ())
        }))
    }
    fn clear(&mut self) {
        self.screen.clear();
    }
    fn update(&mut self) {
        self.screen.update();
    }

    fn pixel(px: u8) -> image::Rgb<u8> { image::Rgb([px, px, px]) }
    fn set(&mut self, x: u32, y: u32, px: u8) {
        self.screen.image.put_pixel(x, y, Self::pixel(px));
    }
    fn xa(x: f32) -> f32 { x * 178.0 }
    fn xr(x: f32) -> f32 { x / 178.0 }
    fn ya(y: f32) -> f32 { y * 128.0 }
    fn yr(y: f32) -> f32 { y / 128.0 }

    fn draw_line(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, px: u8) {
        imageproc::drawing::draw_line_segment_mut(&mut self.screen.image, (x1, y1), (x2, y2), Self::pixel(px));
    }
    fn draw_robot(&mut self, x: f32, y: f32, w: f32, l: f32, rot: f32) {
        let (sin, cos) = (rot.sin(), rot.cos());
        let vec_right = (
            cos * w,
            sin * l,
        );
        let vec_forward = (
            sin * w,
            - cos * l,
        );
        // rectangle: front/back/left/right
        self.draw_line(x - vec_right.0 + vec_forward.0, y - vec_right.1 + vec_forward.1, x + vec_right.0 + vec_forward.0, y + vec_right.0 + vec_forward.0, 0);
        self.draw_line(x - vec_right.0 - vec_forward.0, y - vec_right.1 - vec_forward.1, x + vec_right.0 - vec_forward.0, y + vec_right.0 - vec_forward.0, 0);
        self.draw_line(x - vec_right.0 + vec_forward.0, y - vec_right.1 + vec_forward.1, x - vec_right.0 - vec_forward.0, y - vec_right.0 - vec_forward.0, 0);
        self.draw_line(x + vec_right.0 + vec_forward.0, y + vec_right.1 + vec_forward.1, x + vec_right.0 - vec_forward.0, y + vec_right.0 - vec_forward.0, 0);
        // arrow: mid, left, right
        self.draw_line(x - 0.7 * vec_forward.0, y - 0.7 * vec_forward.1, x + 0.7 * vec_forward.0, y + 0.7 * vec_forward.1, 0);
        self.draw_line(x + 0.5 * vec_forward.0 + 0.2 * vec_forward.1, y + 0.5 * vec_forward.1 - 0.2 * vec_forward.0, x + 0.7 * vec_forward.0, y + 0.7 * vec_forward.1, 0);
        self.draw_line(x + 0.5 * vec_forward.0 - 0.2 * vec_forward.1, y + 0.5 * vec_forward.1 + 0.2 * vec_forward.0, x + 0.7 * vec_forward.0, y + 0.7 * vec_forward.1, 0);
    }

    fn run_sync(&mut self) -> Result<(), ()> {
        loop {
            match self.receiver.recv() {
                Ok(task) => match task {
                    Task::Stop => break,
                    Task::Clear => self.screen.clear(),
                    Task::ShowLines { lines, robot_pos } => {
                        self.clear();
                        let y1 = Self::ya(0.8);
                        let y2 = Self::ya(0.2);
                        for line in 0..lines {
                            let x = Self::xa(0.25 + 0.5 * line as f32 / lines as f32).round();
                            self.draw_line(x, y1.round(), x, y2.round(), 0); // black lines
                        }
                        if let Some((line, y, rot)) = robot_pos {
                            let y = (0.8 - y * 0.6).round();
                            let x = (0.25 + 0.5 * line / (lines - 1) as f32).round();
                            self.draw_robot(Self::xa(x), Self::ya(y), 15.0/*px*/, 20.0/*px*/, rot);
                        }
                        self.update();
                    },
                },
                Err(_) => break,
            }
        }
        Err(()) // because () is StopReason
    }
}

pub enum Task {
    Stop,
    Clear,
    /// How many lines there are, on what line the robot is (integer values = on the line, .5 = between), how far up the robot is, and the robot's rotation in radians, where 0.0 is up and positive values turn the robot clockwise.
    ShowLines { lines: i32, robot_pos: Option<(f32, f32, f32)> },
}
